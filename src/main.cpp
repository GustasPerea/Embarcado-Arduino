// Modular main.cpp que usa os módulos: flow_sensor, buzzer, hidrometer, sim7600, gps
#include <Arduino.h>
#include <time.h>
#include "flow_sensor.h"
#include "buzzer.h"
#include "hidrometer.h"
#include "sim7600.h"
#include "gps.h"
#include "esp_timer.h"
#include "wifi_manager.h"
#include "mqtt_client.h"

// ---------- Config (ajuste conforme necessário) ----------
const char* DEVICE_NAME = "PW_EMBARCADO_ARDUINO";
// ID único do dispositivo para tópicos MQTT
// Opção A) usar ID fixo: coloque USE_FIXED_DEVICE_ID=true e edite FIXED_DEVICE_ID (ex.: "device001")
// Opção B) automático: gera "deviceXXXXXX" a partir do MAC do ESP32 (padrão)
static const bool USE_FIXED_DEVICE_ID = true;
static const char* FIXED_DEVICE_ID = "0001PA-000";
static char DEVICE_ID[32] = {0};
// MQTT: defina aqui o broker e (opcional) usuário/senha
// Instruções:
// - Para broker local (ex.: seu PC rodando Mosquitto), use o IP da sua rede, ex.: "192.168.0.10"
// - Para um serviço público, use o host, ex.: "test.mosquitto.org"
// - Porta padrão usada é 1883 (configurada em mqtt_client.cpp)
// - Se o broker exigir autenticação, preencha MQTT_USER e MQTT_PASS. Caso contrário, deixe como "".
const char* MQTT_BROKER = "192.168.47.64:1883"; // altere para o IP/host do seu broker
const char* MQTT_USER   = "";             // ex.: "meuUsuario" (ou deixe vazio)
const char* MQTT_PASS   = "";             // ex.: "minhaSenha" (ou deixe vazio)

// Intervalos de envio (ajuste aqui)
const uint32_t WIFI_PUBLISH_INTERVAL_MS     = 2000;  // Wi‑Fi/MQTT: 2s por padrão
const uint32_t CELLULAR_PUBLISH_INTERVAL_MS = 30000; // Celular (SIM7600): 30s por padrão
const uint32_t GPS_UPDATE_INTERVAL_MS       = 20000; // Atualiza GPS a cada 10s para não travar o loop
const uint32_t SAVE_TOTAL_INTERVAL_MS       = 60000; // Salva total no flash a cada 30s para evitar desgaste

// Pins
const int BUZZER_PIN       = 18; // PWM output
// LED RGB (ajustado para igual ao PW_Embarcado)
const int LED_RED_GPIO     = 26;
const int LED_GREEN_GPIO   = 27;
const int LED_BLUE_GPIO    = 33;
const int RESET_BUTTON_GPIO= 4;
const int FLOW_SENSOR_PIN  = 25; // entrada do sensor de fluxo (pulsos) — igual ao PW_Embarcado

// SIM7600 pinout (module labels: G, R, T, K, V, G, S)
// Map to ESP32 pins below. Adjust as needed for your board wiring.
// ESP32 RX connects to module T (TX of module)
// ESP32 TX connects to module R (RX of module)
const int SIM7600_RX_PIN   = 16; // ESP32 RX  <- Module T
const int SIM7600_TX_PIN   = 17; // ESP32 TX  -> Module R
const int SIM7600_PWRKEY_PIN = 5;  // ESP32 GPIO -> Module K (power key)
const int SIM7600_STATUS_PIN = 34; // ESP32 GPIO <- Module S (status, input-only)

// YF-S201: ~450 pulsos por litro (f = 7.5 * Q[L/min])
const float PULSES_PER_LITER = 450.0f;

// LEDC (buzzer) config
const int LEDC_CHANNEL = 0;
const int LEDC_FREQ_HZ = 2000;
const int LEDC_RESOLUTION = 8; // 8-bit -> duty 0..255

// ---------- Globals ----------
static uint8_t led_state = 0;
static int64_t last_led_toggle = 0;
static float totalLiters = 0.0f;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static bool mqttInitDone = false;
static bool timeSynced = false;
static uint32_t lastNtpAttemptMs = 0;
static uint32_t lastCellularPublishMs = 0;
static uint32_t lastGpsUpdateMs = 0;
static uint32_t lastTotalSaveMs = 0;
// constrói DEVICE_ID em setup
static void build_device_id() {
  if (USE_FIXED_DEVICE_ID && FIXED_DEVICE_ID && FIXED_DEVICE_ID[0] != '\0') {
    strncpy(DEVICE_ID, FIXED_DEVICE_ID, sizeof(DEVICE_ID)-1);
    return;
  }
  uint64_t mac = ESP.getEfuseMac();
  // usa 3 bytes menos significativos do MAC
  uint8_t b2 = (mac >> 16) & 0xFF;
  uint8_t b1 = (mac >> 8) & 0xFF;
  uint8_t b0 = (mac >> 0) & 0xFF;
  snprintf(DEVICE_ID, sizeof(DEVICE_ID), "device%02X%02X%02X", b2, b1, b0);
}


// ---------- forward
void update_led(float flow_Lday);

void update_led(float flow_Lday) {
  int64_t now = esp_timer_get_time();

  if (flow_Lday >= 5000.0f) {
    // Verde
    digitalWrite(LED_RED_GPIO, LOW);
    digitalWrite(LED_GREEN_GPIO, HIGH);
    digitalWrite(LED_BLUE_GPIO, LOW);
    buzzer_off();
  } else if (flow_Lday >= 3000.0f) {
    // Azul
    digitalWrite(LED_RED_GPIO, LOW);
    digitalWrite(LED_GREEN_GPIO, LOW);
    digitalWrite(LED_BLUE_GPIO, HIGH);
    buzzer_off();
  } else if (flow_Lday >= 1000.0f) {
    // Vermelho fixo
    digitalWrite(LED_RED_GPIO, HIGH);
    digitalWrite(LED_GREEN_GPIO, LOW);
    digitalWrite(LED_BLUE_GPIO, LOW);
    buzzer_off();
  } else {
    // Piscar vermelho (500 ms) + buzzer junto
    if (now - last_led_toggle > 500000) { // micros
      led_state = !led_state;
      digitalWrite(LED_RED_GPIO, led_state ? HIGH : LOW);
      digitalWrite(LED_GREEN_GPIO, LOW);
      digitalWrite(LED_BLUE_GPIO, LOW);

      if (led_state) buzzer_on();
      else buzzer_off();

      last_led_toggle = now;
    }
  }
}

// ---------- FreeRTOS task ----------
void monitorTask(void* pvParameters) {
  (void) pvParameters;
  int64_t last_time = esp_timer_get_time();
  uint32_t lastPublishMs = 0;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1s loop

    // Botão reset (LOW quando pressionado)
    static uint32_t pressStart = 0;
    if (digitalRead(RESET_BUTTON_GPIO) == LOW) {
      if (pressStart == 0) pressStart = millis();
    } else if (pressStart != 0) {
      uint32_t dur = millis() - pressStart;
      pressStart = 0;
      if (dur > 3000) {
        Serial.println("Long-press: clearing WiFi credentials");
        wifi_manager_clear_credentials();
      } else {
        hidrometer_reset();
        totalLiters = 0.0f;
        Serial.println("WARN: Botão pressionado -> Reset do hidrômetro");
      }
      vTaskDelay(pdMS_TO_TICKS(500)); // debounce
    }

    int64_t now = esp_timer_get_time();
    float interval = (now - last_time) / 1000000.0f;
    if (interval <= 0.0f) interval = 1.0f;
    last_time = now;

    float increment = flow_sensor_get_increment(interval);
    totalLiters += increment;
    // Salva no flash apenas de tempos em tempos (reduz travas e desgaste)
    if (millis() - lastTotalSaveMs >= SAVE_TOTAL_INTERVAL_MS) {
      hidrometer_save_total(totalLiters);
      lastTotalSaveMs = millis();
    }

    float flow_Lmin = (increment / interval) * 60.0f;
    float flow_Lday = flow_Lmin * 60.0f * 24.0f;

    update_led(flow_Lday);

    // Local time
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);

    char gpsbuf[64];
    // Atualiza GPS apenas no intervalo configurado para evitar bloqueios longos (~6s)
    static char lastGpsCached[64] = "0.000000,0.000000";
    if (millis() - lastGpsUpdateMs >= GPS_UPDATE_INTERVAL_MS) {
      gps_get_position(lastGpsCached, sizeof(lastGpsCached));
      lastGpsUpdateMs = millis();
    }
    strncpy(gpsbuf, lastGpsCached, sizeof(gpsbuf)-1);
    gpsbuf[sizeof(gpsbuf)-1] = '\0';

    // tenta sincronizar hora via NTP: primeiro por Wi‑Fi; se não, tenta via SIM7600
    if (!timeSynced) {
      time_t nowCheck = time(NULL);
      if (nowCheck < 1609459200) { // 2021-01-01
        if (millis() - lastNtpAttemptMs > 5000) {
          lastNtpAttemptMs = millis();
          if (wifi_manager_connected()) {
            // POSIX TZ for Brazil UTC-3 (no DST). Use "<-03>3" to avoid DST rules.
            configTzTime("<-03>3", "pool.ntp.org", "time.google.com");
          } else {
            // Tenta CNTP via SIM7600
            sim7600_sync_time_via_ntp();
          }
        }
      } else {
        timeSynced = true;
      }
    }

    // publica e imprime a cada 2s
  if (millis() - lastPublishMs >= WIFI_PUBLISH_INTERVAL_MS) {
      lastPublishMs = millis();
      // JSON compacto: chaves curtas para economizar dados
      // {"id":"dev001","ts":"2025-08-31 12:34:56","l":10.362,"f":1234.56,"g":"-23.5,-46.6"}
      char payload[192];
      snprintf(payload, sizeof(payload),
        "{\"id\":\"%s\",\"ts\":\"%04d-%02d-%02d %02d:%02d:%02d\",\"l\":%.3f,\"f\":%.2f,\"g\":\"%s\"}",
        DEVICE_ID,
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec,
        totalLiters, flow_Lday, gpsbuf);

      Serial.println(payload);
  // Tópicos por dispositivo
      String topicHid = String(DEVICE_ID) + "/sensor/hidrometro";
      String topicGps = String(DEVICE_ID) + "/coordenadas/gps";

      // Prefer WiFi+MQTT quando broker definido aqui; senão fallback SIM7600
      if (wifi_manager_connected() && MQTT_BROKER[0] != '\0') {
        if (!mqttInitDone) {
          const char* useUser = (MQTT_USER[0] != '\0') ? MQTT_USER : nullptr;
          const char* usePass = (MQTT_PASS[0] != '\0') ? MQTT_PASS : nullptr;
          mqtt_client_init(MQTT_BROKER, useUser, usePass);
          mqttInitDone = true;
        }
        if (mqtt_client_connected()) {
          // mensagem principal retained (estado atual), GPS só quando mudar
          static char lastGps[64] = "";
          mqtt_client_publish(topicHid.c_str(), payload, true);
          if (strcmp(lastGps, gpsbuf) != 0) {
            mqtt_client_publish(topicGps.c_str(), gpsbuf);
            strncpy(lastGps, gpsbuf, sizeof(lastGps)-1);
          }
        } else {
          // Fallback celular mesmo com Wi‑Fi: respeita intervalo
          uint32_t nowMs = millis();
          if (nowMs - lastCellularPublishMs >= CELLULAR_PUBLISH_INTERVAL_MS) {
            lastCellularPublishMs = nowMs;
            sim7600_send(topicHid.c_str(), payload);
            // via celular, economiza: só envia GPS quando mudar
            static char lastGpsCell[64] = "";
            if (strcmp(lastGpsCell, gpsbuf) != 0) {
              sim7600_send(topicGps.c_str(), gpsbuf);
              strncpy(lastGpsCell, gpsbuf, sizeof(lastGpsCell)-1);
            }
          }
        }
      } else {
        // Sem Wi‑Fi: envia via celular como MQTT real no intervalo configurado
        uint32_t nowMs = millis();
        if (nowMs - lastCellularPublishMs >= CELLULAR_PUBLISH_INTERVAL_MS) {
          lastCellularPublishMs = nowMs;
          // Parse host:port se fornecido
          const char* brokerStr = MQTT_BROKER;
          const char* colon = strchr(brokerStr, ':');
          unsigned short port = 1883; char host[96] = {0};
          if (colon) {
            size_t hlen = (size_t)(colon - brokerStr); if (hlen >= sizeof(host)) hlen = sizeof(host)-1;
            memcpy(host, brokerStr, hlen); port = (uint16_t)atoi(colon+1);
          } else {
            strncpy(host, brokerStr, sizeof(host)-1);
          }
          sim7600_mqtt_publish(colon?host:brokerStr, port, (MQTT_USER[0]?MQTT_USER:nullptr), (MQTT_PASS[0]?MQTT_PASS:nullptr), DEVICE_ID, topicHid.c_str(), payload, true);
          static char lastGpsNoWifi[64] = "";
          if (strcmp(lastGpsNoWifi, gpsbuf) != 0) {
            sim7600_mqtt_publish(colon?host:brokerStr, port, (MQTT_USER[0]?MQTT_USER:nullptr), (MQTT_PASS[0]?MQTT_PASS:nullptr), DEVICE_ID, topicGps.c_str(), gpsbuf, false);
            strncpy(lastGpsNoWifi, gpsbuf, sizeof(lastGpsNoWifi)-1);
          }
        }
      }
    }
  }
}

// Task para loops de rede (WebServer/MQTT)
void serviceTask(void* pv) {
  (void)pv;
  for (;;) {
    wifi_manager_loop();
    mqtt_client_loop();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.printf("\n\n%s booting...\n", DEVICE_NAME);

  // Pins
  pinMode(LED_RED_GPIO, OUTPUT);
  pinMode(LED_GREEN_GPIO, OUTPUT);
  pinMode(LED_BLUE_GPIO, OUTPUT);
  pinMode(RESET_BUTTON_GPIO, INPUT_PULLUP);

  // Inits modules
  hidrometer_init();
  totalLiters = hidrometer_load_total();

  buzzer_init(BUZZER_PIN, LEDC_CHANNEL, LEDC_FREQ_HZ, LEDC_RESOLUTION);
  flow_sensor_init(FLOW_SENSOR_PIN, PULSES_PER_LITER, &mux);
  // Configure SIM7600 pins and init
  sim7600_config_pins(SIM7600_RX_PIN, SIM7600_TX_PIN, SIM7600_PWRKEY_PIN, SIM7600_STATUS_PIN);
  sim7600_init();
  // Importante: ligar o GPS somente após Serial1 estar configurada pelo sim7600_init()
  gps_init();
  // Start Wi-Fi manager (tries STA connection, or starts SoftAP provisioning)
  build_device_id();
  wifi_manager_init();

  // Garantir fuso horário correto (UTC-3, sem DST) desde o boot
  setenv("TZ", "<-03>3", 1);
  tzset();

  // Cria task principal como FreeRTOS task (pinned core 1)
  xTaskCreatePinnedToCore(
    monitorTask,
    "monitorTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  // Task de serviço para loops de rede
  xTaskCreatePinnedToCore(
    serviceTask,
    "serviceTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  // Vazio — trabalho fica nas tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}