// Modular main.cpp que usa os módulos: flow_sensor, buzzer, hidrometer, sim7600, gps
#include <Arduino.h>
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
// Defina um broker padrão (pode ficar vazio). Portal pode sobrescrever.
const char* MQTT_BROKER = ""; // ex.: "test.mosquitto.org". Vazio = usar apenas se portal fornecer

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
    hidrometer_save_total(totalLiters);

    float flow_Lmin = (increment / interval) * 60.0f;
    float flow_Lday = flow_Lmin * 60.0f * 24.0f;

    update_led(flow_Lday);

    // Local time
    time_t t = time(NULL);
    struct tm tm;
    localtime_r(&t, &tm);

    char gpsbuf[64];
    gps_get_position(gpsbuf, sizeof(gpsbuf));

    char payload[256];
    snprintf(payload, sizeof(payload), "%s %04d-%02d-%02d %02d:%02d:%02d | Hidrometro: %.3f L | Vazao Estimada: %.2f L/dia | GPS: %s",
             DEVICE_NAME,
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec,
             totalLiters, flow_Lday, gpsbuf);

    Serial.println(payload);
    // Prefer WiFi+MQTT quando broker (portal ou default) definido, senão fallback SIM7600
    const char* portalBroker = wifi_manager_get_broker();
    const char* portalUser = wifi_manager_get_mqtt_user();
    const char* portalPass = wifi_manager_get_mqtt_pass();

    bool haveBroker = (portalBroker && portalBroker[0] != '\0') || (MQTT_BROKER[0] != '\0');
    if (wifi_manager_connected() && haveBroker) {
      if (!mqttInitDone) {
        const char* useBroker = (portalBroker && portalBroker[0] != '\0') ? portalBroker : MQTT_BROKER;
        const char* useUser = (portalUser && portalUser[0] != '\0') ? portalUser : nullptr;
        const char* usePass = (portalPass && portalPass[0] != '\0') ? portalPass : nullptr;
        mqtt_client_init(useBroker, useUser, usePass);
        mqttInitDone = true;
      }
      if (mqtt_client_connected()) {
        mqtt_client_publish("/sensor/hidrometro", payload);
      } else {
        sim7600_send("/sensor/hidrometro", payload);
      }
    } else {
      sim7600_send("/sensor/hidrometro", payload);
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
  wifi_manager_init();

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