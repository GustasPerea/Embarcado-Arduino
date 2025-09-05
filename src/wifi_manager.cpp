#include "wifi_manager.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Arduino.h>
// ...existing code...

static WebServer* server = nullptr;
static Preferences prefs;
static const char* PREF_NS = "wifi";
// Provisioning window control
static bool provisioningActive = false;
static uint32_t provisioningStartMs = 0;
static bool wifiSavedThisSession = false;
static bool wifiDisabledAfterTimeout = false;
static const uint32_t PROVISIONING_WINDOW_MS = 120000; // 2 minutes
static bool hadSavedCredentials = false;
static uint32_t lastReconnectAttemptMs = 0;
static const uint32_t WIFI_RECONNECT_INTERVAL_MS = 600000; // 10 minutes
// Broker/usuário/senha MQTT agora são definidos diretamente no código (ver main.cpp)

void handleRoot() {
  String page = "<html><body><h2>Configurar Wi‑Fi</h2>";
  page += "<p>Conecte-se ao AP PW_SETUP, acesse <a href='http://192.168.4.1'>http://192.168.4.1</a> e salve suas credenciais de Wi‑Fi.</p>";
  page += "<p>Observação: o servidor MQTT é definido no código (main.cpp). Este painel não altera o broker.</p>";
  page += "<form method=POST action=save>SSID:<input name=ssid><br>Senha:<input name=pwd type=password><br><input type=submit value=Salvar></form>";
  // ...existing code...
  page += "</body></html>";
  server->send(200, "text/html", page);
}

void handleSave() {
  String ssid = server->arg("ssid");
  String pwd = server->arg("pwd");
  prefs.putString("ssid", ssid);
  prefs.putString("pwd", pwd);
  wifiSavedThisSession = true;
  server->send(200, "text/html", "Saved. Rebooting...");
  delay(500);
  ESP.restart();
}

// ...existing code...

void wifi_manager_start_provisioning() {
  WiFi.mode(WIFI_AP);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  // Configure AP IP (optional but helps some clients)
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  // Open AP for easy first-time access as requested
  WiFi.softAP("PW_SETUP");
  server = new WebServer(80);
  server->on("/", handleRoot);
  server->on("/save", HTTP_POST, handleSave);
  // ...existing code...
  server->begin();
  Serial.println("Provisioning AP started: PW_SETUP");
  Serial.println("Portal ativo por 2 minutos para configurar o Wi‑Fi.");
  provisioningActive = true;
  provisioningStartMs = millis();
}

void wifi_manager_loop() {
  if (server) server->handleClient();

  // Auto-shutdown AP if no Wi-Fi was saved within the provisioning window
  if (provisioningActive && !wifiDisabledAfterTimeout) {
    uint32_t now = millis();
    if (!wifiSavedThisSession && (now - provisioningStartMs >= PROVISIONING_WINDOW_MS)) {
      bool staConnected = (WiFi.getMode() & WIFI_MODE_STA) && (WiFi.status() == WL_CONNECTED);
      if (staConnected) {
        Serial.println("Provisioning window expired: disabling SoftAP; mantendo STA conectado");
      } else {
        Serial.println("Provisioning window expired: disabling SoftAP and Wi-Fi (cellular only)");
      }
      if (server) {
        server->close();
        delete server;
        server = nullptr;
      }
      WiFi.softAPdisconnect(true);
      if (!staConnected) {
        WiFi.mode(WIFI_OFF); // keep only cellular path active
      } else {
        // Garante apenas STA ativo (sem AP)
        WiFi.mode(WIFI_STA);
      }
      provisioningActive = false;
      wifiDisabledAfterTimeout = true;
    }
  }

  // Periodic reconnect attempts: if we have saved credentials and we're not connected,
  // try to reconnect every WIFI_RECONNECT_INTERVAL_MS. This also handles the case
  // where the AP was disabled after the provisioning window.
  if (!wifi_manager_connected() && hadSavedCredentials) {
    uint32_t now = millis();
    if ((now - lastReconnectAttemptMs) >= WIFI_RECONNECT_INTERVAL_MS) {
      lastReconnectAttemptMs = now;
      String ssid = prefs.getString("ssid", "");
      String pwd = prefs.getString("pwd", "");
      if (ssid.length() > 0) {
        Serial.printf("WiFi: tentativa de reconectar ao SSID salvo '%s'...\n", ssid.c_str());
        // If WiFi was turned OFF after provisioning timeout, enable STA mode
        if ((WiFi.getMode() & WIFI_MODE_STA) == 0) {
          WiFi.mode(WIFI_STA);
          WiFi.persistent(false);
          WiFi.setSleep(false);
        }
        WiFi.begin(ssid.c_str(), pwd.c_str());
        // Wait a short period for connection (non-blocking long)
        uint32_t start = millis();
        int tries = 0;
        while (millis() - start < 8000) {
          if (WiFi.status() == WL_CONNECTED) break;
          delay(200);
          if (++tries % 10 == 0) Serial.print('.');
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("\nWiFi: reconectado com sucesso");
          // Re-enable AP only if provisioningActive (shouldn't be) — keep AP down
          wifiDisabledAfterTimeout = false; // keep state consistent
        } else {
          Serial.println("\nWiFi: falha ao reconectar (será tentado novamente em 10 min)");
        }
      }
    }
  }
}

void wifi_manager_init() {
  prefs.begin(PREF_NS, false);
  String ssid = prefs.getString("ssid", "");
  String pwd = prefs.getString("pwd", "");
  hadSavedCredentials = (ssid.length() != 0);
  // broker/usuario/senha MQTT não são mais configurados via painel

  // Sempre manter AP para acesso ao portal, além de STA quando possível
  wifi_manager_start_provisioning(); // inicia AP e servidor

  if (ssid.length() == 0) {
    Serial.println("WiFi: sem SSID salvo; mantendo apenas AP");
    return;
  }

  WiFi.mode(WIFI_AP_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), pwd.c_str());
  Serial.printf("Connecting to %s...\n", ssid.c_str());
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print('.');
    tries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    // Sync time for proper timestamps with multiple servers
    // POSIX TZ for Brazil UTC-3 (no DST). Note: POSIX sign is inverted.
    // "<-03>" is the display name; offset 3 means UTC-3.
    Serial.println("Syncing time with NTP servers...");
    configTzTime("<-03>3", "pool.ntp.org", "time.google.com", "a.ntp.br");
    
    // Wait a bit for time sync and log the result
    delay(2000);
    time_t now = time(nullptr);
    if (now > 1609459200) { // 2021-01-01
      struct tm timeinfo;
      localtime_r(&now, &timeinfo);
      Serial.printf("Time synced successfully: %04d-%02d-%02d %02d:%02d:%02d\n", 
                    timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
      Serial.println("Time sync may have failed, will retry later");
    }
  } else {
  Serial.println("WiFi: STA connection failed; AP permanece ativo para portal por 2 minutos");
  }
}

bool wifi_manager_connected() {
  return WiFi.status() == WL_CONNECTED;
}
// getters removidos

void wifi_manager_clear_credentials() {
  prefs.putString("ssid", "");
  prefs.putString("pwd", "");
  Serial.println("Credentials cleared. Rebooting for provisioning...");
  delay(300);
  ESP.restart();
}
