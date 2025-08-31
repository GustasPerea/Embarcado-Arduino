#include "wifi_manager.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Arduino.h>
// ...existing code...

static WebServer* server = nullptr;
static Preferences prefs;
static const char* PREF_NS = "wifi";
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
}

void wifi_manager_loop() {
  if (server) server->handleClient();
}

void wifi_manager_init() {
  prefs.begin(PREF_NS, false);
  String ssid = prefs.getString("ssid", "");
  String pwd = prefs.getString("pwd", "");
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
  // Sync time for proper timestamps
  configTzTime("-03", "pool.ntp.org", "time.google.com");
  } else {
    Serial.println("WiFi: STA connection failed; AP permanece ativo para portal");
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
