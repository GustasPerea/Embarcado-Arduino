#include "wifi_manager.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Arduino.h>
#include "qrcodegen.hpp"

static WebServer* server = nullptr;
static Preferences prefs;
static const char* PREF_NS = "wifi";
static char broker_buf[128] = "";
static char muser_buf[64] = "";
static char mpass_buf[64] = "";

void handleRoot() {
  String page = "<html><body><h2>Configurar Wi‑Fi e MQTT</h2>";
  page += "<p>Conecte-se ao AP PW_SETUP, acesse <a href='http://192.168.4.1'>http://192.168.4.1</a> e salve suas credenciais.</p>";
  page += "<form method=POST action=save>SSID:<input name=ssid><br>Senha:<input name=pwd type=password><br>Broker MQTT:<input name=broker value='" + String(broker_buf) + "'><br>MQTT user:<input name=muser value='" + String(muser_buf) + "'><br>MQTT pass:<input name=mpass type=password value='" + String(mpass_buf) + "'><br><input type=submit value=Salvar></form>";
  page += "<p><a href='/qr'>Ver QR (SVG)</a> com as configurações atuais</p>";
  page += "</body></html>";
  server->send(200, "text/html", page);
}

void handleSave() {
  String ssid = server->arg("ssid");
  String pwd = server->arg("pwd");
  String broker = server->arg("broker");
  String muser = server->arg("muser");
  String mpass = server->arg("mpass");
  prefs.putString("ssid", ssid);
  prefs.putString("pwd", pwd);
  prefs.putString("broker", broker);
  prefs.putString("muser", muser);
  prefs.putString("mpass", mpass);
  server->send(200, "text/html", "Saved. Rebooting...");
  delay(500);
  ESP.restart();
}

void handleQR() {
  String ssid = prefs.getString("ssid", "");
  String pwd = prefs.getString("pwd", "");
  String broker = prefs.getString("broker", "");
  String muser = prefs.getString("muser", "");
  String mpass = prefs.getString("mpass", "");
  String json = "{\"ssid\":\"" + ssid + "\",\"pwd\":\"" + pwd + "\",\"broker\":\"" + broker + "\",\"muser\":\"" + muser + "\",\"mpass\":\"" + mpass + "\"}";
  auto q = qrcodegen::QrCode::encodeText(json.c_str(), qrcodegen::QrCode::Ecc::QRECC_LOW);
  int size = q.getSize();
  String svg = "<svg xmlns='http://www.w3.org/2000/svg' shape-rendering='crispEdges' viewBox='0 0 " + String(size) + " " + String(size) + "'>";
  for (int y=0;y<size;y++) {
    for (int x=0;x<size;x++) {
      if (q.getModule(x,y)) svg += "<rect x='" + String(x) + "' y='" + String(y) + "' width='1' height='1' fill='black'/>";
    }
  }
  svg += "</svg>";
  server->send(200, "image/svg+xml", svg);
}

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
  server->on("/qr", handleQR);
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
  String broker = prefs.getString("broker", "");
  String muser = prefs.getString("muser", "");
  String mpass = prefs.getString("mpass", "");
  if (broker.length()) strncpy(broker_buf, broker.c_str(), sizeof(broker_buf)-1);
  if (muser.length()) strncpy(muser_buf, muser.c_str(), sizeof(muser_buf)-1);
  if (mpass.length()) strncpy(mpass_buf, mpass.c_str(), sizeof(mpass_buf)-1);

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

const char* wifi_manager_get_broker() {
  return broker_buf;
}

const char* wifi_manager_get_mqtt_user() {
  return muser_buf;
}

const char* wifi_manager_get_mqtt_pass() {
  return mpass_buf;
}

void wifi_manager_clear_credentials() {
  prefs.putString("ssid", "");
  prefs.putString("pwd", "");
  prefs.putString("broker", "");
  prefs.putString("muser", "");
  prefs.putString("mpass", "");
  Serial.println("Credentials cleared. Rebooting for provisioning...");
  delay(300);
  ESP.restart();
}
