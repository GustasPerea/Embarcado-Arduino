#include "mqtt_client.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>

static WiFiClient espClient;
static PubSubClient client(espClient);
static char broker_buf[128] = "";
static char mqtt_user[64] = "";
static char mqtt_pass[64] = "";


void mqtt_client_init(const char* broker, const char* user, const char* pass) {
  if (!broker) return;
  strncpy(broker_buf, broker, sizeof(broker_buf)-1);
  if (user) strncpy(mqtt_user, user, sizeof(mqtt_user)-1);
  if (pass) strncpy(mqtt_pass, pass, sizeof(mqtt_pass)-1);
  // Support "host" or "host:port"; trim and handle IP literals to bypass DNS
  const char* colon = strchr(broker, ':');
  uint16_t port = 1883;
  char host[128] = {0};
  if (colon) {
    size_t hlen = (size_t)(colon - broker);
    if (hlen >= sizeof(host)) hlen = sizeof(host)-1;
    memcpy(host, broker, hlen);
    host[hlen] = '\0';
    port = (uint16_t)atoi(colon + 1);
  } else {
    strncpy(host, broker, sizeof(host)-1);
  }
  // Trim leading/trailing whitespace/control chars
  auto trim = [](char* s){
    size_t len = strlen(s);
    size_t i = 0; while (i < len && (unsigned char)s[i] <= 0x20) i++;
    if (i > 0) memmove(s, s + i, len - i + 1);
    len = strlen(s);
    while (len && (unsigned char)s[len-1] <= 0x20) { s[len-1] = '\0'; len--; }
  };
  trim(host);
  IPAddress ip;
  if (ip.fromString(host)) {
    client.setServer(ip, port);
  } else {
    client.setServer(host, port);
  }
}

bool mqtt_client_connected() {
  if (client.connected()) return true;
  if (strlen(broker_buf) == 0) return false;
  // attempt reconnect
  bool ok = false;
  if (strlen(mqtt_user) && strlen(mqtt_pass)) {
    ok = client.connect("pw_esp32", mqtt_user, mqtt_pass);
  } else {
    ok = client.connect("pw_esp32");
  }
  if (ok) Serial.println("MQTT connected");
  return ok;
}

void mqtt_client_publish(const char* topic, const char* payload) {
  if (!mqtt_client_connected()) return;
  client.publish(topic, payload);
}

void mqtt_client_publish(const char* topic, const char* payload, bool retained) {
  if (!mqtt_client_connected()) return;
  client.publish(topic, payload, retained);
}

void mqtt_client_loop() {
  if (client.connected()) client.loop();
}

// Build and publish telemetry JSON in the requested shape
void mqtt_client_publish_telemetry(
  const char* topic,
  const char* device_id,
  float lat,
  float lon,
  float flow_rate,
  float total_liters,
  const char* status)
{
  if (!topic || !device_id || !status) return;
  if (!mqtt_client_connected()) return;

  // Use a fixed-size buffer; adjust if you add more fields
  // Ensure enough precision for GPS
  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"device_id\":\"%s\",\"gps\":{\"lat\":%.6f,\"lon\":%.6f},\"flow_rate\":%.3f,\"total_liters\":%.3f,\"status\":\"%s\"}",
             device_id, lat, lon, flow_rate, total_liters, status);

  client.publish(topic, payload);
}
