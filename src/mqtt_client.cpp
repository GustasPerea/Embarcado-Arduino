#include "mqtt_client.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "aws_iot_config.h"
#include <Arduino.h>
#include <time.h>

static WiFiClientSecure tlsClient;
static PubSubClient client(tlsClient);
static char broker_buf[128] = "";
static char host_buf[128] = "";
static char client_id[64] = "pw_esp32";
static char mqtt_user[64] = "";
static char mqtt_pass[64] = "";
static uint16_t mqtt_port = 8883; // default AWS IoT TLS
static bool aws_endpoint = false;
static bool alpn_enabled = false;
static bool ip_logged = false;

static const char* mqttStateName(int s) {
  switch (s) {
    case -4: return "MQTT_CONNECTION_TIMEOUT";
    case -3: return "MQTT_CONNECTION_LOST";
    case -2: return "MQTT_CONNECT_FAILED";
    case -1: return "TCP_CONNECT_FAILED";
    case 0:  return "MQTT_CONNECTED";
    case 1:  return "MQTT_CONNECT_BAD_PROTOCOL";
    case 2:  return "MQTT_CONNECT_BAD_CLIENT_ID";
    case 3:  return "MQTT_CONNECT_UNAVAILABLE";
    case 4:  return "MQTT_CONNECT_BAD_CREDENTIALS";
    case 5:  return "MQTT_CONNECT_UNAUTHORIZED";
    default: return "MQTT_UNKNOWN";
  }
}


void mqtt_client_init(const char* broker, const char* clientId, const char* user, const char* pass) {
  if (!broker) return;
  strncpy(broker_buf, broker, sizeof(broker_buf)-1);
  if (clientId && *clientId) strncpy(client_id, clientId, sizeof(client_id)-1);
  if (user) strncpy(mqtt_user, user, sizeof(mqtt_user)-1);
  if (pass) strncpy(mqtt_pass, pass, sizeof(mqtt_pass)-1);
  // Support "host" or "host:port"; trim and handle IP literals to bypass DNS
  const char* colon = strchr(broker, ':');
  uint16_t port = 8883; // AWS IoT default TLS port
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
  // Persist host to a static buffer (PubSubClient stores the pointer)
  strncpy(host_buf, host, sizeof(host_buf)-1);
  host_buf[sizeof(host_buf)-1] = '\0';
  mqtt_port = port;

  // Detect AWS IoT endpoints
  aws_endpoint = (strstr(host_buf, "amazonaws.com") != nullptr) || (strstr(host_buf, "-ats.iot") != nullptr);
  alpn_enabled = false;
  ip_logged = false;

  // Load AWS certs
  tlsClient.setCACert(AWS_CERT_CA);
  tlsClient.setCertificate(AWS_CERT_CRT);
  tlsClient.setPrivateKey(AWS_CERT_PRIVATE);
  
  // Verify certificates are loaded (basic check)
  Serial.println("TLS: AWS certificates loaded");
  Serial.printf("TLS: CA cert length: %d bytes\n", strlen(AWS_CERT_CA));
  Serial.printf("TLS: Device cert length: %d bytes\n", strlen(AWS_CERT_CRT));
  Serial.printf("TLS: Private key length: %d bytes\n", strlen(AWS_CERT_PRIVATE));
  
  // More lenient TLS settings for troubleshooting
  tlsClient.setTimeout(10000);  // Increase timeout to 10s
  
  // Try to disable some strict checks that might be causing issues
  #if defined(ARDUINO_ARCH_ESP32)
  tlsClient.setHandshakeTimeout(15000); // 15s handshake timeout
  #endif
  client.setServer(host_buf, mqtt_port);
  client.setSocketTimeout(5);           // 5s MQTT socket timeout
  client.setKeepAlive(30);              // 30s keepalive
  client.setBufferSize(512);            // allow slightly larger MQTT frames
}

bool mqtt_client_connected() {
  if (client.connected()) return true;
  if (strlen(broker_buf) == 0) return false;
  // Avoid TLS handshake before time is synced (cert validity depends on RTC)
  time_t now = time(nullptr);
  if (now < 1609459200) { // 2021-01-01
    static uint32_t lastLog = 0; uint32_t ms = millis();
    if (ms - lastLog > 5000) { // log every 5s
      lastLog = ms;
      Serial.printf("Waiting for time sync before TLS connect... (current time: %ld)\n", (long)now);
    }
    return false;
  }
  
  // Log current time for debugging
  static bool timeLogged = false;
  if (!timeLogged) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    Serial.printf("TLS: Current time synced: %04d-%02d-%02d %02d:%02d:%02d\n", 
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    timeLogged = true;
  }
  // attempt reconnect
  static uint32_t lastAttempt = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastAttempt < 10000) { // throttle attempts to every 10s
    return false;
  }
  lastAttempt = nowMs;
  // One-time DNS resolution log to help diagnose TCP issues
  if (!ip_logged) {
    IPAddress ip;
    if (WiFi.hostByName(host_buf, ip)) {
      Serial.printf("MQTT: resolved %s -> %s (port %u)\n", host_buf, ip.toString().c_str(), (unsigned)mqtt_port);
    } else {
      Serial.printf("MQTT: DNS failed for %s\n", host_buf);
    }
    ip_logged = true;
  }
  bool ok = false;
  Serial.printf("TLS: Attempting MQTT connect to %s:%u with clientId '%s'\n", host_buf, mqtt_port, client_id);
  
  if (strlen(mqtt_user) && strlen(mqtt_pass)) ok = client.connect(client_id, mqtt_user, mqtt_pass);
  else ok = client.connect(client_id);
  
  if (ok) Serial.println("MQTT connected");
  else {
    int st = client.state();
    Serial.printf("MQTT connect failed, state=%d (%s)\n", st, mqttStateName(st));
#if defined(ARDUINO_ARCH_ESP32)
    // Try to print TLS stack last error if available
    char terr[160];
    int r = tlsClient.lastError(terr, sizeof(terr));
    if (r) {
      Serial.printf("TLS lastError: %s\n", terr);
    }
    
    // Additional debugging: check if we can connect to the TCP socket at all
    WiFiClient testClient;
    Serial.printf("Testing raw TCP connection to %s:%u...\n", host_buf, mqtt_port);
    if (testClient.connect(host_buf, mqtt_port)) {
      Serial.println("Raw TCP connection successful - TLS handshake is the issue");
      testClient.stop();
    } else {
      Serial.println("Raw TCP connection failed - network/firewall issue");
    }
#endif

    // If AWS IoT over 8883 failed, retry using TLS over 443 with ALPN (common when 8883 is blocked)
    if (!ok && aws_endpoint && mqtt_port == 8883) {
      Serial.println("AWS IoT: retrying via 443 with ALPN (x-amzn-mqtt-ca)");
#if defined(ARDUINO_ARCH_ESP32)
      static const char* alpnProtocols[] = { "x-amzn-mqtt-ca", nullptr };
      tlsClient.setAlpnProtocols(alpnProtocols);
      alpn_enabled = true;
#endif
      mqtt_port = 443;
      client.setServer(host_buf, mqtt_port);
      delay(50);
      ok = (strlen(mqtt_user) && strlen(mqtt_pass)) ? client.connect(client_id, mqtt_user, mqtt_pass)
                                                   : client.connect(client_id);
      if (ok) {
        Serial.println("MQTT connected over 443/ALPN");
      } else {
        int st2 = client.state();
        Serial.printf("AWS IoT 443/ALPN connect failed, state=%d (%s)\n", st2, mqttStateName(st2));
#if defined(ARDUINO_ARCH_ESP32)
        char terr2[160];
        int r2 = tlsClient.lastError(terr2, sizeof(terr2));
        if (r2) Serial.printf("TLS lastError (443): %s\n", terr2);
#endif
      }
    }
  }
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
