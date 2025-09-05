#include "sim7600.h"
#include <Arduino.h>
#include <string>
#include <cstring>

// Allow passing APN, APN_USER, APN_PASS without quotes via build_flags
#ifndef APN
#define APN claro.com.br
#endif
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

static int PIN_RX = 16; // ESP32 RX (connect to module T)
static int PIN_TX = 17; // ESP32 TX (connect to module R)
static int PIN_PWRKEY = -1; // ESP32 to module K
static int PIN_STATUS = -1; // ESP32 from module S
static bool s_verbose = true;
static bool s_mqtt_connected = false; // AT+CMQTT session state

void sim7600_set_verbose(bool enabled) { s_verbose = enabled; }

void sim7600_config_pins(int rx_pin, int tx_pin, int pwrkey_pin, int status_pin) {
  if (rx_pin >= 0) PIN_RX = rx_pin;
  if (tx_pin >= 0) PIN_TX = tx_pin;
  if (pwrkey_pin >= 0) PIN_PWRKEY = pwrkey_pin;
  if (status_pin >= 0) PIN_STATUS = status_pin;
}

void sim7600_power_on() {
  if (PIN_PWRKEY < 0) return;
  pinMode(PIN_PWRKEY, OUTPUT);
  digitalWrite(PIN_PWRKEY, HIGH);
  delay(10);
  // Pull low for 3s to power on (alguns módulos precisam mais tempo)
  digitalWrite(PIN_PWRKEY, LOW);
  delay(3000);
  digitalWrite(PIN_PWRKEY, HIGH);
}

bool sim7600_is_on() {
  if (PIN_STATUS < 0) return true; // assume on if no status pin
  pinMode(PIN_STATUS, INPUT);
  int v = digitalRead(PIN_STATUS);
  if (s_verbose) Serial.printf("SIM7600 STATUS pin(%d)=%d\n", PIN_STATUS, v);
  return v == HIGH;
}

// Send AT command and read raw response up to timeout.
// If requireOk=true, returns when "OK"/"ERROR" seen; else reads until timeout.
// Logs are reduced unless verbose.
static String sim7600_at(const char* cmd, uint32_t to_ms, bool requireOk = false) {
  if (s_verbose) { Serial.print("SIM7600 >> "); Serial.println(cmd); }
  while (Serial1.available()) Serial1.read(); // clear
  Serial1.println(cmd);
  String r; uint32_t start = millis();
  while (millis() - start < to_ms) {
    while (Serial1.available()) r += (char)Serial1.read();
    if (requireOk && (r.indexOf("OK") >= 0 || r.indexOf("ERROR") >= 0)) break;
    delay(10);
  }
  if (s_verbose) { Serial.print("SIM7600 << "); Serial.println(r); }
  return r;
}

void sim7600_init() {
  if (s_verbose) Serial.printf("SIM7600: init UART RX=%d TX=%d (PWRKEY=%d STATUS=%d)\n", PIN_RX, PIN_TX, PIN_PWRKEY, PIN_STATUS);
  if (PIN_PWRKEY >= 0) sim7600_power_on();
  delay(200);

  // Fixa em 115200 como no seu teste simples
  Serial1.end();
  delay(20);
  Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(50);

  // Tenta um "AT" simples sem exigir OK (alguns firmwares ecoam sem OK)
  sim7600_at("AT", 600, false);
  // Opcional: desligar eco, mas sem poluir log
  sim7600_at("ATE0", 800, true);
}

void sim7600_send(const char* topic, const char* payload) {
  Serial.println("SIM7600: Iniciando envio via rede móvel...");
  String atResp = sim7600_at("AT", 500, false);
  Serial.print("SIM7600: Resposta AT: "); Serial.println(atResp);
  String reg = sim7600_at("AT+CREG?", 1500, true);
  Serial.print("SIM7600: Resposta CREG: "); Serial.println(reg);
  if (reg.indexOf(",1") < 0 && reg.indexOf(",5") < 0) {
    Serial.println("SIM7600: Não registrado na rede móvel. Verifique o chip/SIM e sinal.");
    return;
  } else {
    Serial.println("SIM7600: Registrado na rede móvel!");
  }
  String att = sim7600_at("AT+CGATT=1", 3000, true);
  Serial.print("SIM7600: Resposta CGATT: "); Serial.println(att);
  if (att.indexOf("OK") < 0) {
    Serial.println("SIM7600: Falha ao ativar contexto de dados (CGATT). Verifique APN e sinal.");
    return;
  }
  Serial.print("SIM7600: Configurando APN: "); Serial.println(STR(APN));
  {
    String apnCmd = String("AT+CGDCONT=1,\"IP\",\"") + String(STR(APN)) + "\"";
    String apnResp = sim7600_at(apnCmd.c_str(), 2000, true);
    Serial.print("SIM7600: Resposta CGDCONT: "); Serial.println(apnResp);
    if (apnResp.indexOf("OK") < 0) {
      Serial.println("SIM7600: Falha ao configurar APN.");
      return;
    }
  }
#if defined(APN_USER) && defined(APN_PASS)
  Serial.print("SIM7600: Autenticando APN com usuário/senha...");
  Serial.print(STR(APN_USER));
  Serial.print("/");
  Serial.println(STR(APN_PASS));
  {
    String auth = String("AT+CGAUTH=1,1,\"") + String(STR(APN_USER)) + "\",\"" + String(STR(APN_PASS)) + "\"";
    String authResp = sim7600_at(auth.c_str(), 2000, true);
    Serial.print("SIM7600: Resposta CGAUTH: "); Serial.println(authResp);
    if (authResp.indexOf("OK") < 0) {
      Serial.println("SIM7600: Falha na autenticação do APN.");
      return;
    }
  }
#endif
  Serial.println("SIM7600: Abrindo rede de dados...");
  String netopen = sim7600_at("AT+NETOPEN", 4000, true);
  Serial.print("SIM7600: Resposta NETOPEN: "); Serial.println(netopen);
  if (netopen.indexOf("OK") < 0 && netopen.indexOf("Network is already opened") < 0) {
    Serial.println("SIM7600: Falha ao abrir rede de dados (NETOPEN).");
    return;
  } else if (netopen.indexOf("Network is already opened") >= 0) {
    Serial.println("SIM7600: Rede de dados já estava aberta, continuando...");
  }
  Serial.println("SIM7600: Abrindo conexão TCP com broker...");
  String cipopen = sim7600_at("AT+CIPOPEN=0,\"TCP\",\"test.mosquitto.org\",1883", 4000, true);
  Serial.print("SIM7600: Resposta CIPOPEN: "); Serial.println(cipopen);
  if (cipopen.indexOf("OK") < 0) {
    Serial.println("SIM7600: Falha ao abrir conexão TCP com broker.");
    return;
  }
  String cmd = String("AT+CIPSEND=0,") + String(strlen(payload));
  sim7600_at(cmd.c_str(), 500, false);
  Serial1.print(payload);
  delay(400);
  Serial1.println();
  delay(300);
  Serial.println("SIM7600: Payload enviado. Fechando conexão...");
  String cipclose = sim7600_at("AT+CIPCLOSE=0", 1000, true);
  Serial.print("SIM7600: Resposta CIPCLOSE: "); Serial.println(cipclose);
}

// --- MQTT over SIM7600 (simple QoS0 one-shot) ---
static void mqtt_write_uint16(uint8_t* b, uint16_t v) { b[0] = (v >> 8) & 0xFF; b[1] = v & 0xFF; }
static void mqtt_wr_str(std::string& out, const char* s) {
  uint16_t n = s ? (uint16_t)strlen(s) : 0; uint8_t len[2]; mqtt_write_uint16(len, n);
  out.append((char*)len, 2); if (n) out.append(s, n);
}

bool sim7600_mqtt_publish(
  const char* broker,
  unsigned short port,
  const char* user,
  const char* pass,
  const char* clientId,
  const char* topic,
  const char* payload,
  bool retained)
{
  if (!broker || !clientId || !topic || !payload) return false;
  Serial.printf("SIM7600: MQTT publish to %s:%u topic=%s len=%u\n", broker, (unsigned)port, topic, (unsigned)strlen(payload));
  // Ensure network open
  // Quick network status
  sim7600_at("AT+CREG?", 1500, true);
  sim7600_at("AT+CSQ", 1500, true);
  String att = sim7600_at("AT+CGATT=1", 3000, true);
  if (att.indexOf("OK") < 0) return false;
  String netopen = sim7600_at("AT+NETOPEN", 4000, true);
  if (netopen.indexOf("OK") < 0 && netopen.indexOf("Network is already opened") < 0) return false;

  // TCP open
  char openCmd[128];
  snprintf(openCmd, sizeof(openCmd), "AT+CIPOPEN=0,\"TCP\",\"%s\",%u", broker, (unsigned)port);
  String cipopen = sim7600_at(openCmd, 5000, true);
  // After OK, wait for URC +CIPOPEN: 0,0 (success) or error codes
  String urc = sim7600_at("", 5000, false);
  if (cipopen.indexOf("OK") < 0) {
    Serial.println("SIM7600: CIPOPEN did not return OK");
    return false;
  }
  if (urc.indexOf("+CIPOPEN:") >= 0) {
    // Look for status code
    int p = urc.indexOf("+CIPOPEN:");
    int comma = urc.indexOf(',', p);
    int end = (comma >= 0) ? urc.indexOf('\n', comma) : -1;
    if (comma >= 0) {
      int code = urc.substring(comma+1, (end>comma?end:urc.length())).toInt();
      if (code != 0) {
        Serial.printf("SIM7600: CIPOPEN URC error code=%d (TCP open failed)\n", code);
        return false;
      }
    }
  }

  // Build MQTT CONNECT packet
  std::string pkt;
  pkt.reserve(256);
  // Fixed header (type 1 << 4)
  pkt.push_back((char)0x10);
  // Variable header + payload
  std::string vh;
  // Protocol name "MQTT"
  mqtt_wr_str(vh, "MQTT");
  // Protocol level 4 (3.1.1)
  vh.push_back((char)0x04);
  // Connect flags
  uint8_t flags = 0x02; // CleanSession
  if (user && user[0]) flags |= 0x80; // username
  if (pass && pass[0]) flags |= 0x40; // password
  vh.push_back((char)flags);
  // Keepalive 60s
  vh.push_back((char)0x00); vh.push_back((char)0x3C);
  // Payload: ClientID, [User], [Pass]
  mqtt_wr_str(vh, clientId);
  if (user && user[0]) mqtt_wr_str(vh, user);
  if (pass && pass[0]) mqtt_wr_str(vh, pass);
  // Remaining length (MQTT varint)
  uint32_t rl = (uint32_t)vh.size();
  do { uint8_t enc = rl % 128; rl /= 128; if (rl) enc |= 128; pkt.push_back((char)enc);} while (rl);
  pkt += vh;

  // Send CONNECT
  char sendHdr[32]; snprintf(sendHdr, sizeof(sendHdr), "AT+CIPSEND=0,%u", (unsigned)pkt.size());
  sim7600_at(sendHdr, 500, false);
  // Write binary bytes to socket
  for (size_t i=0;i<pkt.size();++i) Serial1.write((uint8_t)pkt[i]);
  Serial1.flush();
  delay(200);
  // Read until CONNACK
  String rx = sim7600_at("", 1200, true);
  if (rx.indexOf("ERROR") >= 0) { sim7600_at("AT+CIPCLOSE=0", 800, true); return false; }

  // Build PUBLISH QoS0
  std::string pub;
  pub.push_back((char)(0x30 | (retained ? 0x01 : 0x00))); // qos0, retained flag
  std::string pl;
  mqtt_wr_str(pl, topic);
  // payload bytes
  pl.append(payload);
  // Remaining length varint
  uint32_t rl2 = (uint32_t)pl.size();
  do { uint8_t enc = rl2 % 128; rl2 /= 128; if (rl2) enc |= 128; pub.push_back((char)enc);} while (rl2);
  pub += pl;

  // Send PUBLISH
  snprintf(sendHdr, sizeof(sendHdr), "AT+CIPSEND=0,%u", (unsigned)pub.size());
  sim7600_at(sendHdr, 500, false);
  for (size_t i=0;i<pub.size();++i) Serial1.write((uint8_t)pub[i]);
  Serial1.flush();
  delay(150);

  // Disconnect and close
  // MQTT DISCONNECT (type 0xE0, length 0)
  const uint8_t disc[2] = {0xE0, 0x00};
  sim7600_at("AT+CIPSEND=0,2", 300, false);
  Serial1.write(disc, 2); Serial1.flush();
  delay(120);
  sim7600_at("AT+CIPCLOSE=0", 800, true);
  return true;
}

// --- MQTT persistent session via AT+CMQTT ---
static bool cmqtt_send(const char* cmd, uint32_t to_ms, bool requireOk = true) {
  String r = sim7600_at(cmd, to_ms, requireOk);
  if (!requireOk) return true;
  if (r.indexOf("OK") >= 0) return true;
  if (r.indexOf("ALREADY") >= 0 || r.indexOf("already") >= 0) return true;
  return false;
}

bool sim7600_mqtt_session_begin(
    const char* host,
    unsigned short port,
    const char* clientId,
    const char* user,
    const char* pass,
    uint16_t keepalive_sec,
    bool use_tls) {
  if (!host || !clientId) return false;
  
  Serial.printf("SIM7600: Starting MQTT session to %s:%u (TLS: %s)\n", 
                host, (unsigned)port, use_tls ? "enabled" : "disabled");
  
  // Ensure packet data
  String att = sim7600_at("AT+CGATT=1", 3000, true);
  if (att.indexOf("OK") < 0) return false;
  String netopen = sim7600_at("AT+NETOPEN", 4000, true);
  if (netopen.indexOf("OK") < 0 && netopen.indexOf("Network is already opened") < 0) return false;

  // Start MQTT stack and connect
  cmqtt_send("AT+CMQTTSTART", 5000, true); // OK or ALREADY START
  
  // Configure TLS if requested
  if (use_tls) {
    Serial.println("SIM7600: Configuring TLS for MQTT...");
    
    // Set SSL context (context 1)
    if (!cmqtt_send("AT+CSSLCFG=\"sslversion\",1,3", 2000, true)) {
      Serial.println("SIM7600: Failed to set SSL version");
      return false;
    }
    
    // Set cipher suite for AWS IoT compatibility
    if (!cmqtt_send("AT+CSSLCFG=\"ciphersuite\",1,0xFFFF", 2000, true)) {
      Serial.println("SIM7600: Failed to set cipher suite");
      return false;
    }
    
    // Enable certificate verification
    if (!cmqtt_send("AT+CSSLCFG=\"authmode\",1,2", 2000, true)) {
      Serial.println("SIM7600: Failed to set auth mode");
      return false;
    }
    
    // Configure the MQTT client to use SSL context 1
    if (!cmqtt_send("AT+CMQTTSSL=0,1,1", 2000, true)) {
      Serial.println("SIM7600: Failed to enable SSL for MQTT client");
      return false;
    }
  }
  
  // Create a client index 0 with desired clientId, clean session=1
  {
    char cmd[256]; snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s\",1", clientId);
    if (!cmqtt_send(cmd, 1500, true)) return false;
  }
  // Optional credentials must be set before CONNECT on some firmwares
  if (user && user[0]) {
    char cmd[300]; snprintf(cmd, sizeof(cmd), "AT+CMQTTUSERCFG=0,1,\"%s\",\"%s\",\"\",0,0,\"\"", user, (pass && pass[0]) ? pass : "");
    if (!cmqtt_send(cmd, 2000, true)) return false;
  }
  // Build URL: tcp://host:port or ssl://host:port
  char url[160]; 
  if (use_tls) {
    snprintf(url, sizeof(url), "ssl://%s:%u", host, (unsigned)port);
  } else {
    snprintf(url, sizeof(url), "tcp://%s:%u", host, (unsigned)port);
  }
  
  {
    char cmd[240]; snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s\",%u,1", // keepalive, clean_session=1
      url, (unsigned)keepalive_sec);
    if (!cmqtt_send(cmd, 15000, true)) { // Longer timeout for TLS handshake
      s_mqtt_connected = false; 
      Serial.println("SIM7600: MQTT connect failed");
      return false; 
    }
  }
  s_mqtt_connected = true;
  Serial.println("SIM7600: MQTT session established successfully");
  return true;
}

bool sim7600_mqtt_session_publish(const char* topic, const char* payload, bool retained) {
  if (!s_mqtt_connected || !topic || !payload) return false;
  // Topic
  {
    char cmd[220]; snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=0,%u", (unsigned)strlen(topic));
    if (!cmqtt_send(cmd, 1000, true)) return false;
    // Send topic content
    while (Serial1.available()) Serial1.read();
    Serial1.write((const uint8_t*)topic, strlen(topic));
    Serial1.flush();
    delay(50);
  }
  // Payload
  {
    char cmd[220]; snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=0,%u", (unsigned)strlen(payload));
    if (!cmqtt_send(cmd, 1000, true)) return false;
    while (Serial1.available()) Serial1.read();
    Serial1.write((const uint8_t*)payload, strlen(payload));
    Serial1.flush();
    delay(50);
  }
  // Publish QoS0
  {
    char cmd[64]; snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=0,%d,60", retained ? 1 : 0);
    if (!cmqtt_send(cmd, 4000, true)) return false;
  }
  return true;
}

bool sim7600_mqtt_session_connected() { return s_mqtt_connected; }

void sim7600_mqtt_session_end() {
  if (!s_mqtt_connected) {
    cmqtt_send("AT+CMQTTSTOP", 3000, true);
    return;
  }
  cmqtt_send("AT+CMQTTDISC=0,60", 3000, true);
  cmqtt_send("AT+CMQTTREL=0", 1500, true);
  cmqtt_send("AT+CMQTTSTOP", 3000, true);
  s_mqtt_connected = false;
}

bool sim7600_sync_time_via_ntp(const char* ntp1, const char* ntp2) {
  // Abre rede se necessário
  String att = sim7600_at("AT+CGATT=1", 3000, true);
  if (att.indexOf("OK") < 0) return false;
  String netopen = sim7600_at("AT+NETOPEN", 4000, true);
  if (netopen.indexOf("OK") < 0 && netopen.indexOf("Network is already opened") < 0) return false;
  // Configura servidores NTP
  String set1 = String("AT+CNTP=\"") + (ntp1?ntp1:"pool.ntp.org") + "\",0";
  sim7600_at(set1.c_str(), 1200, true);
  // Tenta update
  String upd = sim7600_at("AT+CNTP", 8000, true);
  if (upd.indexOf("OK") < 0) { Serial.println("SIM7600: CNTP falhou"); return false; }
  // Lê hora da rede
  String clk = sim7600_at("AT+CCLK?", 1200, true);
  int p = clk.indexOf("\"");
  if (p >= 0) {
    int q = clk.indexOf("\"", p+1);
    if (q > p) {
      String ts = clk.substring(p+1, q); // "yy/MM/dd,HH:MM:SS±zz"
      // Parse para time_t UTC
      int yy = ts.substring(0,2).toInt();
      int MM = ts.substring(3,5).toInt();
      int dd = ts.substring(6,8).toInt();
      int hh = ts.substring(9,11).toInt();
      int mm = ts.substring(12,14).toInt();
      int ss = ts.substring(15,17).toInt();
      int tzSign = (ts.length() >= 19 && (ts[17] == '+' || ts[17] == '-')) ? (ts[17] == '-' ? -1 : 1) : 1;
      int tzVal = (ts.length() >= 20) ? ts.substring(18).toInt() : 0; // pode ser horas (ex: 03) ou quartes (ex: 12=3h)
      int tzHours = (abs(tzVal) > 12) ? (tzVal / 4) : tzVal; // heurística: >12 => unidades de 15 min
      // Monta tm como horário local do modem
  struct tm tml{};
      tml.tm_year = 2000 + yy - 1900;
      tml.tm_mon  = MM - 1;
      tml.tm_mday = dd;
      tml.tm_hour = hh;
      tml.tm_min  = mm;
      tml.tm_sec  = ss;
  // Converte para epoch assumindo que tml é horário local do modem (antes do offset). Para obter UTC:
  // Faz mktime em UTC forçando TZ=UTC temporariamente
  char* oldtz = getenv("TZ");
  setenv("TZ", "UTC0", 1); tzset();
  time_t local = mktime(&tml);
  if (oldtz) setenv("TZ", oldtz, 1); else unsetenv("TZ");
  tzset();
      // Ajusta para UTC real subtraindo o offset local
      time_t utc = local - (tzSign * tzHours * 3600);
      struct timeval tv; tv.tv_sec = utc; tv.tv_usec = 0;
      settimeofday(&tv, nullptr);
      Serial.printf("SIM7600: Hora atualizada via CNTP/CCLK -> %04d-%02d-%02d %02d:%02d:%02d (UTC)\n",
                    tml.tm_year+1900, tml.tm_mon+1, tml.tm_mday, tml.tm_hour, tml.tm_min, tml.tm_sec);
      return true;
    }
  }
  return false;
}

// --- GPS support ---
static bool parse_cgpsinfo(const String& line, float* plat, float* plon) {
  // +CGPSINFO: lat,NS,lon,EW,date,UTC,alt,spd
  int p = line.indexOf("+CGPSINFO:");
  if (p < 0) return false;
  String s = line.substring(p + 10);
  s.trim();
  int c1 = s.indexOf(','); if (c1 < 0) return false;
  int c2 = s.indexOf(',', c1+1); if (c2 < 0) return false;
  int c3 = s.indexOf(',', c2+1); if (c3 < 0) return false;
  int c4 = s.indexOf(',', c3+1); if (c4 < 0) return false;
  String lat = s.substring(0, c1);
  String ns  = s.substring(c1+1, c2);
  String lon = s.substring(c2+1, c3);
  String ew  = s.substring(c3+1, c4);
  if (lat.length() < 3 || lon.length() < 4) return false;

  auto nmeaToDec = [](const String& nmea, const String& dir) -> float {
    int dot = nmea.indexOf('.');
    if (dot < 0) return 0.0f;
    int degLen = dot - 2;
    float deg = nmea.substring(0, degLen).toFloat();
    float min = nmea.substring(degLen).toFloat();
    float val = deg + (min / 60.0f);
    if (dir == "S" || dir == "W") val = -val;
    return val;
  };

  float flat = nmeaToDec(lat, ns);
  float flon = nmeaToDec(lon, ew);
  if (plat) *plat = flat;
  if (plon) *plon = flon;
  return true;
}

// Fallback parser for +CGNSINF
static bool parse_cgnsinf(const String& line, float* plat, float* plon) {
  int p = line.indexOf("+CGNSINF:");
  if (p < 0) return false;
  String s = line.substring(p + 9);
  s.trim();
  float flat = 0.0f, flon = 0.0f;
  int fieldIndex = 0;
  int start = 0;
  for (int i = 0; i <= s.length(); ++i) {
    if (i == s.length() || s[i] == ',') {
      String tok = s.substring(start, i);
      tok.trim();
      if (fieldIndex == 3) flat = tok.toFloat();
      else if (fieldIndex == 4) flon = tok.toFloat();
      fieldIndex++;
      start = i + 1;
    }
  }
  if (flat == 0.0f && flon == 0.0f) return false;
  if (plat) *plat = flat;
  if (plon) *plon = flon;
  return true;
}

bool sim7600_gps_on() {
  // Usa o comando simples que funcionou no seu teste
  String r = sim7600_at("AT+CGPS=1", 1500, true);
  if (r.indexOf("OK") < 0) {
    // Tenta modo (1,1) se necessário
    r = sim7600_at("AT+CGPS=1,1", 1800, true);
  }
  // Se ainda falhar, tenta pilha GNSS unificada
  if (r.indexOf("OK") < 0) {
    r = sim7600_at("AT+CGNSPWR=1", 1500, true);
  }
  delay(200);
  sim7600_at("AT+CGPS?", 800, false);
  // Aguarda um pouco para o subsistema GNSS iniciar
  delay(500);
  return true; // não força OK; segue a mesma estratégia do teste simples
}

bool sim7600_gps_off() {
  String r = sim7600_at("AT+CGPS=0", 1200, true);
  return r.indexOf("OK") >= 0;
}

bool sim7600_gps_get_fix(float* lat, float* lon) {
  // Estratégia do seu teste: enviar e ler cru por ~2.5s para capturar a linha completa
  String r = sim7600_at("AT+CGPSINFO", 2500, false);
  if (!parse_cgpsinfo(r, lat, lon)) {
    delay(500);
    r = sim7600_at("AT+CGPSINFO", 2000, false);
  }
  if (parse_cgpsinfo(r, lat, lon)) return true;
  // Fallback CGNSINF
  r = sim7600_at("AT+CGNSINF", 2000, false);
  return parse_cgnsinf(r, lat, lon);
}

bool sim7600_gps_status(char* buffer, size_t len) {
  // Primeiro, tenta o status via CGPSSTATUS (pilha CGPS)
  String r = sim7600_at("AT+CGPSSTATUS?", 2500, true);
  bool fixed = false;

  // Prefer the last occurrence in case of echoed lines
  int p = r.lastIndexOf("+CGPSSTATUS:");
  String s;
  if (p >= 0 && r.indexOf("ERROR") < 0) {
    s = r.substring(p + 12);
    int nl = s.indexOf('\n');
    int cr = s.indexOf('\r');
    int cut = (nl < 0) ? cr : ((cr < 0) ? nl : (nl < cr ? nl : cr));
    if (cut >= 0) s = s.substring(0, cut);
    s.trim();
    if (s.indexOf("2D Fix") >= 0 || s.indexOf("3D Fix") >= 0) fixed = true;
    if (buffer && len) s.toCharArray(buffer, len);
    return fixed;
  }

  // Fallback: usar pilha GNSS unificada (CGNS)
  String pwr = sim7600_at("AT+CGNSPWR?", 1200, true);
  int pwri = pwr.indexOf("+CGNSPWR:");
  int gnssOn = 0;
  if (pwri >= 0) {
    String tail = pwr.substring(pwri + 9);
    tail.trim();
    gnssOn = tail.toInt();
  }
  String inf = sim7600_at("AT+CGNSINF", 2000, true);
  int gi = inf.indexOf("+CGNSINF:");
  int run = 0, fix = 0;
  if (gi >= 0) {
    String line = inf.substring(gi + 9);
    line.trim();
    // Extrai os dois primeiros campos: run status, fix status
    int c1 = line.indexOf(',');
    int c2 = (c1 >= 0) ? line.indexOf(',', c1+1) : -1;
    if (c1 > 0) run = line.substring(0, c1).toInt();
    if (c2 > c1) fix = line.substring(c1+1, c2).toInt();
    // Monta string amigável
    String ss = String("GNSSPWR=") + gnssOn + ", CGNSINF run=" + run + ", fix=" + fix;
    if (buffer && len) ss.toCharArray(buffer, len);
    return fix == 1;
  }

  // Se nada funcionou, devolve "ERROR"
  if (buffer && len) strncpy(buffer, "ERROR", len);
  return false;
}

void sim7600_gps_cold_reset() {
  sim7600_at("AT+CGPSRST=1", 1200, false);
}

// --- TLS Certificate management ---
bool sim7600_tls_configure_certs(const char* ca_cert, const char* client_cert, const char* private_key) {
  if (!ca_cert || !client_cert || !private_key) {
    Serial.println("SIM7600: Invalid certificate parameters");
    return false;
  }
  
  Serial.println("SIM7600: Configuring TLS certificates...");
  
  // Clear any existing certificates first
  sim7600_tls_clear_certs();
  
  // Configure CA certificate (context 1, CA cert)
  if (!cmqtt_send("AT+CCERTDOWN=\"ca_cert.pem\",1", 3000, true)) {
    Serial.println("SIM7600: Failed to start CA cert download");
    return false;
  }
  
  // Send CA certificate data
  Serial1.print(ca_cert);
  delay(500);
  
  // Configure client certificate
  if (!cmqtt_send("AT+CCERTDOWN=\"client_cert.pem\",2", 3000, true)) {
    Serial.println("SIM7600: Failed to start client cert download");
    return false;
  }
  
  // Send client certificate data
  Serial1.print(client_cert);
  delay(500);
  
  // Configure private key
  if (!cmqtt_send("AT+CCERTDOWN=\"private_key.pem\",3", 3000, true)) {
    Serial.println("SIM7600: Failed to start private key download");
    return false;
  }
  
  // Send private key data
  Serial1.print(private_key);
  delay(500);
  
  // Configure SSL context to use the certificates
  if (!cmqtt_send("AT+CSSLCFG=\"cacert\",1,\"ca_cert.pem\"", 2000, true)) {
    Serial.println("SIM7600: Failed to set CA cert");
    return false;
  }
  
  if (!cmqtt_send("AT+CSSLCFG=\"clientcert\",1,\"client_cert.pem\"", 2000, true)) {
    Serial.println("SIM7600: Failed to set client cert");
    return false;
  }
  
  if (!cmqtt_send("AT+CSSLCFG=\"clientkey\",1,\"private_key.pem\"", 2000, true)) {
    Serial.println("SIM7600: Failed to set private key");
    return false;
  }
  
  Serial.println("SIM7600: TLS certificates configured successfully");
  return true;
}

bool sim7600_tls_clear_certs() {
  Serial.println("SIM7600: Clearing TLS certificates...");
  
  // Clear certificates (ignore errors as they might not exist)
  cmqtt_send("AT+CCERTDELE=\"ca_cert.pem\"", 1000, false);
  cmqtt_send("AT+CCERTDELE=\"client_cert.pem\"", 1000, false);
  cmqtt_send("AT+CCERTDELE=\"private_key.pem\"", 1000, false);
  
  return true;
}
