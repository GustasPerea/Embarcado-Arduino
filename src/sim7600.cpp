#include "sim7600.h"
#include <Arduino.h>

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
  String r = sim7600_at("AT+CGPS=1", 1200, false);
  if (r.indexOf("OK") < 0) {
    // Tenta modo (1,1) se necessário
    r = sim7600_at("AT+CGPS=1,1", 1500, false);
  }
  delay(200);
  sim7600_at("AT+CGPS?", 800, false);
  return true; // não força OK; segue a mesma estratégia do teste simples
}

bool sim7600_gps_off() {
  String r = sim7600_at("AT+CGPS=0", 1200, true);
  return r.indexOf("OK") >= 0;
}

bool sim7600_gps_get_fix(float* lat, float* lon) {
  // Estratégia do seu teste: enviar e ler cru por ~2s
  String r = sim7600_at("AT+CGPSINFO", 2000, false);
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
  String r = sim7600_at("AT+CGPSSTATUS?", 1500, false);
  int p = r.indexOf("+CGPSSTATUS:");
  bool fixed = false;
  if (p >= 0) {
    String s = r.substring(p + 12);
    s.trim();
    if (buffer && len) s.toCharArray(buffer, len);
    if (s.indexOf("2D Fix") >= 0 || s.indexOf("3D Fix") >= 0) fixed = true;
  } else if (buffer && len) {
    r.toCharArray(buffer, len);
  }
  return fixed;
}

void sim7600_gps_cold_reset() {
  sim7600_at("AT+CGPSRST=1", 1200, false);
}
