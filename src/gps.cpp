#include "gps.h"
#include <Arduino.h>
#include "sim7600.h"

static uint16_t noFixCount = 0;

void gps_init() {
  // Liga o GPS do SIM7600 (logs reduzidos por padrão)
  sim7600_set_verbose(false);
  if (!sim7600_gps_on()) {
    Serial.println("GPS: falha ao ligar");
  }
}

void gps_get_position(char* buffer, size_t len) {
  float lat = 0.0f, lon = 0.0f;
  if (sim7600_gps_get_fix(&lat, &lon)) {
    snprintf(buffer, len, "%.6f,%.6f", lat, lon);
    noFixCount = 0;
  } else {
    // Silencia log repetitivo; apenas atua a cada 10 tentativas
    if ((++noFixCount % 10) == 0) {
      char st[64] = {0};
      sim7600_gps_status(st, sizeof(st));
      Serial.print("GPS sem fix: ");
      Serial.println(st);
    }
    snprintf(buffer, len, "0.000000,0.000000");
    if (noFixCount >= 120) {
      Serial.println("GPS: 2 min sem fix, cold reset...");
      sim7600_gps_cold_reset();
      noFixCount = 0;
    }
  }
}
