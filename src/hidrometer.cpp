#include "hidrometer.h"
#include <Preferences.h>

static Preferences prefs;
const char* PREF_NAMESPACE = "hidro";

void hidrometer_init() {
  prefs.begin(PREF_NAMESPACE, false);
}
float hidrometer_load_total() {
  return prefs.getFloat("totalL", 0.0f);
}
void hidrometer_save_total(float v) {
  prefs.putFloat("totalL", v);
}
void hidrometer_reset() {
  prefs.putFloat("totalL", 0.0f);
}
