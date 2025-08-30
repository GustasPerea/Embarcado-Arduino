#include "buzzer.h"
#include <Arduino.h>

static int buzzer_ledc_channel = 0;
static int buzzer_resolution = 8;

void buzzer_init(int pin, int ledc_channel, int freq_hz, int resolution_bits) {
  buzzer_ledc_channel = ledc_channel;
  buzzer_resolution = resolution_bits;
  ledcSetup(buzzer_ledc_channel, freq_hz, buzzer_resolution);
  ledcAttachPin(pin, buzzer_ledc_channel);
  ledcWrite(buzzer_ledc_channel, 0);
}

void buzzer_on() {
  int duty = 1 << (buzzer_resolution - 1);
  ledcWrite(buzzer_ledc_channel, duty);
}

void buzzer_off() {
  ledcWrite(buzzer_ledc_channel, 0);
}
