#include "flow_sensor.h"
#include <Arduino.h>
#include "driver/gpio.h"

static volatile uint32_t pulseCount = 0;
static int flow_pin = -1;
static float pulses_per_liter = 450.0f; // default, can be set in init
static portMUX_TYPE* flow_mux = NULL;

void IRAM_ATTR flow_isr() {
  if (flow_mux) portENTER_CRITICAL_ISR(flow_mux);
  pulseCount++;
  if (flow_mux) portEXIT_CRITICAL_ISR(flow_mux);
}

void flow_sensor_init(int pin, float calibration_pulses_per_liter, portMUX_TYPE* mux) {
  flow_pin = pin;
  pulses_per_liter = calibration_pulses_per_liter;
  flow_mux = mux;

  // YF-S201 é coletor aberto -> usa pull-up interno. No PW_Embarcado usamos borda de subida (RISING).
  pinMode(flow_pin, INPUT_PULLUP);
  pulseCount = 0;
  // attach interrupt
  attachInterrupt(digitalPinToInterrupt(flow_pin), flow_isr, RISING);
}

// Corrigido: usa o fator K (7.5) e o intervalo em segundos, igual ao PW_Embarcado
float flow_sensor_get_increment(float interval_sec) {
  uint32_t pulses = 0;
  if (flow_mux) {
    portENTER_CRITICAL(flow_mux);
    pulses = pulseCount;
    pulseCount = 0;
    portEXIT_CRITICAL(flow_mux);
  } else {
    noInterrupts();
    pulses = pulseCount;
    pulseCount = 0;
    interrupts();
  }
  // YF-S201: litros = pulsos / (K * 60)
  const float K = 7.5f;
  if (interval_sec <= 0.0f) return 0.0f;
  float liters = (float)pulses / (K * 60.0f);
  return liters;
}
