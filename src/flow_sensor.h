#ifndef FLOW_SENSOR_H
#define FLOW_SENSOR_H
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"

void flow_sensor_init(int pin, float calibration_pulses_per_liter, portMUX_TYPE* mux);
// Retorna litros medidos no intervalo (segundos) e zera o contador
float flow_sensor_get_increment(float interval_sec);

#endif // FLOW_SENSOR_H
