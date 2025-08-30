#ifndef BUZZER_H
#define BUZZER_H

void buzzer_init(int pin, int ledc_channel, int freq_hz, int resolution_bits);
void buzzer_on();
void buzzer_off();

#endif // BUZZER_H
