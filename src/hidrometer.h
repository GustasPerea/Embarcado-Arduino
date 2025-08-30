#ifndef HIDROMETER_H
#define HIDROMETER_H

void hidrometer_init();
float hidrometer_load_total();
void hidrometer_save_total(float v);
void hidrometer_reset();

#endif // HIDROMETER_H
