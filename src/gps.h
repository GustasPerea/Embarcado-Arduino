#ifndef GPS_H
#define GPS_H

#include <stddef.h>

void gps_init();
void gps_get_position(char* buffer, size_t len);

#endif // GPS_H
