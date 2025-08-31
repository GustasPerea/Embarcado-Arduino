#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stddef.h>

void wifi_manager_init();
bool wifi_manager_connected();
// Start SoftAP if not connected; webform will allow saving credentials
void wifi_manager_start_provisioning();
void wifi_manager_loop();
void wifi_manager_clear_credentials();

#endif // WIFI_MANAGER_H
