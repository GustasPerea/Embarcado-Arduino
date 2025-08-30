#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stddef.h>

void wifi_manager_init();
bool wifi_manager_connected();
// Start SoftAP if not connected; webform will allow saving credentials
void wifi_manager_start_provisioning();
// Provide saved broker and credentials
const char* wifi_manager_get_broker();
const char* wifi_manager_get_mqtt_user();
const char* wifi_manager_get_mqtt_pass();
void wifi_manager_loop();
// Clear stored WiFi/MQTT credentials and reboot/start provisioning
void wifi_manager_clear_credentials();

#endif // WIFI_MANAGER_H
