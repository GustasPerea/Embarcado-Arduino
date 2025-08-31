#ifndef SIM7600_H
#define SIM7600_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Configure UART and control pins. All params optional; pass -1 to skip.
// rx_pin: ESP32 RX (connect to module T)
// tx_pin: ESP32 TX (connect to module R)
// pwrkey_pin: ESP32 GPIO to pull module K low to power on
// status_pin: ESP32 GPIO to read module S (high when on)
void sim7600_config_pins(int rx_pin, int tx_pin, int pwrkey_pin = -1, int status_pin = -1);

// Control verbosity (reduced logs by default)
void sim7600_set_verbose(bool enabled);

// Initialize UART and try to power the module on if PWRKEY is available.
void sim7600_init();

// Try to toggle PWRKEY (K) low for ~1.5s to power on the modem.
void sim7600_power_on();

// Read STATUS (S) pin if configured; otherwise returns true after init.
bool sim7600_is_on();

// placeholder: send payload to broker via SIM7600
void sim7600_send(const char* topic, const char* payload);

// MQTT over SIM7600 (TCP socket): open, CONNECT, PUBLISH QoS0, DISCONNECT, close
bool sim7600_mqtt_publish(
	const char* broker,
	unsigned short port,
	const char* user,
	const char* pass,
	const char* clientId,
	const char* topic,
	const char* payload,
	bool retained = false);

// Sync ESP32 time via mobile network using SIM7600 +CNTP
bool sim7600_sync_time_via_ntp(const char* ntp1 = "pool.ntp.org", const char* ntp2 = "time.google.com");

// --- GPS helpers (SIM7600) ---
// Liga o GPS (modo autônomo) usando AT+CGPS=1,1
bool sim7600_gps_on();
// Desliga o GPS
bool sim7600_gps_off();
// Lê posição com AT+CGPSINFO e retorna lat/lon em graus decimais. Retorna false se sem fix.
bool sim7600_gps_get_fix(float* lat, float* lon);

// Lê o status do GPS (AT+CGPSSTATUS?) e opcionalmente retorna a linha de status.
// Retorna true se indicar 2D/3D Fix.
bool sim7600_gps_status(char* buffer, size_t len);

// Força cold reset do GPS (AT+CGPSRST=1)
void sim7600_gps_cold_reset();

#endif // SIM7600_H
