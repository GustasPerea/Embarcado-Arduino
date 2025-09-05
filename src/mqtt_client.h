#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

// For AWS IoT, pass broker as endpoint (no port needed if you use default 8883) and provide clientId.
void mqtt_client_init(const char* broker, const char* clientId=nullptr, const char* user=nullptr, const char* pass=nullptr);
bool mqtt_client_connected();
void mqtt_client_publish(const char* topic, const char* payload);
void mqtt_client_publish(const char* topic, const char* payload, bool retained);
void mqtt_client_loop();

// Convenience: publish telemetry JSON
// Example JSON:
// {
//   "device_id": "0001-0000",
//   "gps": { "lat": -22.0154, "lon": -47.8913 },
//   "flow_rate": 5600,
//   "total_liters": 1234.5,
//   "status": "On"
// }
// Call with the MQTT topic you want to publish to (e.g., "devices/0001-0000/telemetry").
void mqtt_client_publish_telemetry(
	const char* topic,
	const char* device_id,
	float lat,
	float lon,
	float flow_rate,
	float total_liters,
	const char* status);

#endif // MQTT_CLIENT_H
