// Copy this file to wifi_config.h and fill in your network. wifi_config.h is in .gitignore,
// so your password stays out of the repository. Without wifi_config.h the firmware builds
// with the web interface left out.

#define WIFI_SSID     "your-network"
#define WIFI_PASSWORD "your-password"

// Optional: Home Assistant over MQTT. Leave MQTT_HOST out to turn MQTT off. Use the broker's
// IP address: .local names don't resolve here.
#define MQTT_HOST     "192.168.1.10"
#define MQTT_BROKER_PORT 1883    // optional, 1883 is the default
#define MQTT_USER     "mqtt-user"      // leave these two out if the broker needs no login
#define MQTT_PASSWORD "mqtt-password"
