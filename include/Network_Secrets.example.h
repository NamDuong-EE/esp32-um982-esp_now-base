#ifndef NETWORK_SECRETS_H
#define NETWORK_SECRETS_H

// Copy this file to Network_Secrets.h. The real file is ignored by Git.
#define BASE_WIFI_SSID "YOUR_WIFI_SSID"
#define BASE_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define BASE_MQTT_HOST "broker.example.com"
#define BASE_MQTT_PORT 1883
#define BASE_MQTT_USER ""
#define BASE_MQTT_PASSWORD ""

// Used only by the esp32u_base_4g_mqtt environment.
#define BASE_MODEM_APN "YOUR_SIM_APN"
#define BASE_MODEM_GPRS_USER ""
#define BASE_MODEM_GPRS_PASSWORD ""

// ESP-NOW PMK/LMK are provisioned separately with tools/Provision-EspNowSecurity.ps1.
// The generated include/EspNow_Secrets.h is shared with Rover and ignored by Git.

#endif // NETWORK_SECRETS_H
