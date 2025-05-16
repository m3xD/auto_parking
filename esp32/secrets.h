// secrets.h
#include <pgmspace.h>

#ifndef SECRETS_H
#define SECRETS_H

// --- CẤU HÌNH WIFI ---
#define SECRET_WIFI_SSID "YOUR_WIFI_SSID"
#define SECRET_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// --- CẤU HÌNH AWS IoT ---
#define SECRET_AWS_IOT_ENDPOINT "a2xdvypya6llyq-ats.iot.ap-southeast-1.amazonaws.com" 
#define SECRET_AWS_THING_NAME "ESP32_Auto_Parking"

// --- CHỨNG CHỈ AWS IoT ---

// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)EOF";

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
-----END CERTIFICATE-----
)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
-----END RSA PRIVATE KEY-----
)KEY";
#endif // SECRETS_H