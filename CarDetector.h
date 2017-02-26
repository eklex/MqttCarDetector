#ifndef __CAR_DETECTOR__
#define __CAR_DETECTOR__

#include <ESP8266WiFi.h>

#undef DEBUG

#ifdef DEBUG
  #define Debug(_x_)    Serial.print(_x_)
  #define Debugln(_x_)  Serial.println(_x_)
#else
  #define Debug(_x_)
  #define Debugln(_x_)
#endif

const unsigned int config_len  = 64;
const unsigned int topic_len   = 64;
const unsigned int spiffs_len  = 64;

typedef enum _updateType {
  FIRMWARE = false,
  FILESYSTEM = true
} updateType;

extern char wifi_ssid    [config_len];
extern char wifi_key     [config_len];
extern char wifi_hostname[config_len];
extern char mqtt_broker  [config_len];
extern char mqtt_topic   [topic_len];
extern char mqtt_id      [topic_len*2];
extern char spiffs_ver   [spiffs_len];

unsigned int retrieveConfig(void);

int wifiConnect(void);
int otaUpdate(const char*, const char*, bool);

void mqttConfig(void);
int mqttConnect(void);
int mqttInit(void);
int mqttProcess(bool, unsigned int, const char*);
void mqttLoop(void);

int lpf(int data, float lp_val, float previous_val);
bool movingCar(int data, int on_th, int off_th);

#endif /* __CAR_DETECTOR__ */

