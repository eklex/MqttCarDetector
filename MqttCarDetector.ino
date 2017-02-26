#include "MqttCarDetector.h"

/**
 * Constants
 */
/* Firmware version 
 * Note: Need to be updated to reflect release number */
const char* firmware_version = "car-detector-rel0.2";
/* LED pin output */
const unsigned int led_pin = 5;
const unsigned int built_in_led = 2;
/* Sensor pin input */
const unsigned int sensor_pin = A0;
/* Sensor measurement delay (ms) */
const unsigned long sensor_measure_delay = 50;
/* Sensor thresold for moving car */
const unsigned long sensor_threshold_on = 100;
const unsigned long sensor_threshold_off = 20;
/* Low-Pass filter coeff */
const float sensor_lpf = 0.8;
/* Delay before reconnecting to MQTT broker */
const unsigned long mqtt_reconnect_sec = 60;
/* Delay before checking OTA update */
const unsigned long ota_check_sec = 120;

/**
 * Global variables
 */
/* Reserved space for SPIFFS variables */
char wifi_ssid    [config_len]  = {0};
char wifi_key     [config_len]  = {0};
char wifi_hostname[config_len]  = {0};
char mqtt_broker  [config_len]  = {0};
char mqtt_topic   [topic_len]   = {0};
char mqtt_id      [topic_len*2] = {0};
char spiffs_ver   [spiffs_len]  = {0};

static   int  sensor           = 0;
static   int  last_sensor_read = 0;
static   bool car_moving       = false;
static   bool car_detected     = false;
static   bool led_state        = false;
static   int  led_interval     = 0;
static   int  last_led_check   = 0;
static   long last_connection  = 0;
static   long last_ota_check   = 0;
static   int  fw_ota_status    = 0;
static   int  fs_ota_status    = 0;

void setup()
{
  int index;
  
#ifdef DEBUG
  /**
   * Setup serial for debug
   */
  Serial.begin(115200);
#endif

  /**
   * Display firmware version
   */
  Debug("Firmware version: "); Debugln(firmware_version);
  
  /**
   * Setup pin mode
   */
  pinMode(built_in_led, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(sensor_pin, INPUT);
  digitalWrite(built_in_led, false);
  digitalWrite(led_pin, false);
  
  /**
   * Retrieve WiFi and MQTT configurations
   * in file system
   */
  Debugln("\n---> Retrieve WiFi and MQTT configurations");
  if(retrieveConfig() != 0)
  {
    Debugln("Configurations failure!");
    for(index = 0; index < 3; index++)
    {
      digitalWrite(built_in_led, true);
      delay(500);
      digitalWrite(built_in_led, false);
      delay(500);
    }
    delay(1000);
    ESP.reset();
  }

  /**
   * Connect to WiFi
   */
  Debugln("\n---> Connect to WiFi");
  if(wifiConnect() != 0)
  {
    Debugln("Connection failure!");
    for(index = 0; index < 5; index++)
    {
      digitalWrite(built_in_led, true);
      delay(500);
      digitalWrite(built_in_led, false);
      delay(500);
    }
    delay(1000);
    ESP.reset();
  }

  /**
   * Initialize MQTT client
   */
  Debugln("\n---> Initialize MQTT client");
  mqttInit();

  /**
   * Process MQTT topics
   */
  Debugln("\n---> Process MQTT topics");
  mqttProcess(car_detected, sensor, firmware_version);

  digitalWrite(built_in_led, true);
  Debugln("\n---> ESP setup routine completed.");
}

void loop()
{
  /**
   * Process MQTT messages
   */
  mqttLoop();
  
  /**
   * Process sensor value
   */
  if(millis() - last_sensor_read >= sensor_measure_delay)
  {
    last_sensor_read = millis();
    sensor     = lpf(analogRead(sensor_pin), sensor_lpf, (float)sensor);
    car_moving = movingCar(sensor, sensor_threshold_on, sensor_threshold_off);
  }
  
  /**
   * Process led state
   * when car is moving
   */
  if(car_moving == true)
  {
    if(sensor < 80)
    {
      if(led_state == true)
      {
        led_state = false;
        digitalWrite(led_pin, led_state);
      }
      led_interval = (int)(0xFFFFFFFF);
    }
    else if(sensor >= 1000)
    {
      led_interval = 0;
    }
    else
    {
      led_interval = 1000 - sensor;
    }
  
    if(led_state == false &&
       millis() - last_led_check >= led_interval)
    {
      last_led_check = millis();
      led_state = true;
      digitalWrite(led_pin, led_state);
    }
    else if(led_state == true &&
            millis() - last_led_check >= led_interval/2)
    {
      last_led_check = millis();
      led_state = false;
      digitalWrite(led_pin, led_state);
    }
  }
  /**
   * Process eveything else
   * when car is not moving
   */
  else
  {
    /**
     * Switch off led
     */
    if(led_state == true)
    {
      led_state = false;
      digitalWrite(led_pin, led_state);
    }
    /**
     * Publish car detected status
     */
    if(car_detected == false &&
       sensor > 100)
    {
      car_detected = true;
      Debugln("Car detected");
      mqttProcess(car_detected, sensor, NULL);
    }
    else if(car_detected == true &&
            sensor < 80)
    {
      car_detected = false;
      Debugln("No car detected");
      mqttProcess(car_detected, sensor, NULL);
    }
    /**
     * Reconnect to MQTT broker and process data
     */
    else if(millis() - last_connection > mqtt_reconnect_sec*1000)
    {
      last_connection = millis();
      mqttProcess(car_detected, sensor, NULL);
    }
    /**
     * Update Over-The-Air for file system and firmware
     */
    if(millis() - last_ota_check > ota_check_sec*1000)
    {
      Debugln("\n---> Check firmware update");
      last_ota_check = millis();
      fs_ota_status = otaUpdate(mqtt_broker, spiffs_ver, FILESYSTEM);
      if(fs_ota_status == 1)
      {
        Debugln("\n---> File System OTA update done. Going to reset...");
        ESP.restart();
      }
      fw_ota_status = otaUpdate(mqtt_broker, firmware_version, FIRMWARE);
      if(fw_ota_status == 1)
      {
        Debugln("\n---> Firmware OTA update done. Going to reset...");
        ESP.restart();
      }
    }
  }
}

bool movingCar(int data, int on_th, int off_th)
{
  static int16_t      previous_data[200] = {0};
  static int          index              = 0;
  static bool         moving             = false;
  static unsigned int forced_delay       = 0;
  
  if(moving == false &&
     abs((int16_t)data - previous_data[index]) > on_th)
  {
    moving = true;
    forced_delay = index + sizeof(previous_data)/sizeof(previous_data[0]);
    forced_delay %= sizeof(previous_data)/sizeof(previous_data[0]);
    Debugln("Car is moving");
  }
  else if(moving == true &&
          forced_delay == 0 &&
          abs((int16_t)data - previous_data[index]) < off_th)
  {
    moving = false;
    Debugln("No moving car");
  }
  else if(forced_delay != 0)
  {
    forced_delay--;
  }
  
  previous_data[index] = (int16_t)data;
  index = (index + 1) % (sizeof(previous_data)/sizeof(previous_data[0]));
  
  return(moving);
}

int lpf(int data, float lp_val, float previous_val)
{
  if(lp_val > 1)
  {
    lp_val = 0.99;
  }
  else if(lp_val <= 0)
  {
    lp_val = 0;
  }
  
  previous_val = (data * (1 - lp_val)) + (previous_val  *  lp_val);
  
  return((int)previous_val);
}

