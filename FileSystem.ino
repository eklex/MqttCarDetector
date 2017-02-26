#include <FS.h>
#include "CarDetector.h"

static int readFile(const char *filename, char *buff, unsigned int len)
{
  String data;
  
  /* Sanity check */
  if(SPIFFS.exists(filename))
  {
    /* Open file */
    File f = SPIFFS.open(filename, "r");
    if(f != NULL)
    {
      /* Load file contents in temp string */
      data = f.readStringUntil('\n');
      /* Remove CR, LF, space and tab */
      data.trim();
      /* Close file */
      f.close();
      
      /* Verify that buffer is large enough */
      if(data.length() >= len)
      {
        Debugln("Data too large for buffer!");
        return(-3);
      }
      /* Zeroed buffer */
      memset(buff, 0, len);
      /* Copy data to buffer */
      strcpy(buff, data.c_str());
      return(0);
    }
    /* Cannot open the file */
    Debug("Cannot open file "); Debugln(filename);
    return(-2);
  }
  /* File does not exist */
  Debug(filename); Debugln(" does not exist");
  return(-1);
}

unsigned int retrieveConfig(void)
{
  unsigned int return_code = 0;
  
  SPIFFS.begin();
  if(readFile("/wifi_ssid", (char*)wifi_ssid, sizeof(wifi_ssid)/sizeof(wifi_ssid[0])))
  {
    Debugln("Error reading WIFI SSID file");
    return_code |= (1 << 0);
  }
  if(readFile("/wifi_key", (char*)wifi_key, sizeof(wifi_key)/sizeof(wifi_key[0])))
  {
    Debugln("Error reading WIFI password file");
    return_code |= (1 << 1);
  }
  if(readFile("/mqtt_broker", (char*)mqtt_broker, sizeof(mqtt_broker)/sizeof(mqtt_broker[0])))
  {
    Debugln("Error reading MQTT broker file");
    return_code |= (1 << 2);
  }
  if(readFile("/mqtt_topic", (char*)mqtt_topic, sizeof(mqtt_topic)/sizeof(mqtt_topic[0])))
  {
    Debugln("Error reading MQTT main topic file");
    return_code |= (1 << 3);
  }
  if(readFile("/spiffs_version", (char*)spiffs_ver, sizeof(spiffs_ver)/sizeof(spiffs_ver[0])))
  {
    Debugln("Error reading SPIFFS version file");
    return_code |= (1 << 4);
  }
  if(readFile("/wifi_hostname", (char*)wifi_hostname, sizeof(wifi_hostname)/sizeof(wifi_hostname[0])))
  {
    Debugln("Error reading WIFI Hostname file");
    return_code |= (1 << 5);
  }
  
  return(return_code);
}

