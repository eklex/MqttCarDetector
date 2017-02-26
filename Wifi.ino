#include <ESP8266httpUpdate.h>
#include "MqttCarDetector.h"

const unsigned int wifi_timeout_sec = 20;

int wifiConnect()
{
  unsigned int timeout = 0;
  
  Debug("Connecting to SSID ");
  Debug(wifi_ssid); Debug(": ");

  WiFi.hostname(wifi_hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_key);
  while(WiFi.status() != WL_CONNECTED &&
        timeout++ < wifi_timeout_sec*2)
  {
    Debug("#");
    delay(500);
  }
  Debug("\n");
  
  if(timeout >= wifi_timeout_sec*2)
  {
    Debug("WiFi connection failed with error ");
    Debugln(WiFi.status());
    return(-1);
  }
  
  WiFi.setAutoReconnect(true)  
  
  Debug("WiFi connection established to "); Debugln(wifi_ssid);
  Debug("IP address is "); Debugln(WiFi.localIP());
  
  return(0);
}

int otaUpdate(const char* server, const char* version, bool file_sys)
{
  char server_url[64] = {0};
  
  Debug("Server: "); Debugln(server);
  if(file_sys == true) Debug("File System: ");
  else                 Debug("Firmware: ");
  Debugln(version);

  /* Build server URL */
  sprintf(server_url, "http://%s/mqtt/mqtt-esp.php", server);
  
  /* Prevent reboot after update */
  ESPhttpUpdate.rebootOnUpdate(false);
  
  /* Run OTA update */
  t_httpUpdate_return ret = (file_sys == true) ?
                            ESPhttpUpdate.updateSpiffs(server_url, version):
                            ESPhttpUpdate.update(server_url, version);
  switch(ret)
  {
    case HTTP_UPDATE_FAILED:
      Debugln("Update failed.");
      Debugln(ESPhttpUpdate.getLastErrorString());
      return(-1);
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Debugln("No update available.");
      break;
    case HTTP_UPDATE_OK:
      Debugln("Update completed successfully.");
      return(1);
      break;
    default:
      Debugln("Unknown status.");
      Debugln(ESPhttpUpdate.getLastErrorString());
      break;
  }
  
  return(0);
}

