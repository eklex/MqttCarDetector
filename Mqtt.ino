#include <PubSubClient.h>
#include "MqttCarDetector.h"

const unsigned int mqtt_connection_retry = 10;

enum topic_e {
  Root = 0,
  Status = 1,
  Sensor = 2,
  Firmware = 3,
};
const char *device_topics[] = {
  "", "status", "sensor", "firmware",
};
const unsigned int device_topic_cnt =
  sizeof(device_topics)/sizeof(device_topics[0]);

static char mqtt_device_topics [device_topic_cnt][topic_len*2] = {0};

static WiFiClient wifi_client;
static PubSubClient mqtt_client(wifi_client);

void mqttConfig(void)
{
  uint8_t mac[6] = {0};
  int     index;

  /**
   * Setup MQTT client
   */
  /* Set broker IP and port */
  mqtt_client.setServer(mqtt_broker, 1883);

  /**
   * Build MQTT client ID from MAC address
   */
  /* Retrieve MAC address */
  WiFi.macAddress(mac);
  /* Create ID from MAC address */
  for(index = 0; index < sizeof(mac)/sizeof(mac[0]); index++)
  {
    sprintf(mqtt_id, "%s%02x", mqtt_id, mac[index]);
  }
  Debug("MQTT client ID is  "); Debugln(mqtt_id);
  
  /**
   * Build absolute topics
   */
  for(index = 0; index < device_topic_cnt; index++)
  {
    strcpy(mqtt_device_topics[index], mqtt_topic);
    if(mqtt_topic[strlen(mqtt_topic)-1] != '/')
    {
      strcat(mqtt_device_topics[index], "/");
    }
    if(index == 0)
    {
      /* Remove '/' for root topic */
      mqtt_device_topics[index][strlen(mqtt_device_topics[index])-1] = 0;
    }
    strcat(mqtt_device_topics[index], device_topics[index]);
    Debug("Create MQTT topic "); Debugln(mqtt_device_topics[index]);
  }
}

int mqttConnect(void)
{
  unsigned int connection_retry = 0;
  
  /**
   * Connect to MQTT broker
   */
  while(!mqtt_client.connected() &&
        connection_retry++ < mqtt_connection_retry)
  {
    Debug("MQTT client connection ... ");
    if(mqtt_client.connect(mqtt_id))
    {
      Debugln("connected.");
    }
    else
    {
      Debug("failed with status ");
      switch(mqtt_client.state())
      {
        case MQTT_CONNECTION_TIMEOUT:
          Debugln("connection timeout."); break;
        case MQTT_CONNECTION_LOST:
          Debugln("connection lost."); break;
        case MQTT_CONNECT_FAILED:
          Debugln("connection failed."); break;
        case MQTT_DISCONNECTED:
          Debugln("disconnected."); break;
        case MQTT_CONNECT_BAD_PROTOCOL:
          Debugln("bad protocol."); break;
        case MQTT_CONNECT_BAD_CLIENT_ID:
          Debugln("bad clinet ID."); break;
        case MQTT_CONNECT_UNAVAILABLE:
          Debugln("unavailable."); break;
        case MQTT_CONNECT_BAD_CREDENTIALS:
          Debugln("bad credentials."); break;
        case MQTT_CONNECT_UNAUTHORIZED:
          Debugln("connection unauthorized."); break;
        default:
          Debugln("unknown state."); break;
      }
      delay(5000);
    }
  }
  /* Verify that connecton was successful */
  if(connection_retry >= mqtt_connection_retry)
  {
    Debugln("MQTT connection process aborted!");
    return(-1);
  }
  return(0);
}

int mqttInit(void)
{
  const unsigned int wait_topic_max = 500;
  unsigned int wait_topic_cnt = 0;
  /* Configure MQTT client */
  mqttConfig();
  
  /* Connect to MQTT broker */
  if(mqttConnect() != 0)
  {
    return(-1);
  }
  return(0);
}

int mqttProcess(bool stat, int sens)
{
  char         stat_str[8]      = {0};
  char         sens_str[8]      = {0};
  
  /**
   * Ensure connection to MQTT broker is establish
   */
  if(!mqtt_client.connected())
  {
    if(mqttConnect() != 0)
    {
      return(-1);
    }
  }

  /**
   * Publish and Subscribe
   */
  /* Convert numerical values in string */
  sprintf(stat_str, "%s", stat ? "on" : "off");
  sprintf(sens_str, "%d", sens);
  
  /* Publish data to broker */
  mqtt_client.publish(mqtt_device_topics[Root],    "online", false);
  mqtt_client.publish(mqtt_device_topics[Status],  stat_str, true);
  mqtt_client.publish(mqtt_device_topics[Sensor],  sens_str, true);
  Debugln("Data published successfully.");

  /* Let's the MQTT client processes messages */
  mqtt_client.loop();
  return(0);
}

int mqttProcess(const char *firmware)
{
  /**
   * Ensure connection to MQTT broker is establish
   */
  if(!mqtt_client.connected())
  {
    if(mqttConnect() != 0)
    {
      return(-1);
    }
  }

  /**
   * Publish and Subscribe
   */
  if(firmware != NULL)
  {
    mqtt_client.publish(mqtt_device_topics[Firmware], firmware, true);
    Debugln("Data published successfully.");
    
    /* Let's the MQTT client processes messages */
    mqtt_client.loop();
    return(0);
  }
  return(-1);
}

void mqttLoop(void)
{
  mqtt_client.loop();
}

