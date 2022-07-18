#include <Arduino.h>
#include "DHT.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <map>
#include <string>
#include <iostream>
using namespace std;

#define DHTPIN 5
#define DHTTYPE DHT22

#define MQTT_SERVER "10.88.78.186"
#define MQTT_SERVERPORT 1884
#define MSG_BUFFER_SIZE (50)

int BUILTIN_LED_PIN = 2;
int COOLING_RELAY_PIN = 14;
int FAN_RELAY_PIN = 16;

float wantedTemp = 8;
float wantedTempRange = 3;

float extraDegreesForFan = 0.4; // offtemp+extraDegreesForFan*2 should always be smaller than onTemp
boolean ON_STATE = 1;
boolean COOLING_ON_STATE = 0;
float onTemp = 9;
boolean coolingOn = false;

DHT dht(DHTPIN, DHTTYPE);

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient wifiClient;
PubSubClient client(wifiClient);
char msg[MSG_BUFFER_SIZE];

// Replace with your network credentials
const char *ssid = "ko-lab-iot";
const char *password = "PASSWORD";
const string feedPrefix = "feeds/fridge/";
const string feedPrefix_legacy = "/feeds/fridge/";
const string mqqt_debug_topic = feedPrefix + "debug";

const string mqtt_appliance_temp_topic = feedPrefix + "temp/appliance";
const string mqtt_compressor_temp_topic = feedPrefix + "temp/compressor";
const string mqtt_appliance_temp_topic_legacy = feedPrefix_legacy + "appliance_temp";

const string mqtt_compressor_state_topic = feedPrefix + "state/compressor";
const string mqtt_fan_state_topic = feedPrefix + "state/fan";
const string mqtt_light_state_topic = feedPrefix + "state/light";

const string mqtt_toggle_compressor_topic = feedPrefix + "toggle/compressor";
const string mqtt_toggle_fan_topic = feedPrefix + "toggle/fan";
const string mqtt_toggle_light_topic = feedPrefix + "toggle/light";

const string mqtt_set_temp_topic = feedPrefix + "set/temp";
const string mqtt_set_temp_range_topic = feedPrefix + "set/temp_range";

// map<char *, char *> states;
// const int APPLIANCE_POWER_PIN = -1;           // TODO use 2 gpio pins for switching between different thermistor readings
// const int[] ALL_PINS = [APPLIANCE_POWER_PIN]; // TODO use 2 gpio pins for switching between different thermistor readings
float readApplianceTemp()
{
  // TODO enable APPLIANCE_POWER_PIN and disable other pins https://forum.arduino.cc/t/nodemcu-1-analog-pin-multiple-thermistors-reading-problem/593407/7
  return thermistor->readTempC();
}

float readCompressorTemp()
{
  // TODO enable COMPRESSOR_POWER_PIN and disable other pins https://forum.arduino.cc/t/nodemcu-1-analog-pin-multiple-thermistors-reading-problem/593407/7
  return thermistor->readTempC();
}
class FridgeState
{
public:
  float applianceTemp;
  float compressorTemp;
  float dht22Temp;
  boolean compressorState = 0;
  boolean fanState = 0;
  boolean coolingMode = 0;
  boolean killSwitchOn = 0;
  boolean forceFanOn = 0;
  boolean forceOn = 0;

  unsigned long lastCompressorStateChange;
  unsigned long lastLightStateChange;
  unsigned long lastFanStateChange;
  void updateTemps()
  {
    this->applianceTemp = readApplianceTemp();
    this->compressorTemp = readCompressorTemp();
    this->dht22Temp = dht.readTemperature(false);
  }
  void toggleCompressor()
  {
    this->lastCompressorStateChange = millis();
    this->compressorState = !this->compressorState;
    digitalWrite(COOLING_RELAY_PIN, this->compressorState);
  }
  void toggleFan()
  {
    this->lastFanStateChange = millis();
    this->fanState = !this->fanState;
    digitalWrite(FAN_RELAY_PIN, this->fanState);
  }
  void toggleLight()
  {
    this->lastLightStateChange = millis();
    this->lightState = !this->lightState;
    digitalWrite(FAN_RELAY_PIN, this->lightState);
  }
};
FridgeState fridgeState = FridgeState();

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
void setupGPIO();
void mSetupSerial();
void setupWifi();
void setupOTA();
void setupMQTT();
void blink(int delayMillis, int times);
void handleMQTT(float temp);
void handleAutoCooling(float temp);
unsigned long lastPublish = 0;
void mqttDebuglog(char *msg);

void setup(void)
{
  setupGPIO();
  mSetupSerial();
  setupWifi();
  setupMQTT();
  setupOTA();
  dht.begin();
  mqttDebuglog("dht.begin done");
  blink(400, 1);
}

void loop(void)
{
  ArduinoOTA.handle();
  float temp = dht.readTemperature(false);
  handleMQTT(temp);
  if (!fridgeState.on())
  {
    digitalWrite(COOLING_RELAY_PIN, LOW);
    digitalWrite(FAN_RELAY_PIN, LOW);
    return;
  }
  handleAutoCooling(temp);
}

void blink(int delayMillis, int times)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(BUILTIN_LED_PIN, HIGH);
    delay(delayMillis);
    digitalWrite(BUILTIN_LED_PIN, LOW);
    delay(delayMillis);
  }
}
void disableCooling()
{
  digitalWrite(COOLING_RELAY_PIN, LOW);
}

void handleAutoCooling(FridgeState temp)
{
  if (isnan(temp))
  {
    disableCooling();
    blink(400, 2);
  }
  float lowTemp = wantedTemp - (wantedTempRange / 2);
  float highTemp = wantedTemp + (wantedTempRange / 2);

  if (temp < lowTemp)
  {
    digitalWrite(COOLING_RELAY_PIN, LOW);
    coolingOn = false;
  }
  else if (temp > highTemp)
  {
    digitalWrite(COOLING_RELAY_PIN, HIGH);
    coolingOn = true;
    digitalWrite(FAN_RELAY_PIN, HIGH); // To Be extra sure
  }
  if (temp > highTemp - extraDegreesForFan)
  {
    digitalWrite(FAN_RELAY_PIN, HIGH);
    coolingOn = true;
  }
  else if (temp > lowTemp + extraDegreesForFan)
  {
    if (!coolingOn)
    {
      digitalWrite(FAN_RELAY_PIN, LOW);
    }
  }
}
void handleMQTT(FridgeState state)
{
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  String tempMsg = String(temp, 2);
  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  // Adafruit_MQTT_Subscribe *subscription;
  // if ((subscription = mqtt.readSubscription(100)))
  // {
  //   if (subscription == &coolingOnOffButton)
  //   {
  //     COOLING_ON_STATE = !COOLING_ON_STATE;
  //     digitalWrite(COOLING_RELAY_PIN, COOLING_ON_STATE ? HIGH : LOW);
  //     cooling_state.publish(COOLING_ON_STATE ? "HIGH" : "LOW");
  //   }
  //   else if (subscription == &onoffbutton)
  //   {
  //     ON_STATE = !ON_STATE;
  //     digitalWrite(BUILTIN_LED_PIN, ON_STATE ? HIGH : LOW);
  //     fridge_state.publish(ON_STATE ? "HIGH" : "LOW");
  //   }
  // }

  unsigned long nowTime = millis();
  if (nowTime - lastPublish > 2000)
  {
    client.publish(mqtt_appliance_temp_topic, tempMsg.c_str());
    lastPublish = millis();
  }
}

void mSetupSerial()
{
  Serial.begin(115200);
  Serial.println("Booting");
}

void setupGPIO()
{
  // preparing GPIOs
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  pinMode(COOLING_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);

  digitalWrite(BUILTIN_LED_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(COOLING_RELAY_PIN, LOW);
}

void setupWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! No OTA Possible...");
    blink(500, 4);
    digitalWrite(COOLING_RELAY_PIN, LOW);
    return;
  }
}

void setupOTA()
{
  ArduinoOTA.onStart([]()
                     { Serial.println("Start"); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();
  mqttDebuglog("setupOTA done");
}
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  string debugMsg("received: ");
  debugMsg.append(topic);
  debugMsg.append(": ");
  for (int i = 0; i < length; i++)
  {
    debugMsg.append((char)payload[i]);
  }
  mqttDebuglog(debugMsg.c_str());
  switch (topic)
  {
  case mqtt_toggle_compressor_topic:
    fridgeState.toggleCompressor();
    break;
  case mqtt_toggle_fan_topic:
    fridgeState.toggleFan();
    break;
  case mqtt_toggle_light_topic:
    fridgeState.toggleLight();
    break;
  case mqtt_set_temp_range_topic:
    fridgeState.setTemp();
    break;
  case mqtt_set_temp_topic;:
    fridgeState.setTempRange();
    break;
  default:
    mqttDebuglog("Last MQTT Topic not recognized");
  }
}

void setupMQTT()
{
  client.setServer(MQTT_SERVER, MQTT_SERVERPORT);
  client.setCallback(mqttCallback);
  MQTT_connect();
  mqttDebuglog("MQTT connect Done");
  string debugMsg("IP address: ");
  debugMsg.append(WiFi.localIP());
  mqttDebuglog(debugMsg);
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-Fridge";
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      // Once connected, publish an announcement...
      mqttDebuglog("connected");
      // ... and resubscribe
      client.subscribe(mqtt_toggle_compressor_topic);
      client.subscribe(mqtt_toggle_fan_topic);
      client.subscribe(mqtt_toggle_light_topic);
      client.subscribe(mqtt_set_temp_range_topic);
      client.subscribe(mqtt_set_temp_topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect()
{
  reconnect();
  // int8_t ret;

  // // Stop if already connected.
  // if (mqtt.connected())
  // {
  //   return;
  // }

  // Serial.print("Connecting to MQTT... ");

  // uint8_t retries = 3;
  // while ((ret = mqtt.connect()) != 0)
  // { // connect will return 0 for connected
  //   Serial.println(mqtt.connectErrorString(ret));
  //   Serial.println("Retrying MQTT connection in 5 seconds...");
  //   mqtt.disconnect();
  //   delay(500); // wait 5 seconds
  //   retries--;
  //   if (retries == 0)
  //   {
  //     // basically die and wait for WDT to reset me
  //     //      while (1);
  //   }
  // }
  // Serial.println("MQTT Connected!");
}

void mqttDebuglog(char *msg)
{
  client.publish(mqqt_debug_topic, msg);
}