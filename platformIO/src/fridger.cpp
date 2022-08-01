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
#include <thermistor.h>
#include <AsyncTimer.h>

using namespace std;

/**
 * Create a file ".env" in the project root and put the credentials in.
 * Do not commit the .env file to git.
 *
 * $ cat .env
 * KOLAB_SSID=ko-lab-iot
 * KOLAB_PASSWORD=password
 */
#ifndef KOLAB_SSID
#define KOLAB_SSID "ko-lab-iot"
#endif

#ifndef KOLAB_PASSWORD
#define KOLAB_PASSWORD "changeme"
#endif

#define DHTPIN 5
#define DHTTYPE DHT22

#define MQTT_SERVER "10.88.78.186"
#define MQTT_SERVERPORT 1884
#define MSG_BUFFER_SIZE (50)

int BUILTIN_LED_PIN = 2;
int COOLING_RELAY_PIN = 14;
int FAN_RELAY_PIN = 16;
int LIGHT_RELAY_PIN = 12;

DHT dht(DHTPIN, DHTTYPE);

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient wifiClient;
PubSubClient client(wifiClient);
Thermistor *thermistor;
AsyncTimer timer;

#define feedPrefix "feeds/fridge/"
#define concat(first, second) first second

#define feedPrefix_legacy "/feeds/fridge/"
constexpr char *mqqt_debug_topic = concat(feedPrefix, "debug");

constexpr char *mqtt_appliance_temp_topic = concat(feedPrefix, "temp/appliance");
constexpr char *mqtt_compressor_temp_topic = concat(feedPrefix, "temp/compressor");
constexpr char *mqtt_appliance_temp_topic_legacy = concat(feedPrefix_legacy, "appliance_temp");

constexpr char *mqtt_compressor_state_topic = concat(feedPrefix, "state/compressor");
constexpr char *mqtt_fan_state_topic = concat(feedPrefix, "state/fan");
constexpr char *mqtt_light_state_topic = concat(feedPrefix, "state/light");

constexpr char *mqtt_toggle_compressor_topic = concat(feedPrefix, "toggle/compressor");
constexpr char *mqtt_toggle_fan_topic = concat(feedPrefix, "toggle/fan");
constexpr char *mqtt_toggle_light_topic = concat(feedPrefix, "toggle/light");

constexpr char *mqtt_disable_killswitch_topic = concat(feedPrefix, "disable/killswitch");
constexpr char *mqtt_enable_killswitch_topic = concat(feedPrefix, "enable/killswitch");
constexpr char *mqtt_set_temp_topic = concat(feedPrefix, "set/temp");
constexpr char *mqtt_set_temp_range_topic = concat(feedPrefix, "set/temp_range");

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
unsigned int longNbDigits(long number)
{
  if (number < 0)
    return 1 + longNbDigits(abs(number));
  if (number < 10)
    return 1;
  return 1 + longNbDigits(number / 10);
}
boolean publish(const char *topic, float payload, boolean retained, unsigned int minlength, unsigned int precision)
{
  if (minlength < 4 && payload < 0)
    minlength = 4; // 4 = sign + 1 integral digit + fractional point + 1 franctional digit
  char mqtt_messagebuff[longNbDigits((long)payload) + 1 + precision + 1];

  dtostrf(payload, minlength, precision, mqtt_messagebuff);
  return client.publish(topic, (uint8_t *)mqtt_messagebuff, strlen(mqtt_messagebuff), retained);
}

boolean publish(const char *topic, long payload, boolean retained)
{
  char mqtt_messagebuff[longNbDigits(payload) + 1]; // number of digits including sign + null terminaison

  itoa(payload, mqtt_messagebuff, 10);
  return client.publish(topic, (uint8_t *)mqtt_messagebuff, strlen(mqtt_messagebuff), retained);
}
boolean publish(const char *topic, int payload, boolean retained)
{
  return publish(topic, (long)payload, retained);
}
void mqttPublishFloat(string topic, float value)
{
  publish(topic.c_str(), value, false, 3, 2);
}

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void mqttReconnect();
void setupGPIO();
void mSetupSerial();
void setupWifi();
void reconnectWifi();
void twoSecondsLoop();
void setupOTA();
void setupMQTT();
void blink(int delayMillis, int times);
void handleSafety();

void mqttDebugLog(const char *msg, const char *value);
void mqttDebugLog(string msg);
void setupThermistor();
class FridgeState
{
public:
  float applianceTemp;
  float compressorTemp;
  float dht22Temp;
  float wantedTemp = 7.5;
  float wantedTempRange = 3;
  float extraDegreesForFan = 0.4;
  boolean compressorState = 0;
  boolean fanState = 0;
  boolean lightState = 0;
  boolean coolingMode = 0;
  boolean killswitchState = 0;
  boolean forceFanOn = 0;
  boolean forceOn = 0;

  unsigned long lastCompressorStateChange = 0;
  unsigned long lastKillswitchStateChange = 0;
  unsigned long lastLightStateChange = 0;
  unsigned long lastFanStateChange = 0;
  boolean on()
  {
    return !this->killswitchState;
  }
  void updateTemps()
  {
    this->applianceTemp = readApplianceTemp();
    Serial.print("this->applianceTemp: ");
    Serial.println(this->applianceTemp);
    this->compressorTemp = readCompressorTemp();
    Serial.print("this->compressorTemp: ");
    Serial.println(this->compressorTemp);
    this->dht22Temp = dht.readTemperature(false);
    Serial.print("this->dht22Temp: ");
    Serial.println(this->dht22Temp);
  }
  void publishState()
  {
    mqttPublishFloat(mqtt_appliance_temp_topic, this->dht22Temp);
    mqttPublishFloat(mqtt_appliance_temp_topic_legacy, this->dht22Temp);
    mqttPublishFloat(mqtt_compressor_temp_topic, this->compressorTemp);
    // TODO publish other data
  }
  void setKillswitch(boolean newState)
  {
    if (newState)
    {
      this->setCompressor(LOW);
      this->setFan(LOW);
      this->setLight(LOW);
    }
    if (this->killswitchState != newState)
    {
      this->lastKillswitchStateChange = millis();
      this->killswitchState = newState;
      mqttDebugLog("Killswitch to: ", this->killswitchState ? "ON" : "OFF");
    }
  }
  void toggleKillswitch()
  {
    this->setKillswitch(!this->killswitchState);
  }
  void setCompressor(boolean newState)
  {
    if (this->compressorState != newState)
    {
      this->lastCompressorStateChange = millis();
      this->compressorState = newState;
      digitalWrite(COOLING_RELAY_PIN, this->compressorState);
      mqttDebugLog("compressor relay changed to: ", this->compressorState ? "ON" : "OFF");
    }
  }
  void toggleCompressor()
  {
    this->setCompressor(!this->compressorState);
  }
  void setFan(boolean newState)
  {
    if (this->fanState != newState)
    {
      this->lastFanStateChange = millis();
      this->fanState = newState;
      digitalWrite(FAN_RELAY_PIN, this->fanState);
      mqttDebugLog("fan relay changed to: ", this->fanState ? "ON" : "OFF");
    }
  }
  void toggleFan()
  {
    this->setFan(!this->fanState);
  }
  void setLight(boolean newState)
  {
    if (this->lightState != newState)
    {
      this->lastLightStateChange = millis();
      this->lightState = newState;
      digitalWrite(LIGHT_RELAY_PIN, this->lightState);
      mqttDebugLog("light relay changed to: ", this->lightState ? "ON" : "OFF");
    }
  }
  void toggleLight()
  {
    this->setLight(!this->lightState);
  }
  void setTemp(float wantedTemp)
  {
    this->wantedTemp = wantedTemp;
    mqttDebugLog("changed wanted temp to: ", to_string(wantedTemp).c_str());
  }
  void setTempRange(float wantedTempRange)
  {
    this->wantedTempRange = wantedTempRange;
    mqttDebugLog("changed wanted temp range to: ", to_string(wantedTempRange).c_str());
  }
};

void handleMQTT(FridgeState *state);
void handleAutoCooling(FridgeState *state);
FridgeState fridgeState = FridgeState();
void setupTimer()
{
  timer.setInterval([]()
                    {
  if (!fridgeState.on() || fridgeState.coolingMode)
  {
    return;
  }
  fridgeState.setFan(HIGH);

  timer.setTimeout([=]()
                   {
        if (!fridgeState.coolingMode) {
          fridgeState.setFan(LOW);
        } },
                   20 * 1000); },
                    5 * 60 * 1000);
  timer.setInterval([]()
                    { twoSecondsLoop(); },
                    2 * 1000);
}
void setup(void)
{
  setupGPIO();
  mSetupSerial();
  setupWifi();
  setupMQTT();
  setupOTA();
  setupThermistor();
  dht.begin();
  mqttDebugLog("dht.begin done");
  setupTimer();
  blink(400, 1);
}

void loop(void)
{
  reconnectWifi();
  ArduinoOTA.handle();
  fridgeState.updateTemps();
  handleMQTT(&fridgeState);
  handleSafety();
  timer.handle();
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

void handleMQTT(FridgeState *state)
{
  if (!client.connected())
  {
    mqttReconnect();
  }
  client.loop();
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
  pinMode(LIGHT_RELAY_PIN, OUTPUT);

  digitalWrite(BUILTIN_LED_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(COOLING_RELAY_PIN, LOW);
  digitalWrite(LIGHT_RELAY_PIN, LOW);
}

void setupWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(KOLAB_SSID, KOLAB_PASSWORD);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! No OTA Possible...");
    blink(500, 4);
    digitalWrite(COOLING_RELAY_PIN, LOW);
    return;
  }
}

void reconnectWifi()
{
  // if WiFi is down, try reconnecting
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
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
  mqttDebugLog("setupOTA done");
}

void setupThermistor()
{
  float MAX_V_OVER_THERMISTOR = 1.65;
  float MAX_IN_VOLTAGE = 1;
  float MAX_RETURN_VAL = 1023;
  int SERIES_RESISTANCE = 66 * 1000; // 44K in series with 22K:22K voltage divider
  int REF_RESISTANCE = 25 * 1000;    // 25K measured when fridge was around 7 degrees
  int REF_TEMP = 6;
  int B_COEF = 3950;
  int NB_SAMPLES_FOR_AVG = 1;
  int SAMPLE_DELAY = 200;
  // Circuit diagram: https://www.falstad.com/circuit/circuitjs.html?ctz=CQAgjCAMB0l3BWK0BMkDMkBskAskBOLLdXAdlwA50RiQkFJ6BTAWjDACgA3cHcXLhC50KAUKZN8IdNBqTkCTgCcZ6LCBSVKw0Zu1RNaOJ3SNNCDVp1h+1w2HhN1kJK2P04yFFhRCAKswAtgAOzMoAhgAuAK7KzJwA7rpi9iJiYIJQKhZWBujq+jpS+CYhank6BRrqNoaQnOW2TPbNMlh1kkl8LQZtmRI5-VnVRYYoxg3lKJZjMzUd4PWcQA
  thermistor = new Thermistor(A0, MAX_V_OVER_THERMISTOR, MAX_IN_VOLTAGE, MAX_RETURN_VAL, SERIES_RESISTANCE, REF_RESISTANCE, REF_TEMP, B_COEF, NB_SAMPLES_FOR_AVG, SAMPLE_DELAY);
}
constexpr unsigned int mhash(const char *s, int off = 0)
{
  return !s[off] ? 5381 : (mhash(s, off + 1) * 33) ^ s[off];
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length)
{
  string topicString = string(topic);
  string debugMsg("received: ");
  debugMsg.append(topic);
  debugMsg.append(": ");

  string payloadString((char *)payload, length);
  string payloadStringCopy = payloadString;
  debugMsg.append(payloadStringCopy);
  mqttDebugLog(debugMsg);

  if (topicString.compare(string(mqtt_toggle_compressor_topic)) == 0)
  {
    fridgeState.toggleCompressor();
  }
  else if (topicString.compare(string(mqtt_toggle_fan_topic)) == 0)
  {
    fridgeState.toggleFan();
  }
  else if (topicString.compare(string(mqtt_toggle_light_topic)) == 0)
  {
    fridgeState.toggleLight();
  }
  else if (topicString.compare(string(mqtt_set_temp_range_topic)) == 0)
  {
    float wantedTempRange = stof(payloadString);
    fridgeState.setTempRange(wantedTempRange);
  }
  else if (topicString.compare(string(mqtt_set_temp_topic)) == 0)
  {
    float wantedTemp = stof(payloadString);
    fridgeState.setTemp(wantedTemp);
  }
  else if (topicString.compare(string(mqtt_disable_killswitch_topic)) == 0)
  {
    fridgeState.setKillswitch(false);
  }
  else if (topicString.compare(string(mqtt_enable_killswitch_topic)) == 0)
  {
    fridgeState.setKillswitch(true);
  }
  else
  {
    mqttDebugLog("Last MQTT Topic not recognized");
  }
}

void setupMQTT()
{
  client.setServer(MQTT_SERVER, MQTT_SERVERPORT);
  client.setCallback(mqttCallback);
  mqttReconnect();
  mqttDebugLog("MQTT connect Done");
  const char *topic = "IP ADDR:";
  mqttDebugLog(topic, WiFi.localIP().toString().c_str());
}

void mqttReconnect()
{
  // Loop until we're reconnected (max 3 tries)
  int tries = 0;
  while (!client.connected() && tries++ < 3)
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-Fridge";
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      // Once connected, publish an announcement...
      mqttDebugLog("connected");
      // ... and resubscribe
      client.subscribe(mqtt_toggle_compressor_topic);
      client.subscribe(mqtt_toggle_fan_topic);
      client.subscribe(mqtt_toggle_light_topic);
      client.subscribe(mqtt_set_temp_range_topic);
      client.subscribe(mqtt_set_temp_topic);
      client.subscribe(mqtt_enable_killswitch_topic);
      client.subscribe(mqtt_disable_killswitch_topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 500 millis");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}

void handleSafety()
{
  if (fridgeState.compressorTemp > 50)
  {
    fridgeState.setKillswitch(true);
    mqttDebugLog("ERROR: Compressor temp above 50 measured!! KILLING");
  }
  unsigned long minutesTurnedOn = (millis() - fridgeState.lastCompressorStateChange) / (1000 * 60);
  if (fridgeState.compressorState && minutesTurnedOn > 60 && minutesTurnedOn < 2 * 60) // We use minutesTurnedOn < 2 * 60 for when time would wrap around.
  {
    fridgeState.setKillswitch(true);
    mqttDebugLog("ERROR: Compressor has been on for 1 hour straight!! KILLING");
  }
}
void mqttDebugLog(const char *msg, const char *value = "")
{
  string buf = "";
  buf.append(msg);
  buf.append(value);
  mqttDebugLog(buf);
}

void mqttDebugLog(string msg)
{
  Serial.println(msg.c_str());
  client.publish(mqqt_debug_topic, msg.c_str());
}

void twoSecondsLoop()
{
  fridgeState.publishState();
  float temp = fridgeState.dht22Temp;
  if (!fridgeState.on())
  {
    return;
  }
  if (isnan(temp))
  {
    fridgeState.setCompressor(LOW);
    fridgeState.setFan(LOW);
    fridgeState.coolingMode = false;
    return;
  }
  float lowTemp = fridgeState.wantedTemp - (fridgeState.wantedTempRange / 2);
  float highTemp = fridgeState.wantedTemp + (fridgeState.wantedTempRange / 2);

  if (temp < lowTemp)
  {
    fridgeState.setCompressor(LOW);
    timer.setTimeout([=]()
                     {
        if (!fridgeState.coolingMode) {
          fridgeState.setFan(LOW);
        } },
                     60000);

    fridgeState.coolingMode = false;
  }
  else if (temp > highTemp)
  {
    fridgeState.setCompressor(HIGH);
    fridgeState.coolingMode = true;
    fridgeState.setFan(HIGH); // To Be extra sure
  }
  if (temp > highTemp - fridgeState.extraDegreesForFan)
  {
    fridgeState.setFan(HIGH);
    fridgeState.coolingMode = true;
  }
}