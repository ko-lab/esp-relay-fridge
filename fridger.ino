int LED_PIN = 2;
int COOLING_RELAY_PIN = 14;
int FAN_RELAY_PIN = 16;

float offTemp = 12;
float extraDegreesForFan = 0.4; // offtemp+extraDegreesForFan*2 should always be smaller than onTemp

float onTemp = 14;
boolean coolingOn = false;
//assert(extraDegreesForFan > 0);
//assert(offTemp < onTemp);
//assert(offTemp + 2 * extraDegreesForFan < onTemp);
#include "DHT.h"
#define DHTPIN 5
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// Replace with your network credentials
const char* ssid = "Duvair24Ghz";
const char* password = "PASSWORD";

const int ESP_BUILTIN_LED = 2;


void setup(void) {
  pinMode(LED_PIN, OUTPUT);
  pinMode(COOLING_RELAY_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, LOW);
  // preparing GPIOs
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  dht.begin();

  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(FAN_RELAY_PIN, HIGH);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! No OTA Possible...");
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(COOLING_RELAY_PIN, LOW);
    return;
  }
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}

void loop(void) {
  ArduinoOTA.handle();
  float temp = dht.readTemperature(false);
  Serial.println("temp");

  Serial.println(temp);
  Serial.println("offTemp");
  Serial.println(offTemp);
  Serial.println("onTemp");
  Serial.println(onTemp);
  Serial.println("temp< offTemp ??");
  Serial.println(temp < offTemp);
  if (isnan(temp)) {
    digitalWrite(COOLING_RELAY_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }
  if (temp < offTemp) {
    Serial.println("temp< offTemp: yes");
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(COOLING_RELAY_PIN, LOW);
    coolingOn = false;
  }
  else if (temp > onTemp) {
    Serial.println("temp > onTemp: yes");
    digitalWrite(LED_PIN, LOW);
    digitalWrite(COOLING_RELAY_PIN, HIGH);
    coolingOn = true;
    digitalWrite(FAN_RELAY_PIN, HIGH); // To Be extra sure
  }
  if (temp > onTemp - extraDegreesForFan) {
    Serial.println("temp > onTemp-extraDegreesForFan: yes");
    digitalWrite(FAN_RELAY_PIN, HIGH);
    coolingOn = true;
  } else  if (temp > offTemp + extraDegreesForFan) {
    if (!coolingOn) {
      Serial.println("temp > onTemp-extraDegreesForFan: yes");
      digitalWrite(FAN_RELAY_PIN, LOW);
    }
  }
}
