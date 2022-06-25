int gpio13Led = 13;
int gpio12Relay = 12;

float offTemp = 5.5;
float onTemp = 7.5;
#include "DHT.h"
#define DHTPIN 14
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
  // preparing GPIOs
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  dht.begin();
  pinMode(gpio13Led, OUTPUT);
  pinMode(gpio12Relay, OUTPUT);
  digitalWrite(gpio13Led, LOW);
  digitalWrite(gpio12Relay, HIGH);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! No OTA Possible...");
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
  if (temp < offTemp) {
    Serial.println("temp< offTemp: yes");
    digitalWrite(gpio13Led, HIGH);
    digitalWrite(gpio12Relay, LOW);
  } else if (temp > onTemp) {
    Serial.println("temp > onTemp: yes");
    digitalWrite(gpio13Led, LOW);
    digitalWrite(gpio12Relay, HIGH);
  }
}
