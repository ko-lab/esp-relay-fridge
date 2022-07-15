int LED_PIN = 2;
int COOLING_RELAY_PIN = 14;
int FAN_RELAY_PIN = 16;

float offTemp = 6;
float extraDegreesForFan = 0.4; // offtemp+extraDegreesForFan*2 should always be smaller than onTemp
boolean ON_STATE = 1;
float onTemp = 9;
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
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#define AIO_SERVER      "10.88.78.186"
#define AIO_SERVERPORT  1884

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT);
Adafruit_MQTT_Publish fridge = Adafruit_MQTT_Publish(&mqtt, "/feeds/fridge/appliance_temp");
Adafruit_MQTT_Publish fridge_state = Adafruit_MQTT_Publish(&mqtt, "/feeds/fridge/state");

Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, "/feeds/fridge/onoff");

// Replace with your network credentials
const char* ssid = "ko-lab-iot";
const char* password = "PASSWORD";

const int ESP_BUILTIN_LED = 2;

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
unsigned long lastPublish = 0;
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
  mqtt.subscribe(&onoffbutton);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_PIN, ON_STATE? HIGH: LOW);

}

void loop(void) {
  ArduinoOTA.handle();
  float temp = dht.readTemperature(false);
  handleMQTT(temp);
  if (!ON_STATE) {
    digitalWrite(COOLING_RELAY_PIN, LOW);
    digitalWrite(FAN_RELAY_PIN, LOW);
    return;
  }
  if (isnan(temp)) {
    digitalWrite(COOLING_RELAY_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }
  if (temp < offTemp) {
    digitalWrite(COOLING_RELAY_PIN, LOW);
    coolingOn = false;
  }
  else if (temp > onTemp) {
    digitalWrite(COOLING_RELAY_PIN, HIGH);
    coolingOn = true;
    digitalWrite(FAN_RELAY_PIN, HIGH); // To Be extra sure
  }
  if (temp > onTemp - extraDegreesForFan) {
    digitalWrite(FAN_RELAY_PIN, HIGH);
    coolingOn = true;
  } else  if (temp > offTemp + extraDegreesForFan) {
    if (!coolingOn) {
      digitalWrite(FAN_RELAY_PIN, LOW);
    }
  }
}
void handleMQTT(float temp) {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  Adafruit_MQTT_Subscribe *subscription;
  if((subscription = mqtt.readSubscription(100))) {
    if (subscription == &onoffbutton) {
        ON_STATE = !ON_STATE;
        digitalWrite(LED_PIN, ON_STATE? HIGH: LOW);
        fridge_state.publish(ON_STATE? "HIGH": "LOW");
    }
  }


  unsigned long nowTime = millis();
  if (nowTime - lastPublish > 2000) {
    if (!fridge.publish(temp)) {
      Serial.println(F("Failed"));
      delay(500);
      digitalWrite(LED_PIN, LOW);
      delay(500);
      digitalWrite(LED_PIN, HIGH);
    } else {
      Serial.println(F("OK!"));
    }
    lastPublish = millis();
  }
}
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      //      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
