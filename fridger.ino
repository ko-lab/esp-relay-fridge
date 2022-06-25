int gpio13Led = 13;
int gpio12Relay = 12;

float offTemp = 5.5;
float onTemp = 7.5;
#include "DHT.h"
#define DHTPIN 14
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

void setup(void) {
  // preparing GPIOs
  Serial.begin(115200);
  dht.begin();
  pinMode(gpio13Led, OUTPUT);
  pinMode(gpio12Relay, OUTPUT);
  digitalWrite(gpio13Led, LOW);
  digitalWrite(gpio12Relay, HIGH);
}

void loop(void) {
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
  delay(1000);
}
