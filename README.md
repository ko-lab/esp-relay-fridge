# ESP32 Relay Fridge controller with MQTT

This projects contains documentation and code regarding our self made Fridge controller board using an esp32 relay board that reads the fridges sensor data and then controll the compressor, fan and light.
This is then combined with some MQTT functionality to publish the fridge sensor data and receive commands.

We used this on an old drinks fridge where the relays on the controller board had given out while the rest of the fridge was still ok.

# Functionality
- Can fix a fridge if only the controller board is broken
- Upgrades your fridge to be remote controllable over MQTT
- Upgrades your fridge to send out sensor data via MQTT
- OTA Flashable


# Building

Before building the project, configure the correct WiFi password (not committed to the repo).

Create a file `.env` in the project root directory and add the following key/value pairs:

```
KOLAB_SSID=ko-lab-iot
KOLAB_PASSWORD=password
```


# How to update the software

For the OTA updates, check: https://randomnerdtutorials.com/esp8266-ota-updates-with-arduino-ide-over-the-air/#:~:text=ESP8266%20OTA%20Updates%20with%20Arduino,access%20to%20the%20ESP%20module.
