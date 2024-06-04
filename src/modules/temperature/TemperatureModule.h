#pragma once

#include <Module.h>
#include <RadioType.h>
#include <Payload.h>
#include "dht.h"
#include "Timer.h"

class HumidityTemperatureModule : public Module {
private:
    uint8_t sensorPin;

    dht dhtSensor;

    Timer readTimer;

    TEMPERATURE_T temperatureThreshold;
    HUMIDITY_T humidityThreshold;
    
    TEMPERATURE_T readTemp = 0; ///< the current temperature reading
    TEMPERATURE_T tempLastSent = 0; ///< the previous temperature reading
    HUMIDITY_T readHum = 0; ///< the current humidity reading
    HUMIDITY_T humLastSent = 0; ///< the previous humidity reading

    bool shouldSendTemp = false;
    bool shouldSendHum = false;

public:
    HumidityTemperatureModule(GLOBAL_ID_T defaultGlobalId, 
                              uint8_t cePin, 
                              uint8_t csPin, 
                              uint8_t connectButtonPin, 
                              uint8_t sensorPin, 
                              TEMPERATURE_T temperatureThreshold = 1.0f,
                              HUMIDITY_T humidityThreshold = 1.0f) : Module(defaultGlobalId, cePin, csPin, connectButtonPin), sensorPin(sensorPin), readTimer(2500), temperatureThreshold(temperatureThreshold), humidityThreshold(humidityThreshold) {}

    void initSubmodule() override {
        pinMode(sensorPin, INPUT);
    }
    
    bool shouldSend() override {
        if (!readTimer.elapsed()) {
            return false;
        }
        readTimer.reset();
        shouldSendHum = false;
        shouldSendTemp = false;
        dhtSensor.read22(sensorPin);
        readTemp = dhtSensor.temperature;
        readHum = dhtSensor.humidity;
        Serial.println("Temperature: " + String(readTemp) + " Humidity: " + String(readHum));
        Serial.println(readTemp);
        if (abs(readTemp - tempLastSent) > temperatureThreshold) {
            tempLastSent = readTemp;
            shouldSendTemp = true;
        }
        if (abs(readHum - humLastSent) > humidityThreshold) {
            humLastSent = readHum;
            shouldSendHum = true;
        }
        return shouldSendHum || shouldSendTemp;
    }

    void sendData() override {
        Payload payload;
        payload.global_id = globalId;
        if (shouldSendTemp) {
            payload.data = int(readTemp * 10);
            safeWriteToMesh(&payload, (uint8_t)RadioType::TEMPERATURE, sizeof(payload));
        }
        if (shouldSendHum) {
            payload.data = int(readHum * 10);
            safeWriteToMesh(&payload, (uint8_t)RadioType::HUMIDITY, sizeof(payload));
        }
    }
};