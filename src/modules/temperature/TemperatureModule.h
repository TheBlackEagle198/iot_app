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
                              uint8_t statusLedPin,
                              TEMPERATURE_T temperatureThreshold = 1.0f,
                              HUMIDITY_T humidityThreshold = 1.0f) : Module(defaultGlobalId, cePin, csPin, connectButtonPin, statusLedPin), sensorPin(sensorPin), readTimer(2500), temperatureThreshold(temperatureThreshold), humidityThreshold(humidityThreshold) {}

    void initSubmodule() override {
        pinMode(sensorPin, INPUT);
    }
    
    bool readData() {
        if (!readTimer.elapsed()) {
            return false;
        }
        readTimer.reset();
        dhtSensor.read22(sensorPin);
        readTemp = dhtSensor.temperature;
        readHum = dhtSensor.humidity;
        return true;
    }

    bool shouldSend() override {
        if (!readData()) {
            return false;
        }
        shouldSendHum = false;
        shouldSendTemp = false;
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

    void sendData(bool force) override {
        Payload payload;
        payload.global_id = globalId;

        if (force) {
            shouldSendTemp = true;
            shouldSendHum = true;
            readData();
        }

        if (shouldSendTemp) {
            payload.data = int(readTemp * 10);
            safeWriteToMesh(&payload, (uint8_t)RadioType::TEMPERATURE, sizeof(payload));
        }
        if (shouldSendHum) {
            payload.data = int(readHum * 10);
            safeWriteToMesh(&payload, (uint8_t)RadioType::HUMIDITY, sizeof(payload));
        }
    }

    void handleRadioMessage(RF24NetworkHeader header, uint16_t incomingBytesCount) override {
        if (header.type == (uint8_t)RadioType::CHANGE_THRESHOLD) {
            char newIntervalBuffer[21];
            network.read(header, &newIntervalBuffer, incomingBytesCount);
            char* tempToken = strtok(newIntervalBuffer, "\n");
            uint32_t newThreshold;
            newThreshold = atof(tempToken);
            if (newThreshold > 0.0) {
                temperatureThreshold = newThreshold;
            }

            char* humToken = strtok(NULL, "\n");
            newThreshold = atof(humToken);
            if (newThreshold > 0.0) {
                humidityThreshold = newThreshold;
            }
        }
    }
};