#pragma once

#include <Module.h>
#include <RadioType.h>
#include <Payload.h>

class PotentiometerModule : public Module {
private:
    uint8_t potentiometerPin;

    POTENTIOMETER_T potLastSent = 0;
    POTENTIOMETER_T readPot;

    POTENTIOMETER_T threshold;
public:
    PotentiometerModule(GLOBAL_ID_T defaultGlobalId, uint8_t cePin, uint8_t csPin, uint8_t connectButtonPin, uint8_t potentiometerPin, POTENTIOMETER_T threshold, uint8_t statusLedPin) : Module(defaultGlobalId, cePin, csPin, connectButtonPin, statusLedPin), potentiometerPin(potentiometerPin), threshold(threshold) {}

    void initSubmodule() override {}
    
    bool shouldSend() override {
        readPot = analogRead(potentiometerPin);
        if (abs((int)readPot - (int)potLastSent) > threshold) {
            Serial.print("current: ");
            Serial.print(readPot);
            Serial.print(" last: ");
            Serial.println(potLastSent);

            Serial.println(abs((int)readPot - (int)potLastSent));
            potLastSent = readPot;
            return true;
        } else {
            return false;
        }
    }

    void sendData() override {
        Payload payload;
        payload.global_id = globalId;
        payload.data = analogRead(potentiometerPin);
        safeWriteToMesh(&payload, (uint8_t)RadioType::POTENTIOMETER, sizeof(payload));
    }
};