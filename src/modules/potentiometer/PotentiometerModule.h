#pragma once

#include <Module.h>
#include <RadioType.h>

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
            return true;
        } else {
            return false;
        }
    }

    void sendData(bool force) override {
        readPot = analogRead(potentiometerPin);
        potLastSent = readPot;
        safeWriteToMesh(&potLastSent, (uint8_t)RadioType::POTENTIOMETER, sizeof(potLastSent));
    }

    void handleRadioMessage(RF24NetworkHeader header, uint16_t incomingBytesCount) override {
        if (header.type == (uint8_t)RadioType::CHANGE_THRESHOLD) {
            char newIntervalBuffer[10];
            network.read(header, &newIntervalBuffer, incomingBytesCount);
            uint32_t newThreshold;
            newThreshold = strtoul(newIntervalBuffer, NULL, 10); // convert the string to an integer (base 10)
            threshold = newThreshold;
            radioSendThreshold();
        }
    }

    void radioSendThreshold() override {
        char thresholdBuffer[10];
        itoa(threshold, thresholdBuffer, 10);
        safeWriteToMesh(thresholdBuffer, (uint8_t)RadioType::CHANGE_THRESHOLD, sizeof(thresholdBuffer));
    }
};