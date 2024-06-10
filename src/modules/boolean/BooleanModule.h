#pragma once

#include <Module.h>
#include <RadioType.h>
#include <Payload.h>

class BooleanModule : public Module {
private:
    uint8_t BUTTON = 3;
    uint8_t buttonState;
    uint8_t prevButtonState = 0;

public:
    BooleanModule(GLOBAL_ID_T defaultGlobalId, uint8_t cePin, uint8_t csPin, uint8_t buttonPin, uint8_t statusLedPin) : Module(defaultGlobalId, cePin, csPin, buttonPin, statusLedPin) {}

    void initSubmodule() override {
        pinMode(BUTTON, INPUT_PULLUP);
    }
    
    bool shouldSend() override {
        buttonState = digitalRead(BUTTON);
        if (buttonState != prevButtonState) {
            prevButtonState = buttonState;
            return true;
        } else {
            return false;
        }
    }

    void sendData(bool force) override {
        Payload payload;
        payload.global_id = globalId;
        payload.data = digitalRead(BUTTON);
        safeWriteToMesh(&payload, (uint8_t)RadioType::BOOLEAN, sizeof(payload));
    }
    
    void handleRadioMessage(RF24NetworkHeader header) override {
        if (header.type == (uint8_t)RadioType::CHANGE_DELAY) {
            network.read(header, &sendInterval, sizeof(sendInterval));
        }
    }
};