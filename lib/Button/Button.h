#pragma once

#include "GlobalConfig.h"
#include <stdint.h>

class Button {
private:
    uint8_t m_pin;
    uint8_t m_currentState;
    uint8_t m_lastState;
    uint32_t m_pressStart;
    uint32_t m_pressEnd;
    uint32_t m_shortPressDuration;
    uint32_t m_longPressDuration;
    uint32_t m_currentPressDuration;

    VoidCallBack m_onShortPress;
    VoidCallBack m_onLongPress;
public:
    void init();
    void setPin(uint8_t pin);
    void setOnShortPress(VoidCallBack onShortPress);
    void setOnLongPress(VoidCallBack onLongPress);
    void setShortPressDuration(uint32_t shortPressDuration);
    void setLongPressDuration(uint32_t longPressDuration);
    void run();
};