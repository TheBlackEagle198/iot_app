#include "Button.h"
#include <Arduino.h>

void Button::init() {
    pinMode(m_pin, INPUT_PULLUP);
}

void Button::setPin(uint8_t pin) {
    m_pin = pin;
}

void Button::setOnShortPress(VoidCallBack onShortPress) {
    m_onShortPress = onShortPress;
}

void Button::setOnLongPress(VoidCallBack onLongPress) {
    m_onLongPress = onLongPress;
}

void Button::setShortPressDuration(uint32_t shortPressDuration) {
    m_shortPressDuration = shortPressDuration;
}

void Button::setLongPressDuration(uint32_t longPressDuration) {
    m_longPressDuration = longPressDuration;
}

void Button::run() {
    m_currentState = digitalRead(2);
    if (m_lastState == HIGH && m_currentState == LOW) {
        m_pressStart = millis();
    } else if (m_currentState == HIGH && m_lastState == LOW) {
        m_pressEnd = millis();
        m_currentPressDuration = m_pressEnd - m_pressStart;

        if (m_currentPressDuration < m_shortPressDuration) {
        m_onShortPress();
        } else {
        if (m_currentPressDuration > m_longPressDuration) {
            m_onLongPress();
        }
        }
    }
    m_lastState = m_currentState;
}
