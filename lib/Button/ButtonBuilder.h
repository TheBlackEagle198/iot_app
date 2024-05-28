#pragma once
#include "Button.h"

class ButtonBuilder {
private:
  Button m_button;

public:
    ButtonBuilder();
    ButtonBuilder pin(uint8_t pin);
    ButtonBuilder onShortPress(VoidCallBack onShortPress);
    ButtonBuilder onLongPress(VoidCallBack onLongPress);
    ButtonBuilder shortPressDuration(uint32_t shortPressDuration);
    ButtonBuilder longPressDuration(uint32_t longPressDuration);
    Button build();
};