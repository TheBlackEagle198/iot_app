#include "ButtonBuilder.h"

ButtonBuilder::ButtonBuilder() {
    m_button = Button();
}

ButtonBuilder ButtonBuilder::pin(uint8_t pin) {
    m_button.setPin(pin);
    return *this;
}

ButtonBuilder ButtonBuilder::onShortPress(VoidCallBack onShortPress) {
    m_button.setOnShortPress(onShortPress);
    return *this;
}

ButtonBuilder ButtonBuilder::onLongPress(VoidCallBack onLongPress) {
    m_button.setOnLongPress(onLongPress);
    return *this;
}

ButtonBuilder ButtonBuilder::shortPressDuration(uint32_t shortPressDuration) {
    m_button.setShortPressDuration(shortPressDuration);
    return *this;
}

ButtonBuilder ButtonBuilder::longPressDuration(uint32_t longPressDuration) {
    m_button.setLongPressDuration(longPressDuration);
    return *this;
}

Button ButtonBuilder::build() {
    return m_button;
}
