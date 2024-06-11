#include <Arduino.h>
#include "Timer.h"

void Timer::setInterval(uint32_t interval) {
    this->interval = interval;
}

void Timer::reset() {
    lastTime = millis();
}

bool Timer::elapsed() {
    if (millis() - lastTime >= interval) {
        return true;
    }
    return false;
}

uint32_t Timer::getInterval() {
    return this->interval;
}