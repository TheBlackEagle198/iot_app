
#include <Arduino.h>
#include "CronJob.h"

CronJob::CronJob(uint32_t interval, void (*callback)()) {
    this->_interval = interval;
    this->_callback = callback;
    this->_lastRun = millis();
}

void CronJob::init(uint32_t interval, void (*callback)()) {
    this->_interval = interval;
    this->_callback = callback;
    this->_lastRun = millis();
}

void CronJob::run() {
    if (millis() - this->_lastRun >= this->_interval) {
        this->_lastRun = millis();
        if (this->_callback != nullptr) {
            this->_callback();
        }
    }
}

void CronJob::oneshot() {
    if (this->_callback != nullptr) {
        this->_callback();
    }
}

void CronJob::updateInterval(uint32_t interval) {
    // reset the last run time
    this->_lastRun = millis();
    this->_interval = interval;
}
