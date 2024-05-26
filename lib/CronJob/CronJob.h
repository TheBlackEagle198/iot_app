#pragma once
/// @brief A class that represents a cron job/timer
#include <stdint.h>
class CronJob {
private:
    uint32_t _interval;
    uint32_t _lastRun;
    void (*_callback)();
public:
    CronJob() : _interval(0), _lastRun(0), _callback(nullptr) {};
    CronJob(uint32_t interval, void (*callback)());
    void init(uint32_t interval, void (*callback)());
    void run();
    void oneshot();
    void updateInterval(uint32_t interval);
};