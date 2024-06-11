#pragma once

class Timer {
private:
    uint32_t interval;
    uint32_t lastTime;
public:
    Timer(uint32_t interval) : interval(interval) {}
    
    // @brief sets the interval of the timer
    void setInterval(uint32_t interval);

    uint32_t getInterval();
    
    // @brief resets the timer
    void reset();
    
    // @brief checks if the timer has elapsed
    // @return true if the timer has elapsed, false otherwise
    bool elapsed();
};