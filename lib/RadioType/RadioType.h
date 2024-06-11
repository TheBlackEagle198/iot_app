#pragma once

/// @brief Radio message types
enum class RadioType {
    // ack is not sent by the receiver

    TEMPERATURE = 0,
    HUMIDITY,
    BOOLEAN,
    POTENTIOMETER,
    GID_NEGOTIATION,

    // commands
    GET,
    CHANGE_STRATEGY,
    CHANGE_THRESHOLD,
    CHANGE_DELAY,
    // ack is sent by the receiver

    RADIO_LAST
};