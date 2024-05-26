#pragma once

/// @brief Radio message types
enum class RadioType {
    // ack is not sent by the receiver
    TEMPERATURE = 0,
    HUMIDITY,
    BOOLEAN,
    MOTOR_POSITION,
    GID_NEGOTIATION,
    POTENTIOMETER,

    // ack is sent by the receiver
    ACK_1 = 64,
    RADIO_LAST
};