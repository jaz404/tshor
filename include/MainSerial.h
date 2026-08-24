#pragma once

#include <Arduino.h>

namespace MainSerial
{
    struct GripperState
    {
        uint8_t stiffness = 0;   // 0-255
        uint16_t position = 0;   // 0-10000

        bool valid = false;
        uint32_t lastRxMs = 0;
    };

    void begin();

    void update();

    void sendTriggerState(
        uint16_t triggerPos,
        uint16_t thumbX,
        uint16_t thumbY
    );

    const GripperState& getGripperState();
}