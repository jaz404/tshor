#ifndef MAIN_CAN_FD_H
#define MAIN_CAN_FD_H

#include <Arduino.h>

namespace MainCanFD
{
    struct GripperState
    {
        // Normalized gripper position encoded as:
        // 0     = fully open
        // 10000 = fully closed
        uint16_t position = 0;

        bool valid = false;
        uint32_t lastRxMs = 0;
    };

    // Initialize CAN3 / CAN-FD main network.
    void begin();

    // Call every loop iteration.
    // Reads incoming CAN-FD traffic and handles RX timeout.
    void update();

    // Trigger -> main network, CAN-FD ID 0x11.
    void sendTriggerState(
        uint16_t triggerPos,
        uint16_t thumbX,
        uint16_t thumbY
    );

    // Latest gripper state received on CAN-FD ID 0x12.
    const GripperState& getGripperState();
}

#endif