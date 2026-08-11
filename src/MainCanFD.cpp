#include "MainCanFD.h"
#include <FlexCAN_T4.h>

namespace
{
    // Main network is CAN3 / CAN-FD.
    FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> canFD;

    constexpr uint32_t TRIGGER_STATE_ID = 0x11;   // Trigger -> main/gripper side
    constexpr uint32_t GRIPPER_STATE_ID = 0x12;   // Main/gripper side -> trigger

    constexpr uint32_t CANFD_SEND_INTERVAL_MS = 10;   // 100 Hz trigger-state broadcast
    constexpr uint32_t GRIPPER_TIMEOUT_MS = 100;

    uint32_t lastCanSendMs = 0;
    MainCanFD::GripperState gripperState;
}

void MainCanFD::begin()
{
    canFD.begin();


    CANFD_timings_t config;
    config.clock = CLK_60MHz;
    config.baudrate = 1000000;    // 1Mbps Nominal speed
    config.baudrateFD = 5000000;  // 5Mbps Data speed
    config.propdelay = 190;
    config.bus_length = 1;
    config.sample = 70;
        
    canFD.setBaudRate(config);

    delay(100);
    Serial.println("Main CAN-FD initialized: 1 Mbps nominal / 5 Mbps data.");
}

void MainCanFD::update()
{
    canFD.events();

    CANFD_message_t rxMsg;

    while (canFD.read(rxMsg))
    {
        // =========================================================
        // CAN-FD ID 0x12 : GRIPPER STATE
        // Main/gripper side -> trigger controller
        //
        // Current structure:
        //   Byte 0 = gripper position [15:8]
        //   Byte 1 = gripper position [7:0]
        //
        // Position encoding:
        //   uint16_t, big-endian
        //   0     = fully open
        //   10000 = fully closed
        //
        // More fields (force/contact/velocity) can be added later.
        // =========================================================
        if (rxMsg.id == GRIPPER_STATE_ID &&
            !rxMsg.flags.extended &&
            rxMsg.len >= 2)
        {
            uint16_t receivedPosition =
                (static_cast<uint16_t>(rxMsg.buf[0]) << 8) |
                static_cast<uint16_t>(rxMsg.buf[1]);

            if (receivedPosition > 10000)
            {
                receivedPosition = 10000;
            }

            gripperState.position = receivedPosition;
            gripperState.valid = true;
            gripperState.lastRxMs = millis();
        }
    }

    // Do not use stale gripper position for bilateral control.
    if (gripperState.valid &&
        millis() - gripperState.lastRxMs > GRIPPER_TIMEOUT_MS)
    {
        gripperState.valid = false;
    }
}

void MainCanFD::sendTriggerState(
    uint16_t triggerPos,
    uint16_t thumbX,
    uint16_t thumbY
)
{
    if (millis() - lastCanSendMs < CANFD_SEND_INTERVAL_MS)
    {
        return;
    }

    lastCanSendMs = millis();

    CANFD_message_t txMsg{};
    txMsg.id = TRIGGER_STATE_ID;
    txMsg.flags.extended = 0;
    txMsg.edl = 1;
    txMsg.brs = 1;
    txMsg.len = 8;

    // =========================================================
    // CAN-FD ID 0x11 : TRIGGER STATE
    // Trigger controller -> main/gripper side
    //
    // Byte 0-1 : trigger position, uint16_t, big-endian
    //              0     = fully open
    //              10000 = fully closed
    //
    // Byte 2-3 : thumb X, uint16_t, big-endian
    //
    // Byte 4-5 : thumb Y, uint16_t, big-endian
    //
    // Byte 6   : bumperPressed (legacy slot)
    //            Unused on this trigger -> always 0
    //
    // Byte 7   : springForce (legacy slot)
    //            Unused for now -> always 0
    //
    // Keep the original 8-byte message layout so the main-side
    // decoder does not need to change as the controller evolves.
    // =========================================================

    txMsg.buf[0] = static_cast<uint8_t>((triggerPos >> 8) & 0xFF);
    txMsg.buf[1] = static_cast<uint8_t>(triggerPos & 0xFF);

    txMsg.buf[2] = static_cast<uint8_t>((thumbX >> 8) & 0xFF);
    txMsg.buf[3] = static_cast<uint8_t>(thumbX & 0xFF);

    txMsg.buf[4] = static_cast<uint8_t>((thumbY >> 8) & 0xFF);
    txMsg.buf[5] = static_cast<uint8_t>(thumbY & 0xFF);

    // Preserve original byte positions. These fields are unused for now.
    txMsg.buf[6] = 0; // bumperPressed
    txMsg.buf[7] = 0; // springForce

    canFD.write(txMsg);
}

const MainCanFD::GripperState& MainCanFD::getGripperState()
{
    return gripperState;
}