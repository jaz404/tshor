#include "MainSerial.h"

#include <cstring>

namespace
{
    constexpr uint32_t SERIAL_SEND_INTERVAL_MS = 2;   // 500 Hz
    constexpr uint32_t GRIPPER_TIMEOUT_MS = 100;

    constexpr uint8_t GRIPPER_PACKET_SIZE = 9;

    uint32_t lastSerialSendMs = 0;

    MainSerial::GripperState gripperState;

    uint8_t rxPacket[GRIPPER_PACKET_SIZE];
    uint8_t rxIndex = 0;


    void processByte(uint8_t byte)
    {
        // ----------------------------------------------------
        // Wait for start byte
        // ----------------------------------------------------
        if (rxIndex == 0)
        {
            if (byte == 'G')
            {
                rxPacket[0] = byte;
                rxIndex = 1;
            }

            return;
        }


        // ----------------------------------------------------
        // Collect rest of packet
        // ----------------------------------------------------
        rxPacket[rxIndex++] = byte;

        if (rxIndex < GRIPPER_PACKET_SIZE)
        {
            return;
        }

        // ----------------------------------------------------
        // Decode packet
        //
        // G
        // float positionError
        // float torque
        // ----------------------------------------------------

        float positionError;
        float torque;

        memcpy(
            &positionError,
            &rxPacket[1],
            sizeof(float)
        );

        memcpy(
            &torque,
            &rxPacket[5],
            sizeof(float)
        );

        rxIndex = 0;


        // ----------------------------------------------------
        // Basic sanity check
        // ----------------------------------------------------

        if (!isfinite(positionError) ||
            !isfinite(torque))
        {
            return;
        }


        gripperState.positionError = positionError;
        gripperState.torque = torque;

        gripperState.valid = true;
        gripperState.lastRxMs = millis();
    }
}


void MainSerial::begin()
{
    Serial.begin(115200);

    delay(100);
}


void MainSerial::update()
{
    while (Serial.available() > 0)
    {
        uint8_t byte =
            static_cast<uint8_t>(Serial.read());

        processByte(byte);
    }


    // Mark feedback invalid if the PC/follower stops responding
    if (
        gripperState.valid &&
        millis() - gripperState.lastRxMs >
            GRIPPER_TIMEOUT_MS
    )
    {
        gripperState.valid = false;
    }
}

void MainSerial::sendTriggerState(
    uint16_t triggerPos,
    uint16_t thumbX,
    uint16_t thumbY
)
{
    (void)thumbX;
    (void)thumbY;

    if (
        millis() - lastSerialSendMs <
        SERIAL_SEND_INTERVAL_MS
    )
    {
        return;
    }

    lastSerialSendMs = millis();

    if (triggerPos > 10000)
    {
        triggerPos = 10000;
    }

    uint8_t packet[3];

    packet[0] = 'T';
    packet[1] =
        static_cast<uint8_t>(triggerPos & 0xFF);

    packet[2] =
        static_cast<uint8_t>((triggerPos >> 8) & 0xFF);

    Serial.write(packet, sizeof(packet));
}


const MainSerial::GripperState&
MainSerial::getGripperState()
{
    return gripperState;
}