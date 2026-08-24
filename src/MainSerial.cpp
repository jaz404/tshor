#include "MainSerial.h"

#include <cstdlib>
#include <cstring>

namespace
{
    constexpr uint32_t SERIAL_SEND_INTERVAL_MS = 10;   // 100 Hz
    constexpr uint32_t GRIPPER_TIMEOUT_MS = 100;

    constexpr size_t RX_BUFFER_SIZE = 64;

    uint32_t lastSerialSendMs = 0;

    MainSerial::GripperState gripperState;

    char rxBuffer[RX_BUFFER_SIZE];
    size_t rxIndex = 0;


    void processLine(char* line)
    {
        // =========================================================
        // GRIPPER -> TRIGGER
        //
        // Format:
        //
        // G,stiffness,position
        //
        // Example:
        //
        // G,128,5320
        //
        // stiffness:
        //   0   = minimum resistance
        //   255 = maximum resistance
        //
        // position:
        //   0     = fully open
        //   10000 = fully closed
        // =========================================================

        if (std::strncmp(line, "G,", 2) != 0)
        {
            // Not a gripper communication packet.
            // Ignore normal Serial.print() debug messages.
            return;
        }

        char* token = std::strtok(line + 2, ",");

        if (token == nullptr)
        {
            return;
        }

        long receivedStiffness = std::strtol(token, nullptr, 10);


        token = std::strtok(nullptr, ",");

        if (token == nullptr)
        {
            return;
        }

        long receivedPosition = std::strtol(token, nullptr, 10);


        // ---------------------------------------------------------
        // Clamp stiffness
        // ---------------------------------------------------------

        if (receivedStiffness < 0)
        {
            receivedStiffness = 0;
        }

        if (receivedStiffness > 255)
        {
            receivedStiffness = 255;
        }


        // ---------------------------------------------------------
        // Clamp position
        // ---------------------------------------------------------

        if (receivedPosition < 0)
        {
            receivedPosition = 0;
        }

        if (receivedPosition > 10000)
        {
            receivedPosition = 10000;
        }


        // ---------------------------------------------------------
        // Update state
        // ---------------------------------------------------------

        gripperState.stiffness =
            static_cast<uint8_t>(receivedStiffness);

        gripperState.position =
            static_cast<uint16_t>(receivedPosition);

        gripperState.valid = true;
        gripperState.lastRxMs = millis();
    }
}


void MainSerial::begin()
{
    Serial.begin(115200);

    delay(100);

    Serial.println("Main Serial initialized.");
}


void MainSerial::update()
{
    while (Serial.available() > 0)
    {
        char c = static_cast<char>(Serial.read());

        // Ignore carriage return
        if (c == '\r')
        {
            continue;
        }


        // =========================================================
        // Complete line received
        // =========================================================

        if (c == '\n')
        {
            rxBuffer[rxIndex] = '\0';

            if (rxIndex > 0)
            {
                processLine(rxBuffer);
            }

            rxIndex = 0;

            continue;
        }


        // =========================================================
        // Store incoming character
        // =========================================================

        if (rxIndex < RX_BUFFER_SIZE - 1)
        {
            rxBuffer[rxIndex++] = c;
        }
        else
        {
            // Buffer overflow:
            // discard malformed line.
            rxIndex = 0;
        }
    }


    // =============================================================
    // Do not use stale gripper state
    // =============================================================

    if (
        gripperState.valid &&
        millis() - gripperState.lastRxMs > GRIPPER_TIMEOUT_MS
    )
    {
        gripperState.valid = false;

        // Safest fallback: no stiffness feedback.
        gripperState.stiffness = 0;
    }
}


void MainSerial::sendTriggerState(
    uint16_t triggerPos,
    uint16_t thumbX,
    uint16_t thumbY
)
{
    if (
        millis() - lastSerialSendMs <
        SERIAL_SEND_INTERVAL_MS
    )
    {
        return;
    }

    lastSerialSendMs = millis();


    // =============================================================
    // TRIGGER -> PC / GRIPPER
    //
    // Same values as CAN 0x11:
    //
    // triggerPos
    // thumbX
    // thumbY
    //
    // Serial representation:
    //
    // T,triggerPos,thumbX,thumbY
    //
    // Example:
    //
    // T,5320,5010,4980
    // =============================================================

    Serial.print("T,");
    Serial.print(triggerPos);
    Serial.print(",");
    Serial.print(thumbX);
    Serial.print(",");
    Serial.println(thumbY);
}


const MainSerial::GripperState&
MainSerial::getGripperState()
{
    return gripperState;
}