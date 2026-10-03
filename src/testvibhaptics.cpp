#include <Arduino.h>
#include "adafruitmotor.h"

AdafruitMotor motor;

const float TORQUE_DEADBAND = 0.05f;
const float TORQUE_MAX      = 0.30f;

const float ERROR_DEADBAND  = 0.004f;
 // not too sensitive should decrease this further but moteus does not move the motor back to true pos unless it is over a certain value....
 // also moteus gripper error is diff on either side due to gravity and weight 
const float ERROR_MAX       = 0.030f;

const uint8_t MIN_HAPTIC = 20;
const uint8_t MAX_HAPTIC = 127;


// Disable filtering for now while debugging
// We can add it back once everything works.


char rxLine[64];
uint8_t rxIndex = 0;

uint32_t lastFeedbackTime = 0;

const uint32_t FEEDBACK_TIMEOUT_MS = 200;

bool hapticActive = false;


float clampFloat(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}


float normalizeValue(
    float value,
    float deadband,
    float maximum)
{
    value = fabs(value);

    if (value <= deadband)
    {
        return 0.0f;
    }

    float normalized =
        (value - deadband) /
        (maximum - deadband);

    return clampFloat(
        normalized,
        0.0f,
        1.0f
    );
}


// ============================================================
// HAPTIC MAPPING
// ============================================================

uint8_t calculateHapticStrength(
    float torque,
    float positionError)
{
    float torqueLevel =
        normalizeValue(
            torque,
            TORQUE_DEADBAND,
            TORQUE_MAX
        );

    float errorLevel =
        normalizeValue(
            positionError,
            ERROR_DEADBAND,
            ERROR_MAX
        );


    Serial.print("[CALC] torqueLevel=");
    Serial.print(torqueLevel, 3);

    Serial.print(" errorLevel=");
    Serial.print(errorLevel, 3);


    // Torque is the main feedback signal
    float intensity =
        0.7f * torqueLevel +
        0.3f * errorLevel;

    intensity =
        clampFloat(
            intensity,
            0.0f,
            1.0f
        );


    Serial.print(" intensity=");
    Serial.print(intensity, 3);


    if (intensity <= 0.0f)
    {
        Serial.println(" strength=0");

        return 0;
    }


    uint8_t strength =
        MIN_HAPTIC +
        intensity *
        (MAX_HAPTIC - MIN_HAPTIC);


    Serial.print(" strength=");
    Serial.println(strength);


    return strength;
}


void processFeedback(
    float torque,
    float positionError)
{
    Serial.println();
    Serial.println("-----------------------------");

    Serial.print("[RX] Torque: ");
    Serial.print(torque, 4);
    Serial.println(" Nm");

    Serial.print("[RX] Error: ");
    Serial.print(positionError, 5);
    Serial.println(" rev");


    uint8_t strength =
        calculateHapticStrength(
            torque,
            positionError
        );


    Serial.print("[HAPTIC] Sending strength: ");
    Serial.println(strength);


    motor.setStrength(strength);


    if (strength > 0)
    {
        hapticActive = true;
    }
    else
    {
        hapticActive = false;
    }


    lastFeedbackTime = millis();

    Serial.println("-----------------------------");
}


// ============================================================
// SERIAL INPUT
// ============================================================

void readFeedback()
{
    while (Serial.available())
    {
        char c = Serial.read();


        if (c == '\n')
        {
            rxLine[rxIndex] = '\0';
            rxIndex = 0;


            Serial.print("[SERIAL] Received: \"");
            Serial.print(rxLine);
            Serial.println("\"");


            float torque;
            float positionError;


            int fields =
                sscanf(
                    rxLine,
                    "H,%f,%f",
                    &torque,
                    &positionError
                );


            Serial.print("[SERIAL] sscanf fields = ");
            Serial.println(fields);


            if (fields == 2)
            {
                Serial.println(
                    "[SERIAL] Valid haptic packet"
                );

                processFeedback(
                    torque,
                    positionError
                );
            }
            else
            {
                Serial.println(
                    "[SERIAL] INVALID PACKET"
                );
            }
        }
        else if (c != '\r')
        {
            if (rxIndex < sizeof(rxLine) - 1)
            {
                rxLine[rxIndex++] = c;
            }
            else
            {
                Serial.println(
                    "[SERIAL] BUFFER OVERFLOW"
                );

                rxIndex = 0;
            }
        }
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("=============================");
    Serial.println("Trigger Haptic Controller");
    Serial.println("=============================");


    // Uses Wire1 by default
    if (!motor.begin())
    {
        Serial.println(
            "[ERROR] Could not initialize haptic motor"
        );

        while (true)
        {
            delay(1000);
        }
    }


    Serial.println();
    Serial.println("[TEST] Testing built-in effect...");

    // motor.playEffect(1, 500);

    // delay(500);

    Serial.println("[TEST] Built-in effect complete");


    Serial.println();
    Serial.println("[TEST] Testing RTP at 100...");

    // motor.vibrateStrong(1000);

    Serial.println("[TEST] RTP test complete");


    delay(500);


    // IMPORTANT:
    // Enter RTP once and remain there.
    motor.startRealtime();


    Serial.println();
    Serial.println("[READY] Waiting for packets");
    Serial.println("[READY] Expected:");
    Serial.println("H,<torque>,<error>");
    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    readFeedback();


    // Safety timeout
    if (
        lastFeedbackTime != 0 &&
        millis() - lastFeedbackTime >
            FEEDBACK_TIMEOUT_MS
    )
    {
        if (hapticActive)
        {
            Serial.println(
                "[TIMEOUT] Feedback stopped -> haptic OFF"
            );

            motor.setStrength(0);

            hapticActive = false;
        }
    }
}