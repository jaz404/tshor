#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "OT2206.h"

// check 'feedback' is it getting updated as expected? same rater as damping period? 
// check if using torque mode directly better than is being done right now

// with t mode the movement is much more stronger and smoother? 
// also the measured A is much lower than cmd A

constexpr float POSITION_GAIN_A_PER_DEG = 0.25f;
float targetPositionDeg = 0.0f;

bool dampingEnabled = false;

constexpr uint32_t DAMPING_PERIOD_US  = 1000;           // 1/1000us = 1 KHz freq
constexpr float DAMPING_GAIN = 2.00f;
constexpr float STATIC_RESISTANCE_A = 0.5f;
constexpr float MAX_DAMPING_CURRENT_A = 1.5f;
constexpr float VELOCITY_DEADBAND_RAD_S = 0.05f;
uint32_t lastDampingCommandUs = 0;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

OT2206CAN<decltype(Can1)>::Config motorConfig{
    .motorId = 1,
    .hostId = 100,
    .pmaxTurns = 1.0f,
    .speedFullScaleRadS = 200.0f, // Replace with actual OT2206 full scale.
    .currentFullScaleA = 4.0f     // Replace with actual OT2206 full scale.
};

OT2206CAN<decltype(Can1)> motor(Can1, motorConfig);
OT2206CAN<decltype(Can1)>::Feedback feedback;

static void printMenu() {
    Serial.println("\nOT2206 classical CAN test");
    Serial.println("1: normal/absolute CAN control mode");
    Serial.println("2: extended/incremental CAN control mode");
    Serial.println("p: position mode example");
    Serial.println("s: speed mode example");
    Serial.println("t: torque/current mode example");
    Serial.println("r: start motor (setpoint must already be sent)");
    Serial.println("x: stop and free motor");
    Serial.println("z: temporary mechanical zero (motor must be stopped)");
    Serial.println("i: set all PI multipliers to 1.0");
    Serial.println("f: print latest feedback");
    Serial.println("?: menu");
}

static void printFeedback(const decltype(feedback) &f) {
    if (!f.valid) {
        Serial.println("No valid feedback received yet.");
        return;
    }
    Serial.printf("motor=%u dlc=%u mode=%s pos=%.3f deg speed=%.3f rad/s current=%.3f A",
                  f.motorId,
                  f.dlc,
                  f.incrementalMode ? "incremental" : "absolute",
                  f.positionDeg,
                  f.speedRadS,
                  f.currentA);
    if (f.incrementalMode) {
        Serial.printf(" counts=%ld", static_cast<long>(f.positionCounts));
    }
    Serial.println();
}

void setup() {

    #if CONFIG
            Serial.begin(115200);      // USB
            Serial5.begin(19200);      // Motor TTL

            while (!Serial) {}

            Serial.println("Serial bridge ready.");

    #else
        Serial.begin(115200);
        while (!Serial && millis() < 3000) {}

        Can1.begin();
        Can1.setBaudRate(1000000); // Manual specifies 1 Mbit/s classical CAN.
        Can1.setMaxMB(16);
        Can1.enableFIFO();

        // Accept the configured host response ID. Standard response frames are used.
        Can1.setFIFOFilter(REJECT_ALL);
        Can1.setFIFOFilter(0, motorConfig.hostId, STD);         // only accept from host id 1

        delay(100);
        Serial.println("CAN initialized at 1 Mbit/s.");
        printMenu();
    
    #endif
}
void loop()
{
    #if CONFIG
        while (Serial.available())
        {
        char c = Serial.read();
        Serial5.write(c);
        }

        // Motor -> PC
        while (Serial5.available())
        {
        char c = Serial5.read();
        Serial.write(c);
        }
    #else
        Can1.events();

        /*
        * Read all CAN responses.
        *
        * Each torque command should cause the motor to return its current
        * position, velocity, and current.
        */
        CAN_message_t rx;

        while (Can1.read(rx))
        {
            if (motor.parseFeedback(rx, feedback))
            {
                // Avoid printing every frame during damping mode.
                if (!dampingEnabled)
                {
                    printFeedback(feedback);
                }
            }
            else
            {
                Serial.printf(
                    "Unparsed RX: ID=%lu LEN=%u EXT=%u DATA=",
                    rx.id,
                    rx.len,
                    rx.flags.extended
                );

                for (uint8_t i = 0; i < rx.len; i++)
                {
                    Serial.printf("%02X ", rx.buf[i]);
                }

                Serial.println();
            }
        }

        /*
        * Trigger controller:
        *
        * - Low-gain position term returns the trigger toward its starting position.
        * - Damping/static resistance is applied only for positive velocity.
        * - The current limit keeps the trigger manually movable.
        */
        if (dampingEnabled)
        {
            const uint32_t nowUs = micros();

            if (nowUs - lastDampingCommandUs >= DAMPING_PERIOD_US)
            {
                lastDampingCommandUs += DAMPING_PERIOD_US;

                float commandedCurrentA = 0.0f;
                float positionErrorDeg = 0.0f;

                if (feedback.valid)
                {
                    positionErrorDeg =
                        targetPositionDeg - feedback.positionDeg;

                    // New position-return term
                    commandedCurrentA =
                        POSITION_GAIN_A_PER_DEG * positionErrorDeg;

                    // Existing one-direction damping
                    if (feedback.speedRadS > VELOCITY_DEADBAND_RAD_S)
                    {
                        commandedCurrentA +=
                            -STATIC_RESISTANCE_A
                            -DAMPING_GAIN * feedback.speedRadS;
                    }

                    commandedCurrentA = constrain(
                        commandedCurrentA,
                        -MAX_DAMPING_CURRENT_A,
                        MAX_DAMPING_CURRENT_A
                    );
                }

                const bool ok =
                    motor.commandTorque(commandedCurrentA);

                static uint32_t lastDampingPrintMs = 0;

                if (millis() - lastDampingPrintMs >= 200)
                {
                    lastDampingPrintMs = millis();

                    Serial.printf(
                        "TRIGGER: target=%+.2f deg "
                        "pos=%+.2f deg "
                        "err=%+.2f deg "
                        "vel=%+.3f rad/s "
                        "cmd=%+.3f A "
                        "measured=%+.3f A "
                        "TX=%s\n",
                        targetPositionDeg,
                        feedback.positionDeg,
                        positionErrorDeg,
                        feedback.speedRadS,
                        commandedCurrentA,
                        feedback.currentA,
                        ok ? "OK" : "FAILED"
                    );
                }
            }
        }
        /*
        * Do not return when Serial is empty because the damping controller
        * must continue running.
        */
        if (!Serial.available())
        {
            return;
        }
        const char cmd = static_cast<char>(Serial.read());
        switch (cmd)
        {
            case '1':
                dampingEnabled = false;

                Serial.println(
                    motor.setAddressMode(
                        decltype(motor)::AddressMode::NormalAbsolute
                    )
                        ? "Normal absolute mode requested."
                        : "CAN write failed."
                );
                break;

            case '2':
                dampingEnabled = false;

                Serial.println(
                    motor.setAddressMode(
                        decltype(motor)::AddressMode::ExtendedIncremental
                    )
                        ? "Extended incremental mode requested."
                        : "CAN write failed."
                );
                break;

            case 'p':
            {
                dampingEnabled = false;

                bool ok = motor.selectPositionSpeedTorqueMode();
                delay(5);

                ok &= motor.commandPosition(
                    targetPositionDeg, // degrees
                    10.0f, // maximum rad/s
                    0.50f,  // maximum current A -- this is directly changing the feel of the motor 0.25 feels much lighter
                    1,   // Kp
                    0     // Kd
                );


                Serial.println(
                    ok
                        ? "Position command loaded. Press r to start."
                        : "CAN write failed."
                );
                break;
            }
             case 'o':
            {
                dampingEnabled = false;

                bool ok = motor.selectPositionSpeedTorqueMode();
                delay(5);

                ok &= motor.commandPosition(
                    targetPositionDeg, // degrees
                    10.0f, // maximum rad/s
                    0.20f,  // maximum current A -- this is directly changing the feel of the motor 0.25 feels much lighter
                    1,   // Kp
                    0     // Kd
                );


                Serial.println(
                    ok
                        ? "Position command loaded. Press r to start."
                        : "CAN write failed."
                );
                break;
            }

            case 's':
            {
                dampingEnabled = false;

                bool ok = motor.selectSpeedTorqueMode();
                delay(5);

                ok &= motor.commandSpeed(
                    100.0f, // rad/s
                    0.5f    // current limit A
                );

                Serial.println(
                    ok
                        ? "Speed command loaded. Press r to start."
                        : "CAN write failed."
                );
                break;
            }

            case 't':
            {
                dampingEnabled = false;

                bool ok = motor.selectTorqueMode();
                delay(5);

                ok &= motor.commandTorque(1.00f);

                Serial.println(
                    ok
                        ? "Torque command loaded. Press r to start."
                        : "CAN write failed."
                );
                break;
            }

            case 'd':
            {
                /*
                * Start with zero current so the motor does not suddenly move.
                */
                dampingEnabled = false;

                bool ok = motor.stopFree();
                delay(10);

                ok &= motor.selectTorqueMode();
                delay(10);

                ok &= motor.commandTorque(0.0f);
                delay(10);  

                // need to set speed pos torque before starting motor!!
                ok &= motor.start();
                delay(10);

                if (ok)
                {
                    feedback.valid = false;
                    // targetPositionDeg = feedback.positionDeg;
                    dampingEnabled = true;
                    lastDampingCommandUs = micros();

                    Serial.println(
                        "Damping enabled. Rotate the motor by hand."
                    );
                    Serial.println(
                        "Press x to stop and free the motor."
                    );
                }
                else
                {
                    motor.stopFree();
                    Serial.println("Failed to enable damping mode.");
                }

                break;
            }

            case 'r':
                Serial.println(
                    motor.start()
                        ? "Start sent."
                        : "Start blocked: send a setpoint first or CAN write failed."
                );
                break;

            case 'x':
                dampingEnabled = false;

                // Command zero current before freeing the motor.
                motor.commandTorque(0.0f);
                delay(5);

                Serial.println(
                    motor.stopFree()
                        ? "Damping disabled; stop/free sent."
                        : "CAN write failed."
                );
                break;

            case 'z':
                targetPositionDeg = feedback.positionDeg;

                dampingEnabled = false;

                Serial.println(
                    motor.setTemporaryMechanicalZero()
                        ? "Temporary zero sent."
                        : "CAN write failed."
                );
                break;

            case 'i':
            {
                dampingEnabled = false;

                bool ok = true;

                ok &= motor.setPIMultiplier(
                    decltype(motor)::PIParameter::SpeedKp,
                    1.0f
                );

                ok &= motor.setPIMultiplier(
                    decltype(motor)::PIParameter::SpeedKi,
                    1.0f
                );

                ok &= motor.setPIMultiplier(
                    decltype(motor)::PIParameter::CurrentKp,
                    1.0f
                );

                ok &= motor.setPIMultiplier(
                    decltype(motor)::PIParameter::CurrentKi,
                    1.0f
                );

                Serial.println(
                    ok
                        ? "PI multipliers sent."
                        : "At least one CAN write failed."
                );
                break;
            }

            case 'f':
                printFeedback(feedback);
                break;

            case '?':
                printMenu();
                break;

            default:
                break;
        }
    #endif
}