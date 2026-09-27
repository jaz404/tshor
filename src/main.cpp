#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "OT2206.h"
#include "adafruitmotor.h"
#include "BoardConfig.h"
#include "MainCanFD.h"  
#include "MainSerial.h" // using serial for now instead of CAN for now!

AdafruitMotor haptic;

// TIMING

constexpr uint32_t CONTROL_PERIOD_US = 500;         // 2kHz local motor-current loop

uint32_t lastFeedbackPollUs = 0;
uint32_t lastControlCommandUs = 0;
uint32_t lastDampingCommandUs = 0;

static uint32_t lastPrintMs = 0;

// TRIGGER CONTROL

// TODO: these may change over time: ideal fix - recalib everytime on boot
constexpr float TRIGGER_OPEN_DEG = 715.0f;
constexpr float TRIGGER_CLOSED_DEG = 5.0f;

uint8_t STIFFNESS_ACTIVATION_THRES = 50;

// P-P controller gain.
// Input error is normalized (0.0 to 1.0), output is motor current in A.
constexpr float PP_MAX_CURRENT_A = 2.0f;

// CONTROL STATE

float targetPositionDeg = 0.0f;

bool dampingEnabled = false;
bool ppControlEnabled = true;
bool ppMotorStarted = false;


// Inputs

uint16_t thumbX = 0;
uint16_t thumbY = 0;


// OT2206 motor (connected to CAN1)

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

OT2206CAN<decltype(Can1)>::Config motorConfig{
    .motorId = 1,
    .hostId = 100,
    .pmaxTurns = 2.0f,            // Do I need to increase this?
    .speedFullScaleRadS = 200.0f,
    .currentFullScaleA = 4.0f
};

OT2206CAN<decltype(Can1)> motor(Can1, motorConfig);

OT2206CAN<decltype(Can1)>::Feedback feedback;

constexpr float TRIGGER_REDUCTION = 14.0f;

// HELPERS

#ifdef TESTING
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
#endif

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

#ifndef TESTING
// Convert local OT2206 trigger angle to a common 0...10000 position.
static uint16_t getTriggerPositionNormalized()
{
    if (!feedback.valid)
    {
        return 0;
    }

    const float denominator = TRIGGER_CLOSED_DEG - TRIGGER_OPEN_DEG;

    if (fabsf(denominator) < 1e-6f)
    {
        return 0;
    }

    float normalized =
        (feedback.positionDeg - TRIGGER_OPEN_DEG) / denominator;

    normalized = constrain(normalized, 0.0f, 1.0f);

    return static_cast<uint16_t>(normalized * 10000.0f);
}

// =========================================================
// P-P CONTROL
// =========================================================

// Position-mode parameters used for bilateral P-P control.
// The gripper position received over CAN-FD becomes the desired
// trigger position. The current argument limits how strongly the
// trigger motor is allowed to pull toward that position.
constexpr float PP_MAX_SPEED_RAD_S = 5.0f;
constexpr float PP_KP = 0.0f;
constexpr float PP_KD = 0.0f;

// Current limit used by OT2206 position mode for the first P-P version.
// Later this can be changed using gripper force/contact/error to make
// the trigger feel stiffer when the gripper interacts with an object.

static float getHapticMotorCurrent()
{
    const auto& gripper = MainSerial::getGripperState();

    if (!gripper.valid)
    {
        return 0.0f;
    }

    // A / gripper revolution
    constexpr float KP = 100.0f;

    constexpr float MAX_CURRENT_A = 4.0f;

    // Flip this if the trigger pushes in the wrong direction.
    constexpr float HAPTIC_SIGN = 1.0f;

    float current =
        HAPTIC_SIGN *
        KP *
        gripper.positionError;

    return constrain(
        current,
        -MAX_CURRENT_A,
        MAX_CURRENT_A
    );
}
static void runMotor()
{
    const uint32_t nowUs = micros();

    if (
        nowUs - lastControlCommandUs <
        CONTROL_PERIOD_US
    )
    {
        return;
    }

    lastControlCommandUs = nowUs;


    if (!ppMotorStarted)
    {
        bool ok = true;

        ok &= motor.stopFree();
        delay(5);

        ok &= motor.selectTorqueMode();
        delay(5);

        // Initial zero-current command
        // also gets feedback flowing.
        ok &= motor.commandTorque(0.0f);
        delay(5);

        ok &= motor.start();

        if (!ok)
        {
            return;
        }

        ppMotorStarted = true;
        return;
    }

    if (!feedback.valid)
    {
        motor.commandTorque(0.0f);
        return;
    }

    float hapticCurrentA =
        getHapticMotorCurrent();

        static uint32_t lastPrintMs = 0;

    if (millis() - lastPrintMs >= 100)
    {
        lastPrintMs = millis();

        SerialUSB1.printf(
            "hapticCurrentA = %.3f A\n",
            hapticCurrentA
        );
    }

    // apply opposing torque 
    motor.commandTorque(
        hapticCurrentA
    );
}
#endif

void setup() {

    #if CONFIG
            Serial.begin(115200);      // USB
            Serial5.begin(19200);      // Motor TTL

            while (!Serial) {}

            Serial.println("Serial bridge ready.");

    #else
        // Serial.begin(115200);
        MainSerial::begin();
        while (!Serial && millis() < 3000) {}

        // Setup motor CAN
        Can1.begin();
        Can1.setBaudRate(1000000); // Manual specifies 1 Mbit/s classical CAN.
        Can1.setMaxMB(16);
        Can1.enableFIFO();

        // Accept the configured host response ID. Standard response frames are used.
        Can1.setFIFOFilter(REJECT_ALL);
        Can1.setFIFOFilter(0, motorConfig.hostId, STD);         // only accept from host id 1

        delay(100);
        Serial.println("OT2206 CAN initialized at 1 Mbit/s.");

        if (!haptic.begin())
        {
            Serial.println("DRV2605L not detected.");
            while (true)
            {
                delay(100);
            }
        }

        Serial.println("DRV2605L Haptics ready.");
        // delay(5000);
        // MainCanFD::begin();


        #ifdef TESTING
        printMenu();
        #endif

        // SerialUSB1.begin(115200);

        // delay(500);

        // SerialUSB1.println("Debug serial ready");

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
        // Motor CAN
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
                #ifdef TESTING
                printFeedback(feedback);
                #endif

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

    
        #ifdef TESTING
            if (Serial.available()) {

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
                        25.0f,  // maximum rad/s
                        4.00f,  // maximum current A -- this is directly changing the feel of the motor 0.25 feels much lighter
                        50,      // Kp
                        5       // Kd
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
                        10.0f,   // maximum rad/s
                        1.00f,   // maximum current A -- this is directly changing the feel of the motor 0.25 feels much lighter
                        1,       // Kp
                        0        // Kd
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

                    ok &= motor.commandTorque(4.00f);

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
                case 'v':
                    Serial.println("haptic motor test");
                    
                    // haptic.vibrateStrong(1000); // also good
                    // haptic.playEffect(118, 250); // very good (strongest)
                    // haptic.playEffect(38,200);
                    // haptic.playEffect(47, 1000); decent feel
                        
                    //  haptic.playEffect(70, 1000);
                haptic.testStrengths();

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
                    targetPositionDeg = feedback.positionDeg;
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
            }
            #else
            // =====================================================
            // CAN3 FD (Serial for now!)
            // =====================================================
            
            // Read latest gripper position (ID 0x12) and update timeout state.
            // MainCanFD::update();

            // // analog read the joystick each iter
            // thumbX = analogRead(Pins::JOY_X);
            // thumbY = analogRead(Pins::JOY_Y);

            // // Send trigger position + thumb X/Y to main network (ID 0x11).
            // // sendTriggerState() internally limits this to 100 Hz.
            // MainCanFD::sendTriggerState(
            //     getTriggerPositionNormalized(),
            //     thumbX,
            //     thumbY
            // );
            
            // Receive:
            // G,<stiffness>,<gripperPosition>
            MainSerial::update();

            // Read joystick
            // thumbX = analogRead(Pins::JOY_X);
            // thumbY = analogRead(Pins::JOY_Y);
            

            static uint32_t lastTriggerDebugMs = 0;

            // if (millis() - lastTriggerDebugMs >= 200)
            // {
            //     lastTriggerDebugMs = millis();

            //     Serial.printf(
            //         "TRIGGER DEBUG | valid=%d rawDeg=%.2f normalized=%u\n",
            //         feedback.valid,
            //         feedback.positionDeg,
            //         getTriggerPositionNormalized()
            //     );
            // }

            // Send:
            // T,<triggerPosition>,<thumbX>,<thumbY>
            MainSerial::sendTriggerState(
                getTriggerPositionNormalized(),
                thumbX,
                thumbY
            );

            // runPPControl();
            runMotor();

            #endif
    #endif
}
