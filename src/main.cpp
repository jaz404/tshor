#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "OT2206.h"
#include "adafruitmotor.h"
#include "BoardConfig.h"
#include "MainCanFD.h"
#include "MainSerial.h" // using serial for now instead of CAN to setup gripper

bool ppMotorStarted = false;

AdafruitMotor haptic;
constexpr uint32_t FEEDBACK_POLL_PERIOD_US = 5000; // 200 Hz
uint32_t lastFeedbackPollUs = 0;
    static uint32_t lastPrintMs = 0;

// =========================================================
// TRIGGER CONTROL PARAMETERS
// =========================================================

// Existing/local damping controller parameters.
constexpr float POSITION_GAIN_A_PER_DEG = 0.25f;
float targetPositionDeg = 0.0f;

constexpr uint32_t CONTROL_PERIOD_US = 500; // 1 kHz local motor-current loop
constexpr float DAMPING_GAIN = 2.00f;
constexpr float STATIC_RESISTANCE_A = 0.5f;
constexpr float MAX_DAMPING_CURRENT_A = 1.5f;
constexpr float VELOCITY_DEADBAND_RAD_S = 0.05f;

// P-P controller gain.
// Input error is normalized (0.0 to 1.0), output is motor current in A.
constexpr float PP_GAIN_A = 1.0f;
constexpr float PP_MAX_CURRENT_A = 2.0f;

// TODO: Measure and replace with the actual trigger travel.
// These limits are used only to convert OT2206 degrees into 0...10000.
constexpr float TRIGGER_OPEN_DEG = 250.0f;
constexpr float TRIGGER_CLOSED_DEG = -291.0f;

bool dampingEnabled = false;
bool ppControlEnabled = true;
uint32_t lastControlCommandUs = 0;
uint32_t lastDampingCommandUs = 0;

uint16_t thumbX = 0;
uint16_t thumbY = 0;

// =========================================================
// MOTOR NETWORK -- CLASSICAL CAN2
// =========================================================

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

OT2206CAN<decltype(Can1)>::Config motorConfig{
    .motorId = 1,
    .hostId = 100,
    .pmaxTurns = 1.0f,            // TODO: increase this in motor config if needed
    .speedFullScaleRadS = 200.0f,
    .currentFullScaleA = 4.0f
};

OT2206CAN<decltype(Can1)> motor(Can1, motorConfig);
OT2206CAN<decltype(Can1)>::Feedback feedback;

// =========================================================
// HELPERS
// =========================================================

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
constexpr float PP_CURRENT_LIMIT_A = 0.50f;

struct PPCommand
{
    float triggerPosition = 0.0f;
    float gripperPosition = 0.0f;
    float positionError = 0.0f;
    float targetPositionDeg = 0.0f;
};

static PPCommand calculatePPControl(
    uint16_t triggerPos,
    uint16_t gripperPos
)
{
    PPCommand command;

    command.triggerPosition =
        static_cast<float>(triggerPos) / 10000.0f;

    command.gripperPosition =
        static_cast<float>(gripperPos) / 10000.0f;

    // reference - measured
    command.positionError =
        command.gripperPosition -
        command.triggerPosition;

    // Actual gripper position becomes trigger reference.
    command.targetPositionDeg =
        TRIGGER_OPEN_DEG +
        command.gripperPosition *
        (TRIGGER_CLOSED_DEG - TRIGGER_OPEN_DEG);

    return command;
}

static void runPPControl()
{
    const uint32_t nowUs = micros();

    if (nowUs - lastControlCommandUs < CONTROL_PERIOD_US)
    {
        return;
    }

    lastControlCommandUs = nowUs;

    const MainSerial::GripperState& gripper =
        MainSerial::getGripperState();

    if (!gripper.valid)
    {
        return;
    }

    if (!ppMotorStarted)
{
    bool ok = true;

    ok &= motor.selectPositionSpeedTorqueMode();
    delay(5);

    // Load current trigger position first so it does not jump
    ok &= motor.commandPosition(
        feedback.positionDeg,
        PP_MAX_SPEED_RAD_S,
        0.2f,
        PP_KP,
        PP_KD
    );
    delay(5);

    ok &= motor.start();
    delay(5);

    Serial.printf(
        "P-P START: %s | initialPos=%.2f\n",
        ok ? "OK" : "FAIL",
        feedback.positionDeg
    );

    if (!ok)
    {
        return;
    }

    ppMotorStarted = true;
}

    const uint16_t triggerPos =
        getTriggerPositionNormalized();

    const PPCommand pp =
        calculatePPControl(
            triggerPos,
            gripper.position
        );

    const float stiffnessNormalized =
        constrain(
            static_cast<float>(gripper.stiffness) / 255.0f,
            0.0f,
            1.0f
        );

    const float currentLimit =
        stiffnessNormalized * PP_MAX_CURRENT_A;

    const bool ok = motor.commandPosition(
        pp.targetPositionDeg,       // reference = actual gripper
        PP_MAX_SPEED_RAD_S,
        currentLimit,
        PP_KP,
        PP_KD
    );

    static uint32_t lastPrintMs = 0;

    if (millis() - lastPrintMs >= 200)
    {
        lastPrintMs = millis();

        Serial.printf(
            "PP | trigger=%.3f gripper=%.3f "
            "error=%+.3f | target=%.2f deg "
            "Ilim=%.3fA Iactual=%.3fA TX=%s\n",
            pp.triggerPosition,
            pp.gripperPosition,
            pp.positionError,
            pp.targetPositionDeg,
            currentLimit,
            feedback.currentA,
            ok ? "OK" : "FAIL"
        );
    }
}

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
                // Avoid printing every frame during damping mode.
                if (!dampingEnabled)
                {
                    // printFeedback(feedback);
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

        // =====================================================
        // 2. MAIN NETWORK -- CAN3 FD
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
        // @G,<stiffness>,<gripperPosition>
        MainSerial::update();

        // Read joystick
        thumbX = analogRead(Pins::JOY_X);
        thumbY = analogRead(Pins::JOY_Y);

        // Send:
        // @T,<triggerPosition>,<thumbX>,<thumbY>
        MainSerial::sendTriggerState(
            getTriggerPositionNormalized(),
            thumbX,
            thumbY
        );

        // =====================================================
        // 3. LOCAL TRIGGER MOTOR CONTROL
        // =====================================================
        runPPControl();

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
                        10.0f,  // maximum rad/s
                        2.00f,  // maximum current A -- this is directly changing the feel of the motor 0.25 feels much lighter
                        100,      // Kp
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
                        0.20f,   // maximum current A -- this is directly changing the feel of the motor 0.25 feels much lighter
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
            }
            #endif
    #endif
}