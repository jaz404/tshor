#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "OT2206.h"
#include "adafruitmotor.h"
#include "BoardConfig.h"
#include "MainCanFD.h"

AdafruitMotor haptic;

// =========================================================
// TRIGGER CONTROL PARAMETERS
// =========================================================

// Existing/local damping controller parameters.
constexpr float POSITION_GAIN_A_PER_DEG = 0.25f;
float targetPositionDeg = 0.0f;

constexpr uint32_t CONTROL_PERIOD_US = 1000; // 1 kHz local motor-current loop
constexpr float DAMPING_GAIN = 2.00f;
constexpr float STATIC_RESISTANCE_A = 0.5f;
constexpr float MAX_DAMPING_CURRENT_A = 1.5f;
constexpr float VELOCITY_DEADBAND_RAD_S = 0.05f;

// P-P controller gain.
// Input error is normalized (0.0 to 1.0), output is motor current in A.
constexpr float PP_GAIN_A = 1.0f;
constexpr float PP_MAX_CURRENT_A = 1.5f;

// TODO: Measure and replace with the actual trigger travel.
// These limits are used only to convert OT2206 degrees into 0...10000.
constexpr float TRIGGER_OPEN_DEG = 0.0f;
constexpr float TRIGGER_CLOSED_DEG = 30.0f;

bool dampingEnabled = false;
bool ppControlEnabled = false;
uint32_t lastControlCommandUs = 0;

// TODO: Replace/update with analog read 14 and 15
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
constexpr float PP_MAX_SPEED_RAD_S = 10.0f;
constexpr float PP_KP = 1.0f;
constexpr float PP_KD = 0.0f;

// Current limit used by OT2206 position mode for the first P-P version.
// Later this can be changed using gripper force/contact/error to make
// the trigger feel stiffer when the gripper interacts with an object.
constexpr float PP_CURRENT_LIMIT_A = 0.25f;

struct PPCommand
{
    float targetPositionDeg = 0.0f;
    float currentLimitA = PP_CURRENT_LIMIT_A;
    float positionError = 0.0f;
};

static PPCommand calculatePPControl(
    uint16_t triggerPos,
    uint16_t gripperPos
)
{
    PPCommand command;

    const float triggerNormalized =
        static_cast<float>(triggerPos) / 10000.0f;

    const float gripperNormalized =
        static_cast<float>(gripperPos) / 10000.0f;

    // Positive error means the trigger has been squeezed farther closed
    // than the actual gripper has moved.
    command.positionError =
        triggerNormalized - gripperNormalized;

    // Map actual gripper position back into the trigger's mechanical range.
    // This is the key P-P coupling: the trigger motor is commanded toward
    // the position corresponding to the ACTUAL gripper position.
    command.targetPositionDeg =
        TRIGGER_OPEN_DEG +
        gripperNormalized * (TRIGGER_CLOSED_DEG - TRIGGER_OPEN_DEG);

    // For now use a fixed current limit. The OT2206 position controller
    // creates the restoring action from position error. Later we can make
    // this current limit depend on gripper force/contact to vary stiffness.
    command.currentLimitA = PP_CURRENT_LIMIT_A;

    return command;
}

static void runPPControl()
{
    if (!ppControlEnabled)
    {
        return;
    }

    const uint32_t nowUs = micros();

    if (nowUs - lastControlCommandUs < CONTROL_PERIOD_US)
    {
        return;
    }

    lastControlCommandUs += CONTROL_PERIOD_US;

    if (!feedback.valid)
    {
        return;
    }

    const MainCanFD::GripperState& gripper =
        MainCanFD::getGripperState();

    // Do not command a P-P target from stale/missing gripper state.
    if (!gripper.valid)
    {
        return;
    }

    const uint16_t triggerPos =
        getTriggerPositionNormalized();

    const PPCommand pp =
        calculatePPControl(triggerPos, gripper.position);

    // IMPORTANT:
    // OT2206 is still commanded over classical CAN2.
    // We use position/speed/torque mode here, NOT direct torque mode.
    //
    // target position = actual gripper position mapped to trigger degrees
    // max speed       = PP_MAX_SPEED_RAD_S
    // current limit   = fixed for now; later driven by force/contact
    // Kp / Kd         = OT2206 position-control gains
    const bool ok = motor.commandPosition(
        pp.targetPositionDeg,
        PP_MAX_SPEED_RAD_S,
        pp.currentLimitA,
        PP_KP,
        PP_KD
    );

    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs >= 200)
    {
        lastPrintMs = millis();

        const float triggerNorm =
            static_cast<float>(triggerPos) / 10000.0f;

        const float gripperNorm =
            static_cast<float>(gripper.position) / 10000.0f;

        Serial.printf(
            "PP: trigger=%.3f gripper=%.3f err=%+.3f target=%+.2f deg Imax=%.3f A measured=%+.3f A TX=%s\n",
            triggerNorm,
            gripperNorm,
            pp.positionError,
            pp.targetPositionDeg,
            pp.currentLimitA,
            feedback.currentA,
            ok ? "OK" : "FAILED"
        );
    }
}

    // Start OT2206 in position/speed/torque mode for bilateral P-P control.
    static bool startPPPositionControl()
    {
        dampingEnabled = false;
        ppControlEnabled = false;

        bool ok = motor.stopFree();
        delay(10);

        ok &= motor.selectPositionSpeedTorqueMode();
        delay(10);

        // Load a safe initial position command before starting.
        // Use the current trigger position so enabling P-P does not cause a jump.
        const float initialPositionDeg =
            feedback.valid ? feedback.positionDeg : targetPositionDeg;

        ok &= motor.commandPosition(
            initialPositionDeg,
            PP_MAX_SPEED_RAD_S,
            PP_CURRENT_LIMIT_A,
            PP_KP,
            PP_KD
        );
        delay(10);

        ok &= motor.start();
        delay(10);

        if (ok)
        {
            lastControlCommandUs = micros();
        }
        else
        {
            motor.stopFree();
        }

        return ok;
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

        // =====================================================
        // 2. MAIN NETWORK -- CAN3 FD
        // =====================================================

        // Read latest gripper position (ID 0x12) and update timeout state.
        MainCanFD::update();

        // Send trigger position + thumb X/Y to main network (ID 0x11).
        // sendTriggerState() internally limits this to 100 Hz.
        MainCanFD::sendTriggerState(
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
                        0.25f,  // maximum current A -- this is directly changing the feel of the motor 0.25 feels much lighter
                        1,      // Kp
                        0       // Kd
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