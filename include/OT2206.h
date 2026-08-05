#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <string.h>

template <typename CANBus>
class OT2206CAN {
public:
    enum class ControlMode : uint8_t {
        Torque = 0,
        SpeedTorque = 1,
        PositionSpeedTorque = 2
    };

    // needs extended CAN for incremental!
    enum class AddressMode : uint8_t {
        ExtendedIncremental = 0,
        NormalAbsolute = 1
    };

    enum class PIParameter : uint8_t {
        SpeedKp = 1,
        SpeedKi = 2,
        CurrentKp = 3,
        CurrentKi = 4
    };

    struct Config {
        uint16_t motorId = 1;          // Motor receive CAN ID, configured over UART. 
        uint16_t hostId = 100;         // Motor response CAN ID, configured over UART. // The default host ID is 100
        float pmaxTurns = 1.0f;        // Must match Pmax stored in the motor.
        float speedFullScaleRadS = 200.0f; // MUST be set to the OT2206's actual full scale.
        float currentFullScaleA = 4.0f;    // MUST be set to the OT2206's actual full scale.
    };

    struct Feedback {
        bool valid = false;
        bool incrementalMode = false;
        uint8_t motorId = 0;
        uint8_t dlc = 0;
        uint32_t timestamp = 0;

        float positionDeg = 0.0f;
        float speedRadS = 0.0f;
        float currentA = 0.0f;

        int32_t positionCounts = 0; // Extended/incremental feedback only.
        uint16_t rawPosition16 = 0;
        uint16_t rawSpeed12 = 0;
        uint16_t rawCurrent12 = 0;
    };

    explicit OT2206CAN(CANBus &bus, const Config &config = Config())
        : bus_(bus), config_(config) {}

    void setConfig(const Config &config) { config_ = config; }
    const Config &config() const { return config_; }

    // General 8-byte command frame used by all three closed-loop modes.
    bool command(float positionDeg,
                 float speedRadS,
                 float currentA,
                 uint16_t positionKp,
                 uint16_t positionKd) {
        const uint16_t p = encodePosition(positionDeg);
        const uint16_t v = encodeSigned12(speedRadS, config_.speedFullScaleRadS);
        const uint16_t c = encodeSigned12(currentA, config_.currentFullScaleA);
        const uint16_t kp = clamp12(positionKp);
        const uint16_t kd = clamp12(positionKd);

        CAN_message_t msg{};
        msg.id = config_.motorId;
        msg.len = 8;
        msg.flags.extended = false;
        msg.flags.remote = false;

        msg.buf[0] = static_cast<uint8_t>(p >> 8);
        msg.buf[1] = static_cast<uint8_t>(p & 0xFF);
        msg.buf[2] = static_cast<uint8_t>(v >> 4);
        msg.buf[3] = static_cast<uint8_t>(((v & 0x0F) << 4) | ((kp >> 8) & 0x0F));
        msg.buf[4] = static_cast<uint8_t>(kp & 0xFF);
        msg.buf[5] = static_cast<uint8_t>(kd >> 4);
        msg.buf[6] = static_cast<uint8_t>(((kd & 0x0F) << 4) | ((c >> 8) & 0x0F));
        msg.buf[7] = static_cast<uint8_t>(c & 0xFF);

        const bool ok = write(msg);
        if (ok) hasCommand_ = true;
        return ok;
    }

    // Convenience commands. Send one of these before start().
    bool commandTorque(float currentA) {
        return command(0.0f, 0.0f, currentA, 0, 0);
    }

    bool commandSpeed(float speedRadS, float currentLimitA) {
        return command(0.0f, speedRadS, fabsf(currentLimitA), 0, 0);
    }

    bool commandPosition(float positionDeg,
                         float speedLimitRadS,
                         float currentLimitA,
                         uint16_t kp,
                         uint16_t kd) {
        return command(positionDeg, fabsf(speedLimitRadS), fabsf(currentLimitA), kp, kd);
    }

    bool selectTorqueMode() {
        currentMode_ = ControlMode::Torque;
        return sendSpecial(0xF9);
    }

    bool selectSpeedTorqueMode() {
        currentMode_ = ControlMode::SpeedTorque;
        return sendSpecial(0xFA);
    }

    bool selectPositionSpeedTorqueMode() {
        currentMode_ = ControlMode::PositionSpeedTorque;
        return sendSpecial(0xFB);
    }

    // Safety guard: the manual warns not to start before sending a setpoint.
    bool start(bool force = false) {
        if (!hasCommand_ && !force) return false;
        return sendSpecial(0xFC);
    }

    bool stopFree() {
        return sendSpecial(0xFD);
    }

    // Must be called while the motor is stopped/free. This zero is not retained after power-off.
    bool setTemporaryMechanicalZero() {
        return sendSpecial(0xFE);
    }

    // Switch the motor's classical-CAN control interpretation.
    // Uses a 29-bit/extended-ID CAN frame with the same numeric motor ID.
    // This must be repeated after every power cycle when incremental mode is used.
    bool setAddressMode(AddressMode mode) {
        CAN_message_t msg{};
        msg.id = config_.motorId;
        msg.len = 8;
        msg.flags.extended = true;
        msg.flags.remote = false;
        memset(msg.buf, 0, sizeof(msg.buf));
        msg.buf[0] = (mode == AddressMode::NormalAbsolute) ? 0x02 : 0x00;

        const bool ok = write(msg);
        if (ok) addressMode_ = mode;
        return ok;
    }

    // Sets a relative multiplier for an internal PI parameter. IEEE-754 float is
    // transmitted big-endian, as required by the manual. Not retained at power-off.
    bool setPIMultiplier(PIParameter parameter, float multiplier) {
        if (!isfinite(multiplier) || multiplier < 0.0f) return false;

        CAN_message_t msg{};
        msg.id = config_.motorId;
        msg.len = 8;
        msg.flags.extended = true;
        msg.flags.remote = false;
        memset(msg.buf, 0, sizeof(msg.buf));
        msg.buf[0] = 0x00;
        msg.buf[1] = static_cast<uint8_t>(parameter);

        uint32_t bits = 0;
        static_assert(sizeof(float) == sizeof(uint32_t), "32-bit float required");
        memcpy(&bits, &multiplier, sizeof(bits));
        msg.buf[2] = static_cast<uint8_t>((bits >> 24) & 0xFF);
        msg.buf[3] = static_cast<uint8_t>((bits >> 16) & 0xFF);
        msg.buf[4] = static_cast<uint8_t>((bits >> 8) & 0xFF);
        msg.buf[5] = static_cast<uint8_t>(bits & 0xFF);
        return write(msg);
    }

    // Raw classical-CAN frame access for future/vendor-specific functions.
    bool sendRaw(const uint8_t *data, uint8_t length = 8, bool extendedId = false) {
        if (data == nullptr || length > 8) return false;
        CAN_message_t msg{};
        msg.id = config_.motorId;
        msg.len = length;
        msg.flags.extended = extendedId;
        msg.flags.remote = false;
        memcpy(msg.buf, data, length);
        return write(msg);
    }

    // Call this for every received FlexCAN_T4 frame. Returns true only when the
    // frame is a valid response from this motor addressed to this host.
    bool parseFeedback(const CAN_message_t &msg, Feedback &out) const {
        out = Feedback{};
        if (msg.id != config_.hostId) return false;
        if (msg.flags.extended) return false;
        if (msg.len != 6 && msg.len != 8) return false;
        if (msg.buf[0] != static_cast<uint8_t>(config_.motorId)) return false;

        out.valid = true;
        out.incrementalMode = (msg.len == 8);
        out.motorId = msg.buf[0];
        out.dlc = msg.len;
        out.timestamp = msg.timestamp;

        out.rawSpeed12 = static_cast<uint16_t>((static_cast<uint16_t>(msg.buf[3]) << 4) |
                                               ((msg.buf[4] >> 4) & 0x0F));
        out.rawCurrent12 = static_cast<uint16_t>(((msg.buf[4] & 0x0F) << 8) |
                                                 msg.buf[5]);
        out.speedRadS = decodeSigned12(out.rawSpeed12, config_.speedFullScaleRadS);
        out.currentA = decodeSigned12(out.rawCurrent12, config_.currentFullScaleA);

        if (msg.len == 6) {
            out.rawPosition16 = static_cast<uint16_t>((static_cast<uint16_t>(msg.buf[1]) << 8) |
                                                      msg.buf[2]);
            out.positionDeg = decodePosition(out.rawPosition16);
        } else {
            const uint32_t raw = (static_cast<uint32_t>(msg.buf[6]) << 24) |
                                 (static_cast<uint32_t>(msg.buf[7]) << 16) |
                                 (static_cast<uint32_t>(msg.buf[1]) << 8) |
                                 static_cast<uint32_t>(msg.buf[2]);
            out.positionCounts = static_cast<int32_t>(raw);
            out.positionDeg = static_cast<float>(out.positionCounts) *
                              (360.0f / 1048576.0f);
            out.rawPosition16 = static_cast<uint16_t>((msg.buf[1] << 8) | msg.buf[2]);
        }
        return true;
    }

    AddressMode addressMode() const { return addressMode_; }
    ControlMode controlMode() const { return currentMode_; }
    bool hasCommand() const { return hasCommand_; }
    void clearCommandGuard() { hasCommand_ = false; }

private:
    CANBus &bus_;
    Config config_;
    AddressMode addressMode_ = AddressMode::NormalAbsolute;
    ControlMode currentMode_ = ControlMode::PositionSpeedTorque;
    bool hasCommand_ = false;

    bool write(CAN_message_t &msg) {
        return bus_.write(msg) != 0;
    }

    bool sendSpecial(uint8_t finalByte) {
        CAN_message_t msg{};
        msg.id = config_.motorId;
        msg.len = 8;
        msg.flags.extended = false;
        msg.flags.remote = false;
        memset(msg.buf, 0xFF, sizeof(msg.buf));
        msg.buf[7] = finalByte;
        return write(msg);
    }

    static uint16_t clamp12(uint16_t value) {
        return value > 0x0FFF ? 0x0FFF : value;
    }

    uint16_t encodePosition(float degrees) const {
        const float rangeDeg = 360.0f * config_.pmaxTurns;
        if (!(rangeDeg > 0.0f) || !isfinite(degrees)) return 0x8000;
        const float clamped = constrain(degrees, -rangeDeg, rangeDeg);
        long code = lroundf((clamped / rangeDeg) * 32768.0f + 32768.0f);
        if (code < 0) code = 0;
        if (code > 65535) code = 65535;
        return static_cast<uint16_t>(code);
    }

    float decodePosition(uint16_t code) const {
        return (static_cast<int32_t>(code) - 32768) / 32768.0f *
               (360.0f * config_.pmaxTurns);
    }

    static uint16_t encodeSigned12(float value, float fullScale) {
        if (!(fullScale > 0.0f) || !isfinite(value)) return 0x0800;
        const float clamped = constrain(value, -fullScale, fullScale);
        long code = lroundf((clamped / fullScale) * 2048.0f + 2048.0f);
        if (code < 0) code = 0;
        if (code > 4095) code = 4095;
        return static_cast<uint16_t>(code);
    }

    static float decodeSigned12(uint16_t code, float fullScale) {
        return (static_cast<int32_t>(code & 0x0FFF) - 2048) / 2048.0f * fullScale;
    }
};