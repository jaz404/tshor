#include "OT2206.h"
#include "BoardConfig.h"

static constexpr uint8_t CMD_START_MOTOR      = 0x91;
static constexpr uint8_t CMD_STOP_MOTOR       = 0x92;
static constexpr uint8_t CMD_TORQUE_CONTROL   = 0x93;
static constexpr uint8_t CMD_SPEED_CONTROL    = 0x94;
static constexpr uint8_t CMD_POSITION_CONTROL = 0x95;
// static constexpr uint8_t CMD_PTS_CONTROL      = 0x96;    Position/Torque/Speed control is not supported

static constexpr uint8_t CMD_STOP_CONTROL     = 0x97;
static constexpr uint8_t CMD_GET_FAULT        = 0xB2;
static constexpr uint8_t CMD_ACK_FAULT        = 0xB3;
static constexpr uint8_t CMD_GET_INDICATOR    = 0xB4;
static constexpr uint8_t CMD_RETRIEVE_CONF    = 0x84;         
static constexpr uint8_t CONF_TYPE_INT        = 0x00;
static constexpr uint8_t CONF_TYPE_FLOAT      = 0x01;  
static constexpr uint8_t CONF_TORQUE_CONSTANT_ID = 0x03;
static constexpr uint8_t CONF_GEAR_RATIO_ID      = 0x11;


void OT2206::begin() {
    serial.begin(Config::OT2206_UART_BAUD);
}

Result OT2206::start() {
    uint8_t frame[8] = {CMD_START_MOTOR, 0, 0, 0, 0, 0, 0, 0};
    sendFrame(frame);

    uint8_t response[8];
    if (!readFrame(response)) return Result::NoResponse;

    if (response[0] != CMD_START_MOTOR)
        return Result::InvalidResponse;

    Result result = static_cast<Result>(response[1]);

    if (result == Result::Success) {
        torque_const = readTorqueConstant();
        gear_ratio = readGearRatio();
    }

    return result;
}

Result OT2206::setTorque(float torqueNm, uint32_t durationMs) {
    uint8_t frame[8] = {0};

    frame[0] = CMD_TORQUE_CONTROL;

    writeFloatLE(&frame[1], torqueNm);
    writeUInt24LE(&frame[5], durationMs);
    
    sendFrame(frame);
    
    uint8_t response[8];
    if (!readFrame(response)) return Result::NoResponse;

    if (response[0] != CMD_TORQUE_CONTROL || response[1] != 0x00) {
    return Result::InvalidResponse;
    }

    parseControlResponse(response);
    return Result::Success;
}

void OT2206::parseControlResponse(const uint8_t payload[8]) {
    lastStatus.temperature = payload[2];

    uint16_t posRaw = payload[3] | (payload[4] << 8);
    uint16_t speedRaw = (payload[5] << 4) | (payload[6] >> 4);
    uint16_t torqueRaw = ((payload[6] & 0x0F) << 8) | payload[7];

    lastStatus.positionRad = posRaw * 25 / 65535 - 12.5;
    lastStatus.speedRadPerSec = speedRaw * 130 / 4095 - 65;
    lastStatus.torqueNm = (torqueRaw * (450 * torque_const * gear_ratio) / 4095) - 225 * torque_const * gear_ratio;
    lastStatus.valid = true;
}

float OT2206::readTorqueConstant()
{
    uint8_t frame[8] = {
        CMD_RETRIEVE_CONF,                      // Retrieve Configuration
        CONF_TYPE_FLOAT,                        // Float
        CONF_TORQUE_CONSTANT_ID,                // Torque Constant
        0,0,0,0,0
    };

    sendFrame(frame);

    uint8_t response[8];
    if (response[0] != CMD_RETRIEVE_CONF ||
        response[1] != CONF_TYPE_FLOAT ||
        response[2] != CONF_TORQUE_CONSTANT_ID ||
        response[3] != 0x00)
    return false;

    float torqueConstant;
    memcpy(&torqueConstant, &response[4], sizeof(float));

    return torqueConstant;
}

int OT2206::readGearRatio()
{
    uint8_t frame[8] = {
        CMD_RETRIEVE_CONF,                      // Retrieve Configuration
        CONF_TYPE_INT,                          // int
        CONF_GEAR_RATIO_ID,                     // gear ratio Constant
        0,0,0,0,0
    };

    sendFrame(frame);

    uint8_t response[8];
    if (response[0] != CMD_RETRIEVE_CONF ||
        response[1] != CONF_TYPE_INT ||
        response[2] != CONF_GEAR_RATIO_ID ||
        response[3] != 0x00)
    return false;

    int gearRatio;
    memcpy(&gearRatio, &response[4], sizeof(float));

    return gearRatio;
}

void OT2206::writeFloatLE(uint8_t* dst, float value) { 
   // byte 1 .... 4
  static_assert(sizeof(float) == 4, "Expected 32-bit float");
  memcpy(dst, &value, 4);
}

void OT2206::writeUInt24LE(uint8_t* dst, uint32_t value) {
    dst[0] = value & 0xFF;
    dst[1] = (value >> 8) & 0xFF;
    dst[2] = (value >> 16) & 0xFF;
}

void OT2206::sendFrame(const uint8_t payload[8]) {
    // may need header?
  serial.write(payload, 8);
}

bool OT2206::readFrame(uint8_t payload[8]) {
    // may need to remove header
  const uint32_t timeoutMs = 5;
  uint32_t start = millis();

  size_t count = 0;
  while (count < 8 && millis() - start < timeoutMs) {
    if (serial.available()) {
      payload[count++] = serial.read();
    }
  }

  return count == 8;
}

