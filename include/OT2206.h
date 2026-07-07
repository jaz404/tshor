#pragma once
#include <Arduino.h>

struct OT2206Status {
  float positionRad = 0.0f;
  float speedRadPerSec = 0.0f;
  float torqueNm = 0.0f;
  uint8_t temperature = 0;
  bool valid = false;

};

enum class Result : uint8_t {
    Success             = 0x00,
    Failure             = 0x01,
    UnknownCommand      = 0x02,
    UnknownID           = 0x03,
    ReadOnlyRegister    = 0x04,
    UnknownRegister     = 0x05,
    StringFormat        = 0x06,
    DataFormatError     = 0x07,
    WriteOnlyRegister   = 0x0B,
    NoResponse          = 0xFE,
    InvalidResponse     = 0xFF
};

class OT2206 {
public:
  void begin();
  Result start();
  Result setTorque(float torqueNm, uint32_t durationMs);
  OT2206Status readLastStatus() const;

private:
  HardwareSerial& serial = Serial2;
  OT2206Status lastStatus;
  float torque_const;
  int gear_ratio;

  void sendFrame(const uint8_t payload[8]);
  bool readFrame(uint8_t payload[8]);
  void parseControlResponse(const uint8_t payload[8]);

  static void writeFloatLE(uint8_t* dst, float value);
  static void writeUInt24LE(uint8_t* dst, uint32_t value);

  float readTorqueConstant();
  int readGearRatio();

};