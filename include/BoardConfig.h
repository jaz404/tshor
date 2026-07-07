#pragma once
#include <Arduino.h>

namespace Pins {
  static constexpr uint8_t JOY_X = 15;
  static constexpr uint8_t JOY_Y = 14;

  static constexpr uint8_t I2C_SDA = 16;
  static constexpr uint8_t I2C_SCL = 17;

  static constexpr uint8_t CAN_RX = 0;
  static constexpr uint8_t CAN_TX = 1;

  static constexpr uint8_t LED_1 = 19;
  static constexpr uint8_t LED_2 = 20;
  static constexpr uint8_t LED_3 = 21;
}

namespace Config {
  static constexpr uint32_t CAN_BAUD = 1000000;
  static constexpr uint32_t OT2206_UART_BAUD = 921600;

  static constexpr float HANDLE_MIN_RAD = -1.0f;
  static constexpr float HANDLE_MAX_RAD =  1.0f;

  static constexpr float GRIPPER_MIN = 0.0f;
  static constexpr float GRIPPER_MAX = 1.0f;

  static constexpr float MAX_FEEDBACK_TORQUE = 0.4f;
}