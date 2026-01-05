#pragma once
#include "pico/stdlib.h"

// Pin Definitions
constexpr uint PIN_UART_LEFT_TX = 0;
constexpr uint PIN_UART_RIGHT_TX = 8;

constexpr uint PIN_STEP_LEFT = 3;
constexpr uint PIN_DIR_LEFT = 29;

constexpr uint PIN_STEP_RIGHT = 2;
constexpr uint PIN_DIR_RIGHT = 28;

constexpr uint PIN_LED = 16;
constexpr uint PIN_BUTTON = 7;

// Motor Configuration
constexpr uint16_t MICROSTEPS = 16;
constexpr uint16_t CURRENT_MA = 800;

// Calibration
constexpr bool REVERSE_LEFT = true;
constexpr bool REVERSE_RIGHT = false;
constexpr float STEPS_PER_REV = 1600.0f; // 200 * 16
constexpr float MM_PER_REV = 188.5f;     // 60mm diameter * pi
constexpr float STEPS_PER_90 = 639.5f;

// Acceleration Profile
constexpr float MAX_SPEED_MM_PER_SEC = 500.0f;
constexpr float MIN_SPEED_MM_PER_SEC = 50.0f;
constexpr float ACCEL_MM_PER_SEC2 = 750.0f;