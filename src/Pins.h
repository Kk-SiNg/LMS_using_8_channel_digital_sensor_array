/*
 * Pins.h
 * Hardware pin definitions - DIGITAL MODE
 * Target: ESP32 38-Pin DevKit (NodeMCU-32S)
 */

#pragma once
#include <cstdint>

// =========================================================
//  LEFT RAIL: SENSOR ARRAY
//  Physically, Pin 36 is at the TOP (near USB/Antenna)
//  Physically, Pin 26 is further DOWN.
// =========================================================

// Mapping assumes sensors are connected sequentially to the Left Rail
#define SENSOR_PIN_1  36  // Rightmost Sensor (Connected to TOP physical pin) -> INPUT ONLY
#define SENSOR_PIN_2  39  // INPUT ONLY
#define SENSOR_PIN_3  34  // INPUT ONLY
#define SENSOR_PIN_4  35  // INPUT ONLY
#define SENSOR_PIN_5  32  // (Bi-directional IO)
#define SENSOR_PIN_6  33  // (Bi-directional IO)
#define SENSOR_PIN_7  25  // (Bi-directional IO)
#define SENSOR_PIN_8  26  // Leftmost Sensor (Connected to BOTTOM physical pin) -> (Bi-directional IO)
const uint8_t SensorCount = 8;

// Digital mode settings
#define LINE_THRESHOLD 1              // Digital: 1 = line detected
#define MIN_DETECTION_RATIO 0.5
#define CALIBRATION_TIME_MS 1000

// =========================================================
//  RIGHT RAIL (USB Top): MOTORS & ENCODERS
//  Pins are listed in physical order (Top to Bottom)
//  Avoids GPIO 6-11 (Flash) and GPIO 1 (TX0)/3 (RX0)
// =========================================================

// === ENCODERS ===
// GPIO 23, 22, 21 are standard IO. 
// GPIO 5 is a strapping pin (must be HIGH during boot), but standard
// encoders usually leave this floating or high-Z enough to be safe.
#define ENCODER_R_A 23
#define ENCODER_R_B 22
#define ENCODER_L_A 21
#define ENCODER_L_B 5   // VSPI SS (Safe if not pulled LOW at boot)

// === MOTOR CONTROL (L298N) ===
// GPIO 19, 18, 17, 16 are standard.
// GPIO 4 is safe.
// GPIO 15 is a strapping pin (MTDO). MUST NOT be pulled HIGH at boot.
// L298N inputs are high-impedance, so this is generally safe.
#define MOTOR_R_ENB 19
#define MOTOR_R_IN4 18
#define MOTOR_R_IN3 17

#define MOTOR_L_ENA 16
#define MOTOR_L_IN1 15
#define MOTOR_L_IN2 4  // MTDO (Keep L298N powered or disconnected during upload if fails)

// === RGB LED ===
// Common Cathode (-) recommended to keep Pins 12/13 LOW during boot.
#define RGB_PIN_R  14  // Safe GPIO
#define RGB_PIN_G  12  // MTDI (Strapping: Must NOT be pulled HIGH at boot)
#define RGB_PIN_B  13  // Safe GPIO

// === USER INTERFACE ===
#define ONBOARD_LED 2   // Blue LED on DevKit
#define USER_BUTTON 0   // BOOT Button (Active LOW)

// === WIFI CONFIGURATION ===
#define SSID "Redmi A2"
#define PASSWORD "kvsandkks"
#define TELNET_PORT 23