#ifndef CAR_HAL_H
#define CAR_HAL_H

// Set enabled features here
#define USE_MOTORS 1
#define USE_MPU6050 1
#define USE_PID 1
#define USE_SERVOS 1
#define USE_ULTRASONIC 1
#define USE_BATTERY 1
#define USE_LED 1
#define USE_LINE_TRACKING 0
#define USE_IR 0

// Set IR protocol here
// #define DECODE_DENON        // Includes Sharp
// #define DECODE_JVC
// #define DECODE_KASEIKYO
// #define DECODE_PANASONIC    // alias for DECODE_KASEIKYO
// #define DECODE_LG
#define DECODE_NEC  // Includes Apple and Onkyo
// #define DECODE_SAMSUNG
// #define DECODE_SONY
// #define DECODE_RC5
// #define DECODE_RC6

// #define DECODE_BOSEWAVE
// #define DECODE_LEGO_PF
// #define DECODE_MAGIQUEST
// #define DECODE_WHYNTER

// #define DECODE_DISTANCE_WIDTH // Universal decoder for pulse distance width
// protocols #define DECODE_HASH         // special decoder for all protocols

// #define DECODE_BEO          // This protocol must always be enabled manually,
// i.e. it is NOT enabled if no protocol is defined. It prevents decoding of
// SONY!

// ==================== DO NOT EDIT BELOW THIS LINE ====================

#include <Arduino.h>
#include <stdint.h>

// ==================== PIN/HEADER DEFINITIONS ====================

// Motors
#if USE_MOTORS
#define PIN_Motor_A_PWM 5
#define PIN_Motor_B_PWM 6
#define PIN_Motor_A_IN1 7
#define PIN_Motor_B_IN1 8
#define PIN_Motor_STBY 3
#define MOTOR_MAX_PWM 255
#endif

// Servos
#if USE_SERVOS
#include <Servo.h>
#define PIN_Servo_1 10
#define PIN_Servo_2 11
#endif  // USE_SERVOS

// Ultrasonic sensor
#if USE_ULTRASONIC
#define PIN_US_Trig 13
#define PIN_US_Echo 12
// Minimum distance in cm. Sensor is rated for 2 cm to 400 cm.
#define US_Min_Dist 2
// Maximum distance in cm. Sensor is rated for 2 cm to 400 cm.
#define US_Max_Dist 400
#endif  // USE_ULTRASONIC

// Battery
#if USE_BATTERY
#define PIN_Battery A3  // Battery voltage pin
#define Battery_Input_Scale \
    0.0374714891  // (1 / (1.5k / (10k + 1.5k))) * (5 / 1023)
#endif            // USE_BATTERY

// USE_LED
#if USE_LED
#include <FastLED.h>
#define PIN_LED 4
#endif  // USE_LED

// USE_MPU6050
#if USE_MPU6050
#include <MPU6050_6Axis_MotionApps20.h>
#endif  // USE_MPU6050

// USE_LINE_TRACKING
#if USE_LINE_TRACKING
#define PIN_LineTracking_L A2
#define PIN_LineTracking_C A1
#define PIN_LineTracking_R A0
#endif  // USE_LINE_TRACKING

// USE_IR
#if USE_IR
#include "IRremoteButtons.h"
#define PIN_IR 9
#endif  // USE_IR

// ==================== CAR_HAL CLASS DEFINITION ====================

class CAR_HAL {
   private:
#if USE_MOTORS
    int16_t mmap(float in, float inMin, float inMax, int16_t outMin,
                 int16_t outMax);  // Map function for motor control
#endif

#if USE_SERVOS
    Servo Servo_1;  // Ultrasonic sensor servo
    Servo Servo_2;  // Unused servo
#endif              // USE_SERVOS

#if USE_LED
    CRGB leds[1];  // USE_LED
#endif             // USE_LED

#if USE_MPU6050
    MPU6050 mpu;  // MPU6050 object

    bool dmpReady = false;   // set true if DMP init was successful
    uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
    uint8_t fifoBuffer[64];  // FIFO storage buffer

    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorInt16 aa;       // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;  // [x, y, z]            gravity vector
    float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#endif                    // USE_MPU6050

#if USE_PID
    float PID_kP;  // PID constants
    float PID_kI;
    float PID_kD;
    float PID_iTerm;     // PID integral term
    float PID_setpoint;  // PID setpoint
    float PID_output;    // PID output
    float PID_lastError;
    uint32_t PID_lastTime;
#endif  // USE_PID

   public:
#if USE_MOTORS
    // Motor functions
    void Motor_Setup(void);
    void Motor_Control(int16_t left_PWM, int16_t right_PWM);
    void Motor_Drive(float speed, float turn);
#endif  // USE_MOTORS

#if USE_PID
#if !USE_MOTORS || !USE_MPU6050
#error "USE_PID requires USE_MOTORS and USE_MPU6050"
#endif  // !USE_MOTORS || !USE_MPU6050
    // PID functions
    void PID_Setup(float kP, float kI, float kD, float setpoint);
    void PID_Control(float setpoint);
    void PID_Update();
#endif  // USE_PID

#if USE_SERVOS
    // Servo functions
    void Servo_Setup(uint8_t id, bool reset);
    void Servo_Control(uint8_t id, int16_t pos);
    int16_t Servo_GetPos(uint8_t id);
    void Servo_Disable(uint8_t id);
#endif  // USE_SERVOS

#if USE_ULTRASONIC
    // Ultrasonic sensor functions
    void US_Setup(void);
    float US_GetDistance(void);
#endif  // USE_ULTRASONIC

#if USE_BATTERY
    // Battery functions
    void Battery_Setup(void);
    float Battery_GetVoltage(void);
#endif  // USE_BATTERY

#if USE_LED
    // USE_LED functions
    void LED_Setup(void);
    void LED_Control(uint8_t red, uint8_t green, uint8_t blue);
    void LED_Control(uint8_t red, uint8_t green, uint8_t blue, float brightness);
#endif  // USE_LED

#if USE_MPU6050
    // USE_MPU6050 functions
    void MPU6050_Setup();
    void MPU6050_Calibrate();
    void MPU6050_Update();
    void MPU6050_GetData(int16_t* ax, int16_t* ay, int16_t* az, float* gx,
                         float* gy, float* gz);
    void MPU6050_GetAccel(int16_t* ax, int16_t* ay, int16_t* az);
    void MPU6050_GetGyro(float* gx, float* gy, float* gz);
    float MPU6050_GetTemp();
#endif  // USE_MPU6050

#if USE_LINE_TRACKING
    // Line tracking functions
    void LineTracking_Setup(void);
    int16_t LineTracking_GetValue(uint8_t sensor);
#endif  // USE_LINE_TRACKING

#if USE_IR
    // IR functions
    void IR_Setup(void);
    bool IR_GetMessage(uint16_t* message);
#endif  // USE_IR
};
#endif  // CAR_HAL_H