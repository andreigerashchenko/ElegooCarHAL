#include "car_hal.h"
#if USE_IR
#include <IRremote.hpp>
#endif  // USE_IR

CAR_HAL car;

#define LED_BRIGHTNESS 0.01       // 0-1
#define MAX_SPEED_THRESHOLD 60.0  // cm, distance at which to start slowing down
#define DIST_THRESHOLD 30.0       // cm
#define SERVO_CENTER 90           // degrees
#define SERVO_STEP 70             // degrees
#define SERVO_DELAY 500           // ms, time to wait for servo to move
#define MAX_DRIVE_SPEED 255       // 0-255
#define MIN_DRIVE_SPEED 80        // 0-255
#define TURN_TIME 500             // ms, time to turn while stopped
#define STUCK_TIME 5000           // ms, max time to turn while stopped
#define TURN_TOLERANCE 20.0       // degrees, tolerance for turning in place
#define SERVO_LOOK_RANGE 20.0     // degrees, range to look for obstacles
#define SERVO_TURN_INTERVAL 25    // ms, time between servo turns
#define US_MAX_ATTEMPTS 10        // max number of attempts to get US reading

float us_distance = 0.0;
float dist_left = 0.0;
float dist_right = 0.0;

float angleSetpoint = 0.0;

float servoAngle = SERVO_CENTER;
bool servoDirection = true;  // true = right, false = left

float gx, gy, gz;

uint16_t driveSpeed = 100;
uint8_t color = 0;

uint32_t servoTime = 0;
uint32_t turnTime = 0;
uint32_t stuckTime = 0;
bool turned = false;

typedef enum state_t {
    FORWARD,
    START_LEFT,
    WAIT_LEFT,
    READ_LEFT,
    START_RIGHT,
    WAIT_RIGHT,
    READ_RIGHT,
    DECISION,
    WAIT_RESET,
} state_t;

state_t state = FORWARD;

void setup() {
    Serial.begin(115200);
    
    car.Motor_Setup();

    car.Battery_Setup();

    Serial.print(F("Battery voltage: "));
    Serial.print(car.Battery_GetVoltage());
    Serial.println(F("V"));
    if (car.Battery_GetVoltage() < 6.5) {
        Serial.println(F("Battery voltage too low, stopping"));
        car.Motor_Control(0, 0);
        while (true) {
            car.LED_Control(255, 0, 0, LED_BRIGHTNESS);
            delay(100);
            car.LED_Control(0, 0, 0, LED_BRIGHTNESS);
            delay(100);
        }
    }


    car.US_Setup();

    car.Servo_Setup(1, true);

    car.LED_Setup();
    // Yellow LED while calibrating
    car.LED_Control(255, 255, 0, LED_BRIGHTNESS);

    car.MPU6050_Setup();

    car.PID_Setup(7.0, 0.0, 0.2, 0.0);

    // Green LED when ready
    car.LED_Control(0, 255, 0, LED_BRIGHTNESS);
    delay(1000);
}

/**
 * @brief Get distance from ultrasonic sensor, retaking measurement if invalid
 * @return Distance in cm
 */
float validDistance(CAR_HAL* car) {
    float distance = car->US_GetDistance();
    uint16_t attempts = 0;
    while ((distance < US_Min_Dist || distance > US_Max_Dist) && (attempts < US_MAX_ATTEMPTS)) {
        attempts++;
        distance = car->US_GetDistance();
    }
    if (attempts >= US_MAX_ATTEMPTS) {
        distance = US_Min_Dist;
    }
    return distance;
}

/**
 * @brief Clamp angle to range [-180.0, 180.0], wrapping around if necessary
 * @return Clamped angle
 */
float clampAngle(float angle) {
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}

// State machine:
// 1. If in FORWARD state, read distance from ultrasonic sensor
// 2. If distance is less than DIST_THRESHOLD, stop the car and switch to START_LEFT state
// 3. If in START_LEFT state turn servo to the left
// 4. If in WAIT_LEFT state, wait for SERVO_DELAY ms, then switch to READ_LEFT state
// 5. If in READ_LEFT state, read distance from ultrasonic sensor, then switch to WAIT_RIGHT state
// 6. If in START_RIGHT state, turn servo to the right
// 7. If in WAIT_RIGHT state, turn servo to the right, then wait for 2 * SERVO_DELAY ms, then switch to READ_RIGHT state
// 8. If in READ_RIGHT state, read distance from ultrasonic sensor, then switch to DECISION state
// 9. If in DECISION state, decide best direction to turn towards, then update the setpoint angle and switch to WAIT_RESET state and center the servo
// 10. If in WAIT_RESET state, wait for at least SERVO_DELAY ms, wait for the robot to reach its target angle for at least one second, then switch to FORWARD state
// At the end of every loop, update the PID controller and motor speed to keep the control loop running at a constant rate

void loop() {
    // Forward state
    if (state == FORWARD) {
        // Read distance from ultrasonic sensor
        us_distance = validDistance(&car);
        // Slightly move servo to expand field of view
        if (servoDirection) {
            if (servoAngle < SERVO_CENTER + SERVO_LOOK_RANGE) {
                servoAngle += SERVO_STEP;
            } else {
                servoDirection = false;
            }
        } else {
            if (servoAngle > SERVO_CENTER - SERVO_LOOK_RANGE) {
                servoAngle -= SERVO_STEP;
            } else {
                servoDirection = true;
            }
        }
        if (millis() - servoTime > SERVO_TURN_INTERVAL) {
            servoTime = millis();
            car.Servo_Control(1, servoAngle);
        }

        // If distance is less than DIST_THRESHOLD, switch to WAIT_LEFT state, otherwise go forward
        if (us_distance < DIST_THRESHOLD) {
            driveSpeed = 0;
            state = START_LEFT;
        } else {
            if (us_distance >= MAX_SPEED_THRESHOLD) {
                driveSpeed = MAX_DRIVE_SPEED;
            } else {
                driveSpeed = map(us_distance, DIST_THRESHOLD, MAX_SPEED_THRESHOLD, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
                color = map(us_distance, DIST_THRESHOLD, US_Max_Dist, 0, 255);
                car.LED_Control(255 - color, color, 0, LED_BRIGHTNESS);
            }
        }
    }

    // Start left state
    if (state == START_LEFT) {
        // Turn servo to the left
        car.Servo_Control(1, SERVO_CENTER + SERVO_STEP);
        // Record current time
        servoTime = millis();
        // Switch to WAIT_LEFT state
        state = WAIT_LEFT;
        // Turn LED off
        car.LED_Control(0, 0, 0, LED_BRIGHTNESS);
    }

    // Wait left state
    if (state == WAIT_LEFT) {
        // Wait for SERVO_DELAY ms
        if (millis() - servoTime > SERVO_DELAY) {
            // Switch to READ_LEFT state
            state = READ_LEFT;
        }
    }

    // Read left state
    if (state == READ_LEFT) {
        // Read distance from ultrasonic sensor
        dist_left = validDistance(&car);
        // Switch to START_RIGHT state
        state = START_RIGHT;
        // Make LED white
        car.LED_Control(255, 255, 255, LED_BRIGHTNESS);
    }

    // Start right state
    if (state == START_RIGHT) {
        // Turn servo to the right
        car.Servo_Control(1, SERVO_CENTER - SERVO_STEP);
        // Record current time
        servoTime = millis();
        // Switch to WAIT_RIGHT state
        state = WAIT_RIGHT;
        // Turn LED off
        car.LED_Control(0, 0, 0, LED_BRIGHTNESS);
    }

    // Wait right state
    if (state == WAIT_RIGHT) {
        // Wait for 2 * SERVO_DELAY ms
        if (millis() - servoTime > 2 * SERVO_DELAY) {
            // Switch to READ_RIGHT state
            state = READ_RIGHT;
        }
    }

    // Read right state
    if (state == READ_RIGHT) {
        // Read distance from ultrasonic sensor
        dist_right = validDistance(&car);
        // Switch to WAIT_FORWARD state
        state = DECISION;
        // Make LED white
        car.LED_Control(255, 255, 255, LED_BRIGHTNESS);
    }

    // Decision state
    if (state == DECISION) {
        // No room left or right, turn around
        if (dist_left <= DIST_THRESHOLD && dist_right <= DIST_THRESHOLD) {
            angleSetpoint = clampAngle(angleSetpoint + 180.0);
        }

        // At least one side has space, go towards the bigger one
        if (dist_left > dist_right) {
            angleSetpoint = clampAngle(angleSetpoint - 90.0);
        } else {
            angleSetpoint = clampAngle(angleSetpoint + 90.0);
        }

        // Switch to WAIT_RESET state
        state = WAIT_RESET;
        // Reset turned flag
        turned = false;
        // Center the servo
        servoAngle = SERVO_CENTER;
        car.Servo_Control(1, servoAngle);
        // Record current time
        servoTime = millis();
        // Make LED blue
        car.LED_Control(0, 0, 255, LED_BRIGHTNESS);
    }

    // Wait reset state
    if (state == WAIT_RESET) {
        // Read Z-axis gyroscope value
        car.MPU6050_GetGyro(&gx, &gy, &gz);

        // If the robot has just reached its target angle, start the timer
        if (!turned && abs(gz - angleSetpoint) < TURN_TOLERANCE) {
            turnTime = millis();
            turned = true;
            // Make LED yellow
            car.LED_Control(255, 255, 0, LED_BRIGHTNESS);
        }
        // If the robot has not reached its target angle, reset the timer
        else if (abs(gz - angleSetpoint) >= TURN_TOLERANCE) {
            turnTime = millis();
            if (turned) {
                stuckTime = millis();
            }
            turned = false;
            // Make LED red
            car.LED_Control(255, 0, 0, LED_BRIGHTNESS);
        }

        // Wait for SERVO_DELAY ms and for the robot to reach its target angle for at least one second
        if (millis() - servoTime >= SERVO_DELAY && ((turned && millis() - turnTime >= TURN_TIME) || millis() - stuckTime >= STUCK_TIME)) {
            // Switch to FORWARD state
            state = FORWARD;
        }
    }

    car.PID_Control(angleSetpoint);
    car.PID_Update();
    car.Motor_Control(driveSpeed, driveSpeed);
}