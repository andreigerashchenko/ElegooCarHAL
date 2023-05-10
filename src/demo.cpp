#include "car_hal.h"  // Enable/Disable features at the top of this file

#if USE_IR
#include <IRremote.hpp>
#endif  // USE_IR

CAR_HAL car;

void setup() {
    Serial.begin(115200);
    car.MPU6050_Setup();
    car.Motor_Setup();
    car.PID_Setup(8.0, 0.0, 0.0, 180.0);
}

float x, y, z;

void loop() {
    car.MPU6050_GetGyro(&x, &y, &z);
    car.PID_Update();
    car.Motor_Control(0, 0);
}