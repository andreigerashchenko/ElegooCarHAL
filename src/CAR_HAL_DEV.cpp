#include "car_hal.h"  // Enable/Disable features at the top of this file

#if USE_IR
#include <IRremote.hpp>
#endif  // USE_IR

#define MESSAGE_TIMEOUT 120  // ms

CAR_HAL hal;

void setup() {
  Serial.begin(115200);
  hal.Motor_Setup();
  hal.IR_Setup();
  hal.Servo_Setup(1, true);
}

// uint16_t command;
// // 0 - up, 1 - down, 2 - left, 3 - right
// uint32_t command_time[4];
// float speed = 0.0;
// float turn = 0.0;
// bool message_received = false;

// void loop() {
//   if (hal.IR_GetMessage(&command)) {
//     if (!message_received) {
//       message_received = true;
//     }
//     if (command == IR_BUTTON_UP) {
//       command_time[0] = millis();
//     } else if (command == IR_BUTTON_DOWN) {
//       command_time[1] = millis();
//     } else if (command == IR_BUTTON_LEFT) {
//       command_time[2] = millis();
//     } else if (command == IR_BUTTON_RIGHT) {
//       command_time[3] = millis();
//     }
//   }

//   speed = 0.0;
//   turn = 0.0;

//   if (millis() - command_time[0] < MESSAGE_TIMEOUT) {
//     speed = 1.0;
//   } else if (millis() - command_time[1] < MESSAGE_TIMEOUT) {
//     speed = -1.0;
//   }

//   if (millis() - command_time[2] < MESSAGE_TIMEOUT) {
//     turn = -1.0;
//   } else if (millis() - command_time[3] < MESSAGE_TIMEOUT) {
//     turn = 1.0;
//   }

//   hal.Motor_Drive(speed, turn);
// }

#define ARG_MAX 0.2 // 0.0 - 1.0
#define STEP_DELAY 1000 // ms

void loop() {
  // forward
  hal.Motor_Drive(ARG_MAX, 0.0);
  delay(STEP_DELAY);
  // backward
  hal.Motor_Drive(-ARG_MAX, 0.0);
  delay(STEP_DELAY);
  // left
  hal.Motor_Drive(0.0, -ARG_MAX);
  delay(STEP_DELAY);
  // right
  hal.Motor_Drive(0.0, ARG_MAX);
  delay(STEP_DELAY);
  // forward left
  hal.Motor_Drive(ARG_MAX, -ARG_MAX);
  delay(STEP_DELAY);
  // back left
  hal.Motor_Drive(-ARG_MAX, -ARG_MAX);
  delay(STEP_DELAY);
  // forward right
  hal.Motor_Drive(ARG_MAX, ARG_MAX);
  delay(STEP_DELAY);
  // back right
  hal.Motor_Drive(-ARG_MAX, ARG_MAX);
  delay(STEP_DELAY);

  // stop
  hal.Motor_Drive(0.0, 0.0);
  Serial.println("==================================");
  delay(STEP_DELAY * 3);

  // repeat
}