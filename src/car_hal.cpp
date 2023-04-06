#include "car_hal.h"

// Motor functions
#if USE_MOTORS
/**
 * @brief Float to int16_t map function
 */
int16_t mmap(float in, float inMin, float inMax, int16_t outMin, int16_t outMax) {
  return (int16_t)(in - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

/**
 * @brief Configures motor pins as outputs
 */
void CAR_HAL::Motor_Setup() {
  pinMode(PIN_Motor_A_PWM, OUTPUT);
  pinMode(PIN_Motor_A_IN1, OUTPUT);
  // pinMode(PIN_Motor_A_IN2, OUTPUT);
  // SN74LVC2G14 inverts IN1, so IN2 is always ~IN1
  pinMode(PIN_Motor_B_PWM, OUTPUT);
  pinMode(PIN_Motor_B_IN1, OUTPUT);
  // pinMode(PIN_Motor_B_IN2, OUTPUT);
  // SN74LVC2G14 inverts IN1, so IN2 is always ~IN1
  pinMode(PIN_Motor_STBY, OUTPUT);

  // Motors are initially not powered
  digitalWrite(PIN_Motor_STBY, LOW);
}

/**
 * @brief Controls the motors
 * @param left PWM value for left motor (-255 to 255)
 * @param en_left Enable left motor
 * @param right PWM value for right motor (-255 to 255)
 * @param en_right Enable right motor
 */
void CAR_HAL::Motor_Control(int16_t left, int16_t right) {
  digitalWrite(PIN_Motor_STBY, HIGH);  // Enable motor driver

  if (left > 0) {
    digitalWrite(PIN_Motor_A_IN1, HIGH);
    analogWrite(PIN_Motor_A_PWM, left);
  } else {
    digitalWrite(PIN_Motor_A_IN1, LOW);
    analogWrite(PIN_Motor_A_PWM, -left);
  }

  if (right > 0) {
    digitalWrite(PIN_Motor_B_IN1, HIGH);
    analogWrite(PIN_Motor_B_PWM, right);
  } else {
    digitalWrite(PIN_Motor_B_IN1, LOW);
    analogWrite(PIN_Motor_B_PWM, -right);
  }
}

/**
 * @brief Drives the car, mixing speed and turn
 * @param speed Speed of the car (-1 to 1)
 * @param turn Turn of the car (-1 to 1)
 */
void CAR_HAL::Motor_Drive(float speed, float turn) {
  int16_t left, right, speedFwd, speedTurn;

  if (speed > 0) {
    speedFwd = map(speed, 0, 1, 0, MOTOR_MAX_PWM);
  } else if (speed < 0) {
    speedFwd = map(speed, 0, -1, 0, -MOTOR_MAX_PWM);
  } else {
    speedFwd = 0;
  }

  if (turn > 0) {
    speedTurn = map(turn, 0, 1, 0, MOTOR_MAX_PWM);
  } else if (turn < 0) {
    speedTurn = map(turn, 0, -1, 0, -MOTOR_MAX_PWM);
  } else {
    speedTurn = 0;
  }

  left = speedFwd + speedTurn;
  right = speedFwd - speedTurn;

  if (left > MOTOR_MAX_PWM) {
    left = MOTOR_MAX_PWM;
  } else if (left < -MOTOR_MAX_PWM) {
    left = -MOTOR_MAX_PWM;
  }

  if (right > MOTOR_MAX_PWM) {
    right = MOTOR_MAX_PWM;
  } else if (right < -MOTOR_MAX_PWM) {
    right = -MOTOR_MAX_PWM;
  }

  Motor_Control(left, right);
}

#endif  // USE_MOTORS

// Servo functions
#if USE_SERVOS
/**
 * @brief Configures servo pins as outputs
 * @param id Servo ID (1 or 2)
 * @param reset Reset servo to 90 degrees on initalization
 */
void CAR_HAL::Servo_Setup(uint8_t id, bool reset) {
  if (id == 1) {
    Servo_1.attach(PIN_Servo_1);
    if (reset) {
      Servo_1.write(90);
    }
  } else if (id == 2) {
    Servo_2.attach(PIN_Servo_2);
    if (reset) {
      Servo_2.write(90);
    }
  }
}

/**
 * @brief Controls the servo
 * @param id Servo ID (1 or 2)
 * @param pos Servo position in degrees (0 to 180)
 */
void CAR_HAL::Servo_Control(uint8_t id, int16_t pos) {
  if (id == 1) {
    Servo_1.write(pos);
  } else if (id == 2) {
    Servo_2.write(pos);
  }
}

/**
 * @brief Gets the servo position
 * @param id Servo ID (1 or 2)
 * @return Servo position in degrees (0 to 180), -1 if invalid ID
 */
int16_t CAR_HAL::Servo_GetPos(uint8_t id) {
  if (id == 1) {
    return Servo_1.read();
  } else if (id == 2) {
    return Servo_2.read();
  }
  return -1;
}

/**
 * @brief Disables the servo
 * @param id Servo ID (1 or 2)
 */
void CAR_HAL::Servo_Disable(uint8_t id) {
  if (id == 1) {
    Servo_1.detach();
  } else if (id == 2) {
    Servo_2.detach();
  }
}
#endif  // USE_SERVOS

// Ultrasonic sensor functions
#if USE_ULTRASONIC
/**
 * @brief Configures ultrasonic sensor pins as outputs
 */
void CAR_HAL::US_Setup() {
  pinMode(PIN_US_Trig, OUTPUT);
  pinMode(PIN_US_Echo, INPUT);
}

/**
 * @brief Gets the distance from the ultrasonic sensor
 * @return Distance in cm
 */
float CAR_HAL::US_GetDistance() {
  // The following code is based on the example code from the HC-SR04 datasheet
  // and the code from the Arduino Playground
  // https://www.arduino.cc/en/Tutorial/UltrasonicSensor
  // https://playground.arduino.cc/Main/UltrasonicSensor
  // https://www.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf

  // Send a 10us pulse to the trigger pin
  digitalWrite(PIN_US_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_Trig, LOW);

  // Read the echo pin
  float duration = pulseIn(PIN_US_Echo, HIGH);

  // Calculate the distance
  float distance = duration * 0.0171821306;

  // Check if the distance is within the range of the sensor
  if (distance < US_Min_Dist || distance > US_Max_Dist) {
    distance = -1;
  }

  return distance;
}
#endif  // USE_ULTRASONIC

// Battery functions
#if USE_BATTERY
/**
 * @brief Configures battery pin as an input
 */
void CAR_HAL::Battery_Setup() { pinMode(PIN_Battery, INPUT); }

/**
 * @brief Gets the battery voltage
 * @return Battery voltage in volts
 */
float CAR_HAL::Battery_GetVoltage() {
  return (analogRead(PIN_Battery) * Battery_Input_Scale);
}
#endif  // USE_BATTERY

// USE_LED Functions
#if USE_LED
/**
 * @brief Configures USE_LED pins as outputs and initializes USE_LED class
 */
void CAR_HAL::LED_Setup() {
  pinMode(PIN_LED, OUTPUT);
  FastLED.addLeds<WS2812B, PIN_LED, GRB>(leds, 1);
}

/**
 * @brief Controls the onboard RGB LED
 * @param red Red value (0 to 255)
 * @param green Green value (0 to 255)
 * @param blue Blue value (0 to 255)
 */
void CAR_HAL::LED_Control(uint8_t red, uint8_t green, uint8_t blue) {
  leds[0] = CRGB(red, green, blue);
  FastLED.show();
}

/**
 * @brief Controls the onboard RGB LED
 * @param red Red value (0 to 255)
 * @param green Green value (0 to 255)
 * @param blue Blue value (0 to 255)
 * @param brightness Brightness value (0 to 1)
 */
void CAR_HAL::LED_Control(uint8_t red, uint8_t green, uint8_t blue,
                          float brightness) {
  leds[0] = CRGB(red * brightness, green * brightness, blue * brightness);
  FastLED.show();
}
#endif  // USE_LED

// MPU6050 Functions
#if USE_MPU6050
/**
 * @brief Configures USE_MPU6050 pins as outputs and initializes USE_MPU6050
 * class
 */
void CAR_HAL::MPU6050_Setup(bool test = false) {
  uint8_t devID;
  Wire.begin();
  mpu.initialize();
  if (test && !mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    devID = mpu.getDeviceID();
    if (devID != 0x00) {
      Serial.print("Expected MPU6050 Device ID 0x34, found 0x");
      Serial.println(devID, HEX);
    }
  }
}

/**
 * @brief Calibrates the MPU6050
 */
void CAR_HAL::MPU6050_Calibrate(bool quick = false) {
  Serial.println("Calibrating MPU6050... (Don't move the car!)");
  if (quick) {
    Serial.println("Running quick calibration");
    Serial.println("\nCalibrating Accelerometer");
    mpu.CalibrateAccel(2);
    Serial.println("\nCalibrating Gyroscope");
    mpu.CalibrateGyro(2);
  } else {
    Serial.println("Running slow calibration");
    Serial.println("\nCalibrating Accelerometer");
    mpu.CalibrateAccel(10);
    Serial.println("\nCalibrating Gyroscope");
    mpu.CalibrateGyro(10);
  }
  Serial.println("\nCalibration complete! Calibration offsets:");
  mpu.PrintActiveOffsets();
}

/**
 * @brief Gets raw data from the MPU6050
 * @param ax Acceleration in x-axis
 * @param ay Acceleration in y-axis
 * @param az Acceleration in z-axis
 * @param gx Gyroscope in x-axis
 * @param gy Gyroscope in y-axis
 * @param gz Gyroscope in z-axis
 */
void CAR_HAL::MPU6050_GetData(int16_t *ax, int16_t *ay, int16_t *az,
                              int16_t *gx, int16_t *gy, int16_t *gz) {
  mpu.getMotion6(ax, ay, az, gx, gy, gz);
}

/**
 * @brief Gets raw acceleration data from the MPU6050
 * @param ax Acceleration in x-axis
 * @param ay Acceleration in y-axis
 * @param az Acceleration in z-axis
 */
void CAR_HAL::MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az) {
  mpu.getAcceleration(ax, ay, az);
}

/**
 * @brief Gets raw gyroscope data from the MPU6050
 * @param gx Gyroscope in x-axis
 * @param gy Gyroscope in y-axis
 * @param gz Gyroscope in z-axis
 */
void CAR_HAL::MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz) {
  mpu.getRotation(gx, gy, gz);
}

/**
 * @brief Gets the temperature from the MPU6050
 * @return Temperature in degrees Celsius
 */
float CAR_HAL::MPU6050_GetTemp() { return mpu.getTemperature(); }
#endif  // USE_MPU6050

// Line tracking functions
#if USE_LINE_TRACKING
/**
 * @brief Configures line tracking pins as inputs
 */
void CAR_HAL::LineTracking_Setup() {
  pinMode(PIN_LineTracking_L, INPUT);
  pinMode(PIN_LineTracking_C, INPUT);
  pinMode(PIN_LineTracking_R, INPUT);
}

/**
 * @brief Gets line tracking sensor data for a given sensor
 * @param sensor Sensor id (0 = left, 1 = center, 2 = right)
 * @return ADC value from the sensor
 */
int16_t CAR_HAL::LineTracking_GetValue(uint8_t sensor) {
  switch (sensor) {
    case 0:
      return analogRead(PIN_LineTracking_L);
    case 1:
      return analogRead(PIN_LineTracking_C);
    case 2:
      return analogRead(PIN_LineTracking_R);
    default:
      return -1;
  }
}
#endif  // USE_LINE_TRACKING

// IR functions
#if USE_IR
#define USE_IRREMOTE_HPP_AS_PLAIN_INCLUDE
#include <IRremote.hpp>
/**
 * @brief Configures IR pins as inputs
 */
void CAR_HAL::IR_Setup() {
  pinMode(PIN_IR, INPUT);
  IrReceiver.begin(PIN_IR);
}

/**
 * @brief Gets last received IR message
 * @return IR data
 */
bool CAR_HAL::IR_GetMessage(uint16_t *message) {
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(
          F("Received noise or an unknown (or not yet enabled) protocol"));
      // We have an unknown protocol here, print more info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
      Serial.println();
    }
    IrReceiver.resume();  // Enable receiving of the next value
    *message = IrReceiver.decodedIRData.command;
    return true;
  }
  return false;
}
#endif  // USE_IR