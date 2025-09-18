#include <IBT2_MotorDriver.h>

// Define the pins used for the motor driver
const int PIN_RPWM = 10;
const int PIN_LPWM = 9;
const int PIN_IS = A0;

// Create a motor driver object
IBT2_MotorDriver motor(PIN_RPWM, PIN_LPWM, PIN_IS);

void setup() {
  Serial.begin(9600);

  // Initialize the motor driver
  // For a standard Arduino Uno (8-bit PWM, 10-bit ADC, 5V), the default is fine.
  motor.begin();

  // For an ESP32, you might do this instead:
  // int max_pwm = 1023;       // 10-bit PWM
  // float adc_res = 4095.0;   // 12-bit ADC
  // float v_ref = 3.3;        // 3.3V
  // motor.begin(max_pwm, adc_res, v_ref);
}

void loop() {
  // Drive forward at half speed
  Serial.println("Driving forward at half speed...");
  motor.drive(128);
  delay(2000);

  // Print the current draw
  float current = motor.getCurrent();
  Serial.print("Motor current: ");
  Serial.print(current);
  Serial.println(" A");
  delay(1000);

  // Brake the motor
  Serial.println("Braking...");
  motor.stop();
  delay(2000);

  // Drive reverse at full speed
  Serial.println("Driving reverse at full speed...");
  motor.drive(-255);
  delay(2000);

  // Print the current draw
  current = motor.getCurrent();
  Serial.print("Motor current: ");
  Serial.print(current);
  Serial.println(" A");
  delay(1000);

  // Brake the motor
  Serial.println("Braking...");
  motor.stop();
  delay(3000);
}
