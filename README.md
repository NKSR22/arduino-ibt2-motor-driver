# IBT-2 (BTS7960) Motor Driver Library

A flexible and efficient Arduino library for controlling the IBT-2 (also known as BTS7960) H-Bridge motor driver.

This library is designed to be easy to use while providing the flexibility to work with a wide range of microcontrollers (MCUs) like Arduino AVR boards, ESP32, STM32, and more. It allows for easy configuration of hardware-specific parameters such as PWM range, ADC resolution, and ADC reference voltage.

## Features

- **Easy-to-use API:** Simple methods for driving and stopping the motor.
- **Speed and Direction Control:** Control motor speed and direction with a single `drive()` command.
- **MCU Agnostic:** Easily configure for different MCU hardware (PWM duty cycle, ADC resolution, etc.).
- **Current Sensing:** Read the current drawn by the motor.
- **Performance Optimized:** Current sensing calculation is optimized to use a pre-calculated conversion factor, minimizing floating-point math in loops.
- **Configurable Parameters:** Adjust current-sensing parameters to match your specific hardware setup.

## Hardware
### IBT-2 / BTS7960 Driver
The IBT-2 is a high-power H-bridge motor driver based on the Infineon BTS7960 chipset. It is capable of handling high currents (up to 43A) and is suitable for driving large DC motors.

**Pinout:**
- **VCC:** 5V logic supply
- **GND:** Ground
- **R_PWM / RPWM:** Forward PWM signal
- **L_PWM / LPWM:** Reverse PWM signal
- **R_EN / REN:** Right (Forward) Enable (tie to VCC)
- **L_EN / LEN:** Left (Reverse) Enable (tie to VCC)
- **R_IS / RIS:** Right (Forward) Current Sense output
- **L_IS / LIS:** Left (Reverse) Current Sense output

**Note:** On most IBT-2 boards, R_EN and L_EN are enabled by default or can be enabled by jumping them to 5V. This library assumes they are enabled. The R_IS and L_IS pins are often tied together, providing a single current sense output.

### Wiring to an Arduino

| IBT-2 Pin | Arduino Pin              | Description                            |
|-----------|--------------------------|----------------------------------------|
| VCC       | 5V                       | Logic power supply                     |
| GND       | GND                      | Ground                                 |
| RPWM      | Any PWM Pin (e.g., `~10`)  | Forward PWM control                    |
| LPWM      | Any PWM Pin (e.g., `~9`)   | Reverse PWM control                    |
| R_IS/L_IS | Any Analog Pin (e.g., `A0`)| Motor current sense feedback           |
| R_EN      | 5V                       | Enable forward driver (tie high)       |
| L_EN      | 5V                       | Enable reverse driver (tie high)       |

*Connect your motor to the `M+/M-` screw terminals and your power supply to the `B+/B-` terminals.*

## Installation

1.  Download the latest release of this library from the [releases page](https://github.com/your-repo/your-library/releases).
2.  In the Arduino IDE, go to `Sketch` -> `Include Library` -> `Add .ZIP Library...`.
3.  Select the downloaded ZIP file.
4.  The library will now be available in the `Sketch` -> `Include Library` menu.

## API Reference

### `IBT2_MotorDriver(int pinRPWM, int pinLPWM, int pinIS)`
**Description:** The constructor for the class.
- `pinRPWM`: The MCU pin connected to the driver's RPWM pin.
- `pinLPWM`: The MCU pin connected to the driver's LPWM pin.
- `pinIS`: The MCU pin connected to the driver's IS (Current Sense) pin. Use `-1` if not used.

### `begin()`
**Description:** Initializes the driver with default settings (8-bit PWM, 10-bit ADC, 5V V-ref). Call this in `setup()`.

### `begin(int max_pwm, float adc_resolution, float v_ref)`
**Description:** Initializes the driver with custom hardware settings.
- `max_pwm`: The maximum PWM value for your MCU (e.g., `255` for 8-bit, `1023` for 10-bit).
- `adc_resolution`: The ADC resolution of your MCU (e.g., `1023.0` for 10-bit, `4095.0` for 12-bit).
- `v_ref`: The ADC reference voltage of your MCU (e.g., `5.0` or `3.3`).

### `drive(int speed)`
**Description:** Drives the motor.
- `speed`: A value from `-max_pwm` (full reverse) to `+max_pwm` (full forward). `0` brakes the motor.

### `stop()`
**Description:** Stops the motor using an electronic brake.

### `getCurrent()`
**Description:** Returns the measured motor current in Amperes (A). Returns `-1.0` if the IS pin was not configured.

### `setCurrentSense(float r_is, float k_ilis)`
**Description:** Sets custom parameters for current sensing.
- `r_is`: The value of the sense resistor on your board in Ohms (typically 1000).
- `k_ilis`: The current sense ratio from the datasheet (typically 8500 for BTS7960).

## Basic Usage Example

```cpp
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
```
