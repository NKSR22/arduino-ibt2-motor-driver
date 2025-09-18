/*
 * IBT2_MotorDriver.h
 * A library for controlling the IBT-2 (BTS7960) H-Bridge Motor Driver.
 *
 * This library provides a simple interface to control DC motors
 * using the IBT-2 motor driver. It handles forward/reverse movement,
 * speed control via PWM, braking, and current sensing.
 *
 * Features:
 * - Speed and direction control.
 * - Braking functionality.
 * - Motor current sensing.
 * - Configurable for various MCUs (PWM range, ADC resolution, V-ref).
 */

#ifndef IBT2_MOTOR_DRIVER_H
#define IBT2_MOTOR_DRIVER_H

#include <Arduino.h>

// Default configuration values
#define IBT2_DEFAULT_MAX_PWM 255         // 8-bit PWM
#define IBT2_DEFAULT_ADC_RESOLUTION 1023.0f // 10-bit ADC (e.g., Arduino Uno)
#define IBT2_DEFAULT_VREF 5.0f           // 5V reference voltage
#define IBT2_DEFAULT_R_IS 1000.0f        // 1k Ohm sense resistor
#define IBT2_DEFAULT_K_ILIS 8500.0f      // 8500 A/V current sense ratio for BTS7960

class IBT2_MotorDriver {
public:
    /**
     * @brief Constructor for the motor driver.
     * @param pinRPWM The pin for Right PWM (controls forward rotation).
     * @param pinLPWM The pin for Left PWM (controls reverse rotation).
     * @param pinIS The pin for Current Sense (must be an analog pin).
     */
    IBT2_MotorDriver(int pinRPWM, int pinLPWM, int pinIS);

    /**
     * @brief Initializes the motor driver.
     *        Sets pin modes and applies default hardware configurations.
     *        Call this in your setup() function.
     */
    void begin();

    /**
     * @brief Overloaded begin method to set custom hardware configurations.
     * @param max_pwm The maximum PWM value for your MCU (e.g., 255, 1023, 4095).
     * @param adc_resolution The ADC resolution of your MCU (e.g., 1023.0 for 10-bit).
     * @param v_ref The reference voltage of your MCU's ADC (e.g., 5.0, 3.3).
     */
    void begin(int max_pwm, float adc_resolution, float v_ref);

    /**
     * @brief Drives the motor at a specified speed and direction.
     * @param speed The desired speed, from -max_pwm (full reverse) to +max_pwm (full forward).
     *              A value of 0 will brake the motor.
     */
    void drive(int speed);

    /**
     * @brief Stops the motor by applying an electronic brake.
     *        This is achieved by setting both PWM signals to LOW.
     */
    void stop();

    /**
     * @brief Reads and calculates the current drawn by the motor.
     * @return The motor current in Amperes (A). Returns -1.0 on error (e.g., no IS pin).
     */
    float getCurrent();

    /**
     * @brief Sets the custom parameters for the current sensing calculation.
     * @param r_is The value of the sense resistor (in Ohms).
     * @param k_ilis The current sense ratio from the driver datasheet (e.g., 8500 for BTS7960).
     */
    void setCurrentSense(float r_is, float k_ilis);

private:
    // Pin assignments
    int _pinRPWM;
    int _pinLPWM;
    int _pinIS;

    // Hardware configuration
    int _max_pwm;
    float _adc_resolution;
    float _v_ref;

    // Current sensing parameters
    float _r_is;
    float _k_ilis;
    float _current_conversion_factor; // Pre-calculated factor for performance
};

#endif // IBT2_MOTOR_DRIVER_H