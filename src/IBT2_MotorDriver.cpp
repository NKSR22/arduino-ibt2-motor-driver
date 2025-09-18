/*
 * IBT2_MotorDriver.cpp
 * Implementation file for the IBT2_MotorDriver class.
 */

#include "IBT2_MotorDriver.h"

// Constructor: Initializes the pin numbers.
IBT2_MotorDriver::IBT2_MotorDriver(int pinRPWM, int pinLPWM, int pinIS) {
    _pinRPWM = pinRPWM;
    _pinLPWM = pinLPWM;
    _pinIS = pinIS;
}

// begin: Initializes the driver with default hardware settings.
void IBT2_MotorDriver::begin() {
    // Call the overloaded begin with default values from the header.
    begin(IBT2_DEFAULT_MAX_PWM, IBT2_DEFAULT_ADC_RESOLUTION, IBT2_DEFAULT_VREF);
}

// begin (overloaded): Initializes the driver with custom hardware settings.
void IBT2_MotorDriver::begin(int max_pwm, float adc_resolution, float v_ref) {
    // Store hardware configuration
    _max_pwm = max_pwm;
    _adc_resolution = adc_resolution;
    _v_ref = v_ref;

    // Set default current sense parameters and calculate the conversion factor
    setCurrentSense(IBT2_DEFAULT_R_IS, IBT2_DEFAULT_K_ILIS);

    // Set pin modes
    pinMode(_pinRPWM, OUTPUT);
    pinMode(_pinLPWM, OUTPUT);
    if (_pinIS >= 0) {
        pinMode(_pinIS, INPUT); // Standard input for analog pins
    }

    // Start with the motor stopped for safety.
    stop();
}

// drive: Controls the motor's speed and direction.
void IBT2_MotorDriver::drive(int speed) {
    // Constrain the speed to the valid PWM range for the MCU.
    speed = constrain(speed, -_max_pwm, _max_pwm);

    if (speed > 0) {
        // Forward rotation
        analogWrite(_pinLPWM, 0);
        analogWrite(_pinRPWM, speed);
    } else {
        // Reverse rotation or brake
        // If speed is 0, -speed is 0, which results in a brake.
        analogWrite(_pinRPWM, 0);
        analogWrite(_pinLPWM, -speed);
    }
}

// stop: Halts the motor using an electronic brake.
void IBT2_MotorDriver::stop() {
    // Setting both PWM pins to LOW creates a brake effect.
    analogWrite(_pinRPWM, 0);
    analogWrite(_pinLPWM, 0);
}

// getCurrent: Reads and calculates the motor's current draw using a pre-calculated factor.
float IBT2_MotorDriver::getCurrent() {
    // If the current sense pin is not configured, return an error value.
    if (_pinIS < 0) {
        return -1.0f;
    }

    // Read the raw ADC value and multiply by the pre-calculated factor.
    return analogRead(_pinIS) * _current_conversion_factor;
}

// setCurrentSense: Allows overriding the default current sense parameters and recalculates the conversion factor.
void IBT2_MotorDriver::setCurrentSense(float r_is, float k_ilis) {
    _r_is = r_is;
    _k_ilis = k_ilis;

    // Pre-calculate the conversion factor to optimize getCurrent()
    if (_adc_resolution > 0 && _r_is > 0) {
        _current_conversion_factor = _v_ref * _k_ilis / (_adc_resolution * _r_is);
    } else {
        _current_conversion_factor = 0;
    }
}