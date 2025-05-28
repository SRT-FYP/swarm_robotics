// motor_driver.cpp
#include "my_robot_control/motor_driver.hpp"
#include <chrono>
#include <cmath>
#include <iostream>

PiMotorDriver::PiMotorDriver() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed" << std::endl;
    }
}

PiMotorDriver::~PiMotorDriver() {
    gpioTerminate();
}

bool PiMotorDriver::initialize() {
    return gpioInitialise() >= 0;
}

void PiMotorDriver::setVelocity(int pwm_pin, int forward_pin, int backward_pin, double velocity) {
    // Set direction
    if (velocity >= 0) {
        gpioWrite(forward_pin, 1); // Forward
        gpioWrite(backward_pin, 0);
    } else {
        gpioWrite(forward_pin, 0);
        gpioWrite(backward_pin, 1); // Reverse
    }

    // Set PWM duty cycle (0-255)
    int pwm_value = static_cast<int>(fabs(velocity) * 255.0);
    gpioPWM(pwm_pin, std::min(pwm_value, 255));
}

// int PiMotorDriver::readEncoder(int encoder_pin) {
//     std::lock_guard<std::mutex> lock(encoder_mutex_);
//     return encoder_counts_[encoder_pin == LEFT_ENCODER_PIN ? 0 : 1];
// }

// double PiMotorDriver::readVelocity(int encoder_pin) {
//     std::lock_guard<std::mutex> lock(encoder_mutex_);
//     return current_velocity_[encoder_pin == LEFT_ENCODER_PIN ? 0 : 1];
// }

// void PiMotorDriver::encoderISR(int gpio, int level, uint32_t tick, void *user) {
//     PiMotorDriver *driver = static_cast<PiMotorDriver*>(user);
//     int index = (gpio == LEFT_ENCODER_PIN) ? 0 : 1;
    
//     std::lock_guard<std::mutex> lock(driver->encoder_mutex_);
    
//     // Update count
//     driver->encoder_counts_[index] += (level == 1) ? 1 : -1; // Direction detection
    
//     // Calculate velocity (ticks per second)
//     if (driver->last_encoder_time_[index] != 0) {
//         uint32_t delta_t = tick - driver->last_encoder_time_[index];
//         if (delta_t > 0) {
//             double ticks_per_second = 1000000.0 / delta_t; // pigpio uses microseconds
//             driver->current_velocity_[index] = (ticks_per_second / ENCODER_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
//         }
//     }
//     driver->last_encoder_time_[index] = tick;
// }