// motor_driver.hpp
#pragma once

#include <pigpio.h>
#include <memory>
#include <mutex>

class PiMotorDriver {
public:
    PiMotorDriver();
    ~PiMotorDriver();

    bool initialize();
    void setVelocity(int pwm_pin, int forward_pin, int backward_pin, double velocity);
    int readEncoder(int encoder_pin);
    double readVelocity(int encoder_pin);

private:
    static constexpr double ENCODER_TICKS_PER_REV = 20.0; // Change based on your encoder
    static constexpr double WHEEL_CIRCUMFERENCE = 0.2;    // meters, change based on your wheel
    
    std::mutex encoder_mutex_;
    volatile int encoder_counts_[2] = {0};
    int last_encoder_time_[2] = {0};
    double current_velocity_[2] = {0.0};
    
    static void encoderISR(int gpio, int level, uint32_t tick, void *user);
};