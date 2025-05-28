import time
from gpiozero import (
    PWMOutputDevice,
    DigitalOutputDevice,
)

motor_pwm_left = PWMOutputDevice(12)   # GPIO 12 (PWM)
motor_pwm_right = PWMOutputDevice(13)  # GPIO 13 (PWM)
# motor_pwm_left = DigitalOutputDevice(12)
# motor_pwm_right = DigitalOutputDevice(13)


motor_a_forward = DigitalOutputDevice(22)
motor_a_backward = DigitalOutputDevice(23)
motor_b_forward = DigitalOutputDevice(24)
motor_b_backward = DigitalOutputDevice(25)

def set_speed(speed: float):
    motor_pwm_left.value = speed
    motor_pwm_right.value = speed

# def set_speed(speed: float):
#     if speed > 0:
#         motor_pwm_left.on()
#         motor_pwm_right.on()
#     else:
#         motor_pwm_left.off()
#         motor_pwm_right.off()


def stop_all():
    motor_a_forward.off()
    motor_a_backward.off()
    motor_b_forward.off()
    motor_b_backward.off()
    motor_pwm_left.value = 0
    motor_pwm_right.value = 0
    # horn.stop()

def move_forward():
    print("Executing: Move forward")
    stop_all()
    set_speed(0.8)
    motor_a_forward.on()
    motor_b_forward.on()
    return "Moving forward"

def move_backward():
    print("Executing: Move backward")
    stop_all()
    set_speed(0.8)
    motor_a_backward.on()
    motor_b_backward.on()
    return "Moving backward"

def turn_left():
    print("Executing: Turn left")
    stop_all()
    set_speed(0.6)
    motor_a_backward.on()
    motor_b_forward.on()
    return "Turning left"

def turn_right():
    print("Executing: Turn right")
    stop_all()
    set_speed(0.6)
    motor_a_forward.on()
    motor_b_backward.on()
    return "Turning right"

def main():
    move_forward()
    time.sleep(50)
    stop_all()

if __name__ == "__main__":
    main()