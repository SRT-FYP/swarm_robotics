from gpiozero import (
    PWMOutputDevice,
    DigitalOutputDevice,
    TonalBuzzer,
    Button,
)
import time

encoder_left = Button(5, bounce_time=0.005)
encoder_right = Button(6, bounce_time=0.005)

left_ticks = 0
right_ticks = 0

def count_left():
    global left_ticks
    left_ticks += 1
    print(f"Number of turn in the left: {left_ticks}")

def count_right():
    global right_ticks
    right_ticks += 1
    print(f"Number of turn in the right: {right_ticks}")


def main():
    while True:
        encoder_left.when_pressed = count_left
        encoder_right.when_pressed = count_right

if __name__ == "__main__":
    main()