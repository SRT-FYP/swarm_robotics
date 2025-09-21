from gpiozero import PWMLED
from time import sleep

# Define RGB LED pins (BCM mode)
red = PWMLED(17)
green = PWMLED(27)
blue = PWMLED(22)

def set_color(r, g, b):
    red.value = r  # Value between 0 and 1
    green.value = g
    blue.value = b

try:
    while True:
        set_color(1, 0, 0)  # Red
        sleep(1)
        set_color(0, 1, 0)  # Green
        sleep(1)
        set_color(0, 0, 1)  # Blue
        sleep(1)
        set_color(1, 1, 0)  # Yellow
        sleep(1)
        set_color(0, 1, 1)  # Cyan
        sleep(1)
        set_color(1, 0, 1)  # Magenta
        sleep(1)
        set_color(1, 1, 1)  # White
        sleep(1)
        set_color(0, 0, 0)  # Off
        sleep(1)
except KeyboardInterrupt:
    pass
