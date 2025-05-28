import socket
import threading
from time import sleep
from gpiozero import (
    PWMOutputDevice,
    DigitalOutputDevice,
    TonalBuzzer,
    Button,
)
from gpiozero.tones import Tone

# --- Motor and Horn Setup ---

motor_pwm_left = PWMOutputDevice(12)   # GPIO 12 (PWM)
motor_pwm_right = PWMOutputDevice(13)  # GPIO 13 (PWM)

motor_a_forward = DigitalOutputDevice(22)
motor_a_backward = DigitalOutputDevice(23)
motor_b_forward = DigitalOutputDevice(24)
motor_b_backward = DigitalOutputDevice(25)

horn = TonalBuzzer(18)  # GPIO 18 (PWM buzzer)

# --- Encoder Setup (GPIO 5 and 6) ---
encoder_left = Button(5)
encoder_right = Button(6)

left_ticks = 0
right_ticks = 0

def count_left():
    global left_ticks
    left_ticks += 1
    # print(f"Number of turn in the left: {left_ticks}")

def count_right():
    global right_ticks
    right_ticks += 1
    # print(f"Number of turn in the right: {right_ticks}")

encoder_left.when_pressed = count_left
encoder_right.when_pressed = count_right

# --- Functions ---

def set_speed(speed: float):
    motor_pwm_left.value = speed
    motor_pwm_right.value = speed

def stop_all():
    motor_a_forward.off()
    motor_a_backward.off()
    motor_b_forward.off()
    motor_b_backward.off()
    motor_pwm_left.value = 0
    motor_pwm_right.value = 0
    horn.stop()

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

def horn_on():
    global left_ticks, right_ticks
    print("Executing: Horn on")
    horn.play(Tone("A4"))  # 440Hz tone
    left_ticks = 0
    right_ticks = 0
    return "Horn on"

def horn_off():
    print("Executing: Horn off")
    horn.stop()
    return "Horn off"

def stop():
    print("Executing: Stop")
    stop_all()
    return "Stopped"

# --- Command Map ---
command_map = {
    "forward": move_forward,
    "backward": move_backward,
    "left": turn_left,
    "right": turn_right,
    "horn": horn_on,
    "stopHorn": horn_off,
    "stop": stop
}

# --- Client Handler ---
def handle_client(client_socket):
    try:
        while True:
            command = client_socket.recv(1024).decode('utf-8').strip()
            if not command:
                break
            print(f"Received command: {command}")
            response = command_map.get(command, lambda: "Unknown command")()
            client_socket.sendall(response.encode('utf-8'))
    except Exception as e:
        print(f"Client error: {e}")
    finally:
        stop_all()
        client_socket.close()

# --- Main Server Function ---
def main():
    SERVER_IP = '0.0.0.0'
    SERVER_PORT = 8888

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((SERVER_IP, SERVER_PORT))
    server_socket.listen(5)
    print(f"Server listening on {SERVER_IP}:{SERVER_PORT}")

    try:
        while True:
            client_socket, client_address = server_socket.accept()
            print(f"Connection from {client_address}")
            handle_client(client_socket)
    except KeyboardInterrupt:
        print("Shutting down server.")
    finally:
        stop_all()
        server_socket.close()

if __name__ == "__main__":
    main()


# import socket
# from gpiozero import PWMOutputDevice, DigitalOutputDevice
# from time import sleep

# # Motor PWM speed control
# motor_pwm_left = PWMOutputDevice(12)   # GPIO 12
# motor_pwm_right = PWMOutputDevice(13)  # GPIO 13

# # Motor A (Left) direction control
# motor_a_forward = DigitalOutputDevice(22)
# motor_a_backward = DigitalOutputDevice(23)

# # Motor B (Right) direction control
# motor_b_forward = DigitalOutputDevice(24)
# motor_b_backward = DigitalOutputDevice(25)

# # Horn using PWM
# horn = PWMOutputDevice(18)  # GPIO 18

# # Set motor speed (shared)
# def set_speed(speed: float):
#     motor_pwm_left.value = speed
#     motor_pwm_right.value = speed

# # Stop all movement and horn
# def stop_all():
#     motor_a_forward.off()
#     motor_a_backward.off()
#     motor_b_forward.off()
#     motor_b_backward.off()
#     motor_pwm_left.value = 0
#     motor_pwm_right.value = 0
#     horn.off()

# # Movement functions
# def move_forward():
#     print("Executing: Move forward")
#     stop_all()
#     set_speed(0.8)
#     motor_a_forward.on()
#     motor_b_forward.on()
#     return "Moving forward"

# def move_backward():
#     print("Executing: Move backward")
#     stop_all()
#     set_speed(0.8)
#     motor_a_backward.on()
#     motor_b_backward.on()
#     return "Moving backward"

# def turn_left():
#     print("Executing: Turn left")
#     stop_all()
#     set_speed(0.6)
#     motor_a_backward.on()
#     motor_b_forward.on()
#     return "Turning left"

# def turn_right():
#     print("Executing: Turn right")
#     stop_all()
#     set_speed(0.6)
#     motor_a_forward.on()
#     motor_b_backward.on()
#     return "Turning right"

# def horn_on():
#     print("Executing: Horn on")
#     horn.value = 1.0
#     return "Horn on"

# def horn_off():
#     print("Executing: Horn off")
#     horn.off()
#     return "Horn off"

# def stop():
#     print("Executing: Stop")
#     stop_all()
#     return "Stopped"

# # Map commands to functions
# command_map = {
#     "forward": move_forward,
#     "backward": move_backward,
#     "left": turn_left,
#     "right": turn_right,
#     "horn": horn_on,
#     "stopHorn": horn_off,
#     "stop": stop
# }

# # Handle client connection
# def handle_client(client_socket):
#     try:
#         while True:
#             command = client_socket.recv(1024).decode('utf-8').strip()
#             if not command:
#                 break
#             print(f"Received command: {command}")
#             response = command_map.get(command, lambda: "Unknown command")()
#             client_socket.sendall(response.encode('utf-8'))
#     except Exception as e:
#         print(f"Client error: {e}")
#     finally:
#         stop_all()
#         client_socket.close()

# # Main server function
# def main():
#     SERVER_IP = '0.0.0.0'
#     SERVER_PORT = 8888

#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.bind((SERVER_IP, SERVER_PORT))
#     server_socket.listen(5)
#     print(f"Server listening on {SERVER_IP}:{SERVER_PORT}")

#     try:
#         while True:
#             client_socket, client_address = server_socket.accept()
#             print(f"Connection from {client_address}")
#             handle_client(client_socket)
#     except KeyboardInterrupt:
#         print("Shutting down server.")
#     finally:
#         stop_all()
#         server_socket.close()

# if __name__ == "__main__":
#     main()
