import pygame
import serial
import time
import math

# =========================
# CONFIG UART
# =========================
SERIAL_PORT = "COM4"
BAUDRATE = 115200

SYNC1 = 0xAA
SYNC2 = 0x55
TYPE_CMD = 0x01
TYPE_SAVE = 0x02
save_request = 0

# =========================
# CRC8
# =========================
def crc8(data):
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

# =========================
# INIT SERIAL
# =========================
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

# =========================
# INIT GAMEPAD
# =========================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print("Joystick detected:", joystick.get_name())

# =========================
# FLIGHT MODES
# =========================
SAFE = 0
MANUAL = 1
HOLD = 2
AUTO = 3

flight_mode = SAFE

motor_hold_value = 0.0
hold_time = 0.0

prev_triangle = False
prev_x = False

# =========================
# STATE VARIABLES
# =========================
yaw = 0.0
pitch_offset = 0.0

# =========================
# LOOP
# =========================
while True:
    pygame.event.pump()

    # =========================================================
    # MODE SWITCHES
    # =========================================================
    triangle = joystick.get_button(3)
    x_button = joystick.get_button(0)

    # SAFE <-> MANUAL
    if triangle and not prev_triangle:
        if flight_mode == SAFE:
            flight_mode = MANUAL
        else:
            flight_mode = SAFE

    prev_triangle = triangle

    # HOLD activation
    if x_button and not prev_x and flight_mode == MANUAL:
        motor_hold_value = (joystick.get_axis(5) + 1) / 2
        hold_time = time.time()
        flight_mode = HOLD

    prev_x = x_button

    # =========================================================
    # INPUTS
    # =========================================================

    r2 = (joystick.get_axis(5) + 1) / 2
    r2 = max(0.0, min(1.0, r2))

    # =========================================================
    # HOLD LOGIC CORRIGÉE
    # =========================================================
    if flight_mode == HOLD:
        if joystick.get_button(10):
            motor_hold_value += 0.02
        if joystick.get_button(9):
            motor_hold_value -= 0.02
        motor_hold_value = max(0, min(1.0, motor_hold_value))
        # période d'ignorance R2
        if time.time() - hold_time < 2.0:
            r2_effective = 0.0
        else:
            r2_effective = r2

            # reprise contrôle si R2 réutilisé
            if r2_effective > 0.1:
                flight_mode = MANUAL

    else:
        r2_effective = r2

    # =========================================================
    # YAW
    # =========================================================
    yaw_input = joystick.get_axis(0)
    if abs(yaw_input) < 0.05:
        yaw_input = 0.0

    #yaw += yaw_input * 0.1

    if joystick.get_button(14):
        yaw += 0.01
    if joystick.get_button(13):
        yaw -= 0.01

    #yaw = yaw % (2 * math.pi)
    #yaw_byte = int(((yaw / (2 * math.pi)) * 255)
    yaw_byte = int(255 * (yaw+yaw_input+1)/2)
    yaw_byte = max(0, min(255, yaw_byte))

    # =========================================================
    # PITCH
    # =========================================================
    pitch_input = joystick.get_axis(3)
    if abs(pitch_input) < 0.08:
        pitch_input = 0.0

    pitch = pitch_input + pitch_offset
    pitch = max(-1.0, min(1.0, pitch))
    pitch_byte = int(((pitch + 1) / 2) * 255)

    if joystick.get_button(12):
        pitch_offset += 0.01
    if joystick.get_button(11):
        pitch_offset -= 0.01

    # =========================================================
    # MOTOR STATE MACHINE
    # =========================================================
    if flight_mode == SAFE:
        motor = 0

    elif flight_mode == MANUAL:
        motor = int((0.25 + 0.75 * r2_effective) * 255)

    elif flight_mode == HOLD:
        motor = int((0.25 + 0.75 * motor_hold_value) * 255)

    elif flight_mode == AUTO:
        motor = 0

    # =========================================================
    # SAVE STATE MACHINE
    # =========================================================
    square = joystick.get_button(2)
    circle = joystick.get_button(1)
    
    # =========================
    # SAVE REQUEST HANDLING
    # =========================
    if square:
        save_request = 1
    elif circle:
        save_request = 2
    
    # =========================================================
    # UART FRAME
    # =========================================================
    
    if save_request != 0:
        tx_type = TYPE_SAVE
        motor_tx = save_request   # 1 = square, 2 = circle
        save_request = 0          # reset après envoi
    else:
        tx_type = TYPE_CMD
        motor_tx = motor
    
    payload = [tx_type, motor_tx, yaw_byte, pitch_byte]
    crc = crc8(payload)
    
    frame = bytes([
        SYNC1,
        SYNC2,
        tx_type,
        motor_tx,
        yaw_byte,
        pitch_byte,
        crc
    ])
    
    ser.write(frame)

    # =========================================================
    # DEBUG
    # =========================================================
    mode_name = ["SAFE", "MANUAL", "HOLD", "AUTO"][flight_mode]

    print(
        f"[{mode_name}] "
        f"Motor: {motor:3d} | "
        f"Yaw: {yaw+yaw_input:.2f} | "
        f"Pitch: {pitch:.2f}"
    )

    time.sleep(0.05)