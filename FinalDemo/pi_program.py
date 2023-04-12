# SEED Lab Final Demo Group 3
# Dawson J. Gullickson

import math
from typing import Optional

import smbus2

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import piCode.piCode as pi_camera

# Chosen by the Arduino
ADDRESS = 0x08

# Initialize I2C Bus
bus = smbus2.SMBus(1)

# LCD Size
lcd_columns = 16
lcd_rows = 2

# Initialize I2C bus
i2c = board.I2C()

# Initialize LCD
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# Function to send info to lcd and Arduino
SEARCH_INST = 0
MOVE_INST = 1
DISTANCE_THRESHOLD = 2
MAX_DIST = 5
MIN_ANGLE = -math.pi/3
MAX_ANGLE = math.pi/3
ANGLE_RANGE=MAX_ANGLE-MIN_ANGLE
def send_instruction(
    searching: bool,
    angle: Optional[float] = None,
    distance: Optional[float] = None
):
    if searching:
        bus.write_byte(ADDRESS, SEARCH_INST)
        return
    assert angle is not None
    assert distance is not None

    # # Display angle
    # lcd.message = "Angle: {:.2}".format(angle)

    # Don't send angle when under threshold
    if distance < DISTANCE_THRESHOLD: angle = 0

    # Convert data to bytes (0-255)
    print("Sending {} and {}".format(angle, distance))
    angle *= math.pi/180
    angle_byte = int((angle-MIN_ANGLE)*255//ANGLE_RANGE) % 256
    distance_byte = int((distance*255)//MAX_DIST)
    if distance_byte < 0: distance_byte = 0
    if 255 < distance_byte: distance_byte = 255
    print("Bytes {} and {}".format(angle_byte, distance_byte))

    # Send info to arduino
    # bus.write_byte(ADDRESS, MOVE_INST)
    # bus.write_byte(ADDRESS, angle_byte)
    # bus.write_byte(ADDRESS, distance_byte)
    bus.write_i2c_block_data(ADDRESS, MOVE_INST, [angle_byte, distance_byte])

# Calibrate camera
camera_state = pi_camera.video_init()

# Send initial searching instruction
send_instruction(True)

# Program loop
searching = True
last_angle = 0.0
last_dist = 0.0
while True:
    # Get marker info
    result = pi_camera.video_loop(camera_state)
    if result is None: break
    angle, x_dist, y_dist, corners = result

    # Send instruction
    if corners == 0:
        # No markers detected; start searching unless already searching
        if (searching): continue
        # send_instruction(True)
        searching = True
    else:
        # Markers detected; send info unless only small change
        dist = math.sqrt(x_dist**2 + y_dist**2)
        if (abs(dist - last_dist) < 0.1): continue
        if (abs(angle - last_angle) < 0.1): continue
        send_instruction(False, angle, dist)
        searching = False
        last_angle = angle
        last_dist = dist
pi_camera.video_deinit(camera_state)