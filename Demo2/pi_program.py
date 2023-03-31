# SEED Lab Demo 2 Group 3
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
MAX_DIST = 10
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

    # Display angle
    lcd.message = "Angle: {:.2}".format(angle)

    # Convert data to bytes (0-255)
    angle_byte = int((angle+180)*255/360) % 256
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
camera = pi_camera.video_init()

# Send initial searching instruction
send_instruction(True)

# Program loop
searching = True
last_angle = 0.0
last_dist = 0.0
while True:
    # Get the quadrant the marker is in
    angle, x_dist, y_dist, corners = pi_camera.video_loop(camera)

    # Send instruction
    if corners == 0:
        # No markers detected; start searching unless already searching
        if (searching): continue
        send_instruction(True)
        searching = True
    else:
        # Markers detected; send info unless only small change
        dist = math.sqrt(x_dist**2 + y_dist**2)
        if (abs(dist - last_dist) < 0.1): continue
        if (abs(angle - last_angle) < 0.1): continue
        print("Sending {} and {}".format(dist, angle))
        send_instruction(False, angle, dist)
        searching = False
        last_angle = angle
        last_dist = dist

    if pi_camera.was_quit_pressed():
        break
pi_camera.video_deinit(camera)