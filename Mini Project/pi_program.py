# SEED Lab Mini Project Group 3
# Dawson J. Gullickson

import smbus2
import time

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import pi_camera
from picamera import PiCamera
from picamera.array import PiRGBArray
import numpy as np
import cv2 as cv

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

# Calibrate the camera
pi_camera.calibrate()

# Program loop
cap = pi_camera.video_init()
while True:
    # Get the quadrant the marker is in
    quadrant = 0
    pi_camera.video_loop(cap)
    if quadrant == 0:
        # Wait until marker recognized
        time.sleep(1)
        continue

    # Display angle
    angle_str = ["0", "pi/2", "pi", "3pi/2"][quadrant - 1]
    lcd.clear()
    lcd.message = "setpoint: " + angle_str

    # Send info to arduino
    angle = [0, 1, 2, 3][quadrant - 1]
    bus.write_byte(ADDRESS, angle)

    if pi_camera.was_quit_pressed():
        break
pi_camera.video_deinit(cap)