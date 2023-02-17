import smbus2
import time

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import pi_camera

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
while True:
    # Get the quadrant the marker is in
    quadrant = 0
    pi_camera.video()
    if quadrant == 0:
        # Wait until marker recognized
        time.sleep(1)
        continue

    # Display angle
    angle_str = ["0", "π/2", "π", "3π/2"][quadrant - 1]
    lcd.clear()
    lcd.message = f"setpoint: {angle_str}"

    # Send info to arduino
    #TODO