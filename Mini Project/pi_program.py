# SEED Lab Mini Project Group 3
# Dawson J. Gullickson

import smbus2

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

# Function to send info to lcd and Arduino
#  `quadrant` should be in [1,4]
def update_quadrant(quadrant: int):
    quadrant = quadrant - 1
    
    # Display angle
    angle_str = ["0", "pi/2", "pi", "3pi/2"][quadrant]
    lcd.clear()
    lcd.message = "setpoint: " + angle_str

    # Send info to arduino
    angle = [0, 1, 2, 3][quadrant]
    bus.write_byte(ADDRESS, angle)

# Calibrate camera
camera = pi_camera.video_init()

# Send initial quadrant of 1
update_quadrant(1)

# Program loop
last_quadrant = 0
while True:
    # Get the quadrant the marker is in
    quadrant = pi_camera.video_loop(camera)
    if quadrant == 0 or quadrant == last_quadrant:
        continue
    last_quadrant = quadrant

    # Send quadrant (will never be 0)
    update_quadrant(quadrant)

    if pi_camera.was_quit_pressed():
        break
pi_camera.video_deinit(camera)