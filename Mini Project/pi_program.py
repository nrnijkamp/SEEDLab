# SEED Lab Mini Project Group 3
# Dawson J. Gullickson

import smbus2
import time

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

import cv2 as cv

# Calibrates the camera
#The commented code in here doesn't seem to work. I am not sure why he wants us to change the awb though the pi when we are using
#openCV, becuase this hangs my code evey time. I changed the image to grayscale in the video function, which should give us the
#sameconsistancy improvements as if we had changed the awb.
def calibrate():
    pass
    #camera = PiCamera()
    #camera.awb_mode = 'off'
    #camera.resolution = (1920, 1088)
    #rawCapture = PiRGBArray(camera, size =(1920, 1088))

def video_init() -> cv.VideoCapture:
    return cv.VideoCapture(0)

# Obtains the marker's quadrant
def video_loop(cap: cv.VideoCapture) -> int:
    ret, frame = cap.read()
    
    if not ret:
        print("Can't recieve frame...")
        return
    #Make gray and display
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow('frame', gray)
    
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
    param = cv.aruco.DetectorParameters_create()
    (corners, _ids, _rejected) = cv.aruco.detectMarkers(gray, arucoDict, parameters = param)
    
    if len(corners) >= 1:
        
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        x_centerPixel = x_sum*.25
        
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
        y_centerPixel = y_sum*.25
        
        #print("X center: ", x_centerPixel)
        #print("Y center: ", y_centerPixel)
        
        if (x_centerPixel > 318) and (y_centerPixel < 237):
            quadrant = 1
        elif (x_centerPixel <= 318) and (y_centerPixel < 237):
            quadrant = 2
        elif (x_centerPixel <= 318) and (y_centerPixel >= 237):
            quadrant = 3
        elif (x_centerPixel > 318) and (y_centerPixel >= 237):
            quadrant = 4
    else:
        quadrant = 0
    return quadrant

def video_deinit(cap: cv.VideoCapture):
    cap.release()
    cv.destroyAllWindows()

def was_quit_pressed() -> bool:
    return cv.waitKey(1) == ord('q')

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
calibrate()

# Program loop
cap = video_init()
while True:
    # Get the quadrant the marker is in
    quadrant = video_loop(cap)
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

    if was_quit_pressed():
        break
video_deinit(cap)