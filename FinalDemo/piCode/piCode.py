# SEED Lab, Team 3, Demo 2

from typing import Any

from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2 as cv
import math

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# LCD Size
lcd_columns = 16
lcd_rows = 2

# Initialize I2C bus
i2c = board.I2C()

# Initialize LCD
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

class CameraState:
    def __init__(self, camera: PiCamera, curr_marker: int):
        self.camera = camera
        self.curr_marker = curr_marker

# Calibrates the camera
# https://picamera.readthedocs.io/en/release-1.13/api_camera.html
def video_init() -> CameraState:
    camera = PiCamera()

    # Set light sensitivity
    camera.iso = 800
    camera.brightness = 70
    
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = "off"

    # Get automatic white balance
    camera.awb_mode = "auto"
    gains = camera.awb_gains
    # Set constant awb from automatic detection
    camera.awb_mode = "off"
    camera.awb_gains = gains

    return CameraState(camera, curr_marker=1)

# Obtains the marker's position
def video_loop(camera_state: CameraState) -> Any:
    # https://picamera.readthedocs.io/en/release-1.13/api_array.html
    camera = camera_state.camera
    raw_capture = PiRGBArray(camera)
    camera.capture(raw_capture, "bgr", use_video_port=False)
    
    # Display image
    img = raw_capture.array
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # cv.imshow("Image", img)
    # if cv.waitKey(1) & 0xFF == ord('q'):
    #     return None
    
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_1000)
    param = cv.aruco.DetectorParameters_create()
    corners, ids, _rejected = cv.aruco.detectMarkers(img, arucoDict, parameters=param)
    
    # Find position and calculate angle
    # We want the bottom center of the y part of the marker because I did all the calibraion on the ground, and I'm assuming that is where the angle is going to be measured in relation to the robot
    angle = 0.0
    xDistanceInFeet = 0.0
    yDistanceInFeet = 0.0
    saw_marker = False
    if len(corners) >= 1:
        for j in range(len(ids)):
            #print(ids[j][0])
            if ids[j][0] == camera_state.curr_marker:
                saw_marker = True
                print(ids[j][0])
        
                xSum = corners[j][0][0][0] + corners[j][0][1][0] + corners[j][0][2][0] + corners[j][0][3][0]
                xCenterPixel = xSum / 4
                
                ySum = corners[j][0][2][1] + corners[j][0][3][1]
                yBottomCenterPixel = ySum / 2
                
                #print("X center: ", x_centerPixel)
                #print("Y center: ", y_bottom_centerPixel)
                
                #Did a calibration picture with the angle that we have on the camera. This set of equations will convert pixel coors to real life feet away.
                #With real feet as our distance, we can find the real world angle
                xPixelsPerFoot = 1.2982 * yBottomCenterPixel + 215.55
                xDistanceInFeet = (960 - xCenterPixel) / xPixelsPerFoot
                yDistanceInFeet = 184.39 * yBottomCenterPixel ** -0.746
                
                angleInRad = math.atan(xDistanceInFeet / yDistanceInFeet)
                angle = angleInRad * 180 / math.pi

                angle = round(angle, 3)


                #print("Angle in rad:" , angleInRad)
                #print("Angle in degrees:", angle)
    else:
        # No markers detected
        print("No markets detected")

    # Close raw_capture
    raw_capture.close()
        
    #Return angle
    return (angle, xDistanceInFeet, yDistanceInFeet, saw_marker)

def video_deinit(camera: CameraState):
    camera_state.camera.close()
    cv.destroyAllWindows()

# Driver code
# Press 'q' to exit the video mode
# Only runs when not imported
if __name__ == "__main__":
    camera_state = video_init()
    while True:
        #if detectMarker >= 1, there is a marker. If ==0, no marker
        result = video_loop(camera_state)
        if result is None: break
        angle, xDistanceInFeet, yDistanceInFeet, detectMarker = result
        message = "Angle: {}".format(angle)
        print(message)
        # lcd.clear()
        # lcd.message = message
    video_deinit(camera_state)
    print("Done")

