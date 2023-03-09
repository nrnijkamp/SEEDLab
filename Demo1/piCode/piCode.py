# SEED Lab Demo 1
# Group 3

from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2 as cv
import math

# Calibrates the camera
# https://picamera.readthedocs.io/en/release-1.13/api_camera.html
def video_init() -> PiCamera:
    camera = PiCamera()

    # Set light sensitivity
    camera.iso = 400 
    
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = "off"

    # Get automatic white balance
    camera.awb_mode = "auto"
    gains = camera.awb_gains
    # Set constant awb from automatic detection
    camera.awb_mode = "off"
    camera.awb_gains = gains

    return camera

# Obtains the marker's position
def video_loop(camera: PiCamera) -> int:
    # https://picamera.readthedocs.io/en/release-1.13/api_array.html
    raw_capture = PiRGBArray(camera)
    camera.capture(raw_capture, "bgr")
    
    # Display image
    img = raw_capture.array
    img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imshow("Image", img)
    
    arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
    param = cv.aruco.DetectorParameters_create()
    corners, _ids, _rejected = cv.aruco.detectMarkers(img, arucoDict, parameters = param)
    
    # Find position and calculate angle
    #We want the bottom center of the y part of the marker because I did all the calibraion on the ground, and I'm assuming that is where the angle is going to be measured in relation to the robot
    if len(corners) >= 1:
        xSum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        xCenterPixel = xSum/4
        
        ySum = corners[0][0][2][1]+ corners[0][0][3][1]
        yBottomCenterPixel = ySum/2
        
        #print("X center: ", x_centerPixel)
        #print("Y center: ", y_bottom_centerPixel)
        
        #Did a calibration picture with the angle that we have on the camera. This set of equations will convert pixel coors to real life feet away.
        #With real feet as our distance, we can find the real world angle
        xPixelsPerFoot = 1.2982* yBottomCenterPixel + 215.55
        xDistanceInFeet = (960 - xCenterPixel)/xPixelsPerFoot
        yDistanceInFeet = 184.39 * yBottomCenterPixel ** -0.746

        angleInRad = math.atan(xDistanceInFeet/yDistanceInFeet)
        angle = angleInRad * 180 / math.pi

        angle = round(angle,3)


        #print("Angle in rad:" , angleInRad)
        #print("Angle in degrees:", angle)
        
        #Return angle
        return angle

        
    else:
        # No markers detected
        print("No markets detected")

    # Close raw_capture
    #Is this why it's so slow? are we opening and closing the camera every time?
    raw_capture.close()

def video_deinit(camera: PiCamera):
    camera.close()
    cv.destroyAllWindows()

def was_quit_pressed() -> bool:
    return cv.waitKey(1) == ord('q')

#Driver code
#The angle variable is set the the angle the marker is at, when the video function is run
#Press 'q' to exit the video mode
# Only runs when not imported
if __name__ == "__main__":
    camera = video_init()
    while True:
        angle = video_loop(camera)
        print("Angle in degrees: ", angle)
        if was_quit_pressed():
            break
    video_deinit(camera)
    print("Done")
