# SEED Lab Mini Project Group 3
# Nick Nijkamp

from picamera import PiCamera
from picamera.array import PiRGBArray
import cv2 as cv

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

# Obtains the marker's quadrant
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
    
    # Set quadrant
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
        # No markers detected
        quadrant = 0

    # Close raw_capture
    raw_capture.close()
    # Return quadrant
    return quadrant

def video_deinit(camera: PiCamera):
    camera.close()
    cv.destroyAllWindows()

def was_quit_pressed() -> bool:
    return cv.waitKey(1) == ord('q')

#Driver code
#The quadrant variable is set the the quad the marker is in, when the video function is run
#Press 'q' to exit the video mode
# Only runs when not imported
if __name__ == "__main__":
    camera = video_init()
    while True:
        quadrant = video_loop(camera)
        print(quadrant)
        if was_quit_pressed():
            break
    video_deinit(camera)
    print("Done")