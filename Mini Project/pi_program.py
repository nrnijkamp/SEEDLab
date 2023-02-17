#This is the file that where we will make the code for the mini project
#Group 3

from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import numpy as np
import cv2 as cv


def video():
    cap = cv.VideoCapture(0)
    #Infinite loop
    while True:
        ret, frame = cap.read()
        
        if not ret:
            Print("Can't recieve frame...")
            break
        #Make gray and display
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        cv.imshow('frame', gray)
        
        arucoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
        param = cv.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv.aruco.detectMarkers(gray, arucoDict, parameters = param)
        
        if len(corners) >= 1:
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            x_centerPixel = x_sum*.25
            
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
            y_centerPixel = y_sum*.25
            
            #print("X center: ", x_centerPixel)
            #print("Y center: ", y_centerPixel)
            
            if (x_centerPixel > 318) and (y_centerPixel < 237):
                quadrant = 1
                print(quadrant)
            elif (x_centerPixel <= 318) and (y_centerPixel < 237):
                quadrant = 2
                print(quadrant)
            elif (x_centerPixel <= 318) and (y_centerPixel >= 237):
                quadrant = 3
                print(quadrant)
            elif (x_centerPixel > 318) and (y_centerPixel >= 237):
                quadrant = 4
                print(quadrant)
            
            
        else:
            quadrant = 0
            print(quadrant)
        
        if cv.waitKey(1) == ord('q'):
            break
        
    cap.release()
    cv.destroyAllWindows()



#Driver code
#The quadrant variable is set the the quad the marker is in, when the video function is run
#Press 'q' to exit the video mode

quadrant = 0
video()
print("Done")