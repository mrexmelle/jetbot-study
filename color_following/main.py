#!/bin/bash

from jetbot import Camera, bgr8_to_jpeg, Robot
from SCSCtrl import TTLServo
import cv2, time
import numpy as np

camera = Camera.instance(width=300, height=300)

# If the TTL servo communicates too frequently,
# there is a certain probability that rx and tx communication errors will be reported.
# This defines a delay for a period of time after each communication to avoid excessive communication frequency.
servoCtrlTime = 0.001

# Turn the No. 1 and No. 5 servos to the initial position.
# servo No. 1 controls the PAN axis movement of the camera pan/tilt, turning it horizontally.
# servo No. 5 controls the TILT axis movement of the camera pan/tilt, and the upward and downward pitching in the numerical direction.
TTLServo.servoAngleCtrl(1, 0, 1, 150)
time.sleep(servoCtrlTime)
TTLServo.servoAngleCtrl(5, 0, 1, 150)
time.sleep(servoCtrlTime)

# Posisi awal
TTLServo.servoAngleCtrl(1, 0, 1, 150)
TTLServo.servoAngleCtrl(2, 0, 1, 150)
TTLServo.servoAngleCtrl(3, 0, 1, 150)
TTLServo.servoAngleCtrl(4, 80, 1, 150)
TTLServo.servoAngleCtrl(5, 0, 1, 150)

# camera looks up.
def cameraUp(speedInput):
    TTLServo.servoAngleCtrl(5, -70, 1, speedInput)
    time.sleep(servoCtrlTime)

# camera looks down.
def cameraDown(speedInput):
    TTLServo.servoAngleCtrl(5, 25, 1, speedInput)
    time.sleep(servoCtrlTime)

# camera looks right.
def ptRight(speedInput):
    TTLServo.servoAngleCtrl(1, 80, 1, speedInput)
    time.sleep(servoCtrlTime)

# camera looks left.
def ptLeft(speedInput):
    TTLServo.servoAngleCtrl(1, -80, 1, speedInput)
    time.sleep(servoCtrlTime)

# camera tilt axis motion stops.
def tiltStop():
    TTLServo.servoStop(5)
    time.sleep(servoCtrlTime)

# camera pan axis motion stops.
def panStop():
    TTLServo.servoStop(1)
    time.sleep(servoCtrlTime)

robot = Robot()

# Define the color that needs to be recognized.

#Yellow #FFFF00
colorUpper = np.array([44, 255, 255])
colorLower = np.array([24, 100, 100])

# Red FF0000
# colorUpper = np.array([180, 255, 255])
# colorLower = np.array([160, 100, 100])

# Green #00FF00
# colorUpper = np.array([50, 255, 255])
# colorLower = np.array([70, 200, 100])

# Blue #0000FF
# colorUpper = np.array([110, 225, 255])
# colorLower = np.array([135, 180, 200])

# Cyan #00FFFF
# colorUpper = np.array([80, 255, 255])
# colorLower = np.array([105, 180, 180])

# Magenta #FF00FF
# colorUpper = np.array([140, 255, 255])
# colorLower = np.array([160, 150, 200])


# Define the position tolerance of the camera when turning to this object.
# The higher the value, the higher the accuracy of the camera when aiming,
# but too high a value may also cause the camera to continuously swing.
error_tor = 25

# This is the P value of the simple PID regulator,
# which is the proportional adjustment coefficient of the motion speed.
# If this value is too high, it will cause the camera PT motion overshoot,
# and if it is too low, it will cause the color tracking response speed to be too slow.
PID_P = 3

# Color recognition and tracking function.
def findColor(imageInput):
    # Convert video frames to HSV color space.
    hsv = cv2.cvtColor(imageInput, cv2.COLOR_BGR2HSV)
    
    # Create a mask for pixels that match the target color.
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    
    # Erode, this process will remove the relatively
    # small area in the mask just selected, which can be understood as denoising.
    mask = cv2.erode(mask, None, iterations=2)
    
    # dilate, the corrosion process just now will cause the large area to become
    # smaller and the small area to disappear. This step is to restore the large area to its previous size.
    mask = cv2.dilate(mask, None, iterations=2)
    
    # Obtain the conformed area contour.
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    
    # If there is a matching area, start to control the movement of the steering gear to achieve color tracking.
    if len(cnts) > 0:
        
        # Draw text to show that the target has been found.
        imageInput = cv2.putText(imageInput,'Target Detected',(10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
        
        # Find the contour of the largest area.
        c = max(cnts, key=cv2.contourArea)
        
        # Get the location of the center point of this area and the radius of this area.
        ((box_x, box_y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        # X, Y are the center points of the area.
        X = int(box_x)
        Y = int(box_y)
        
        # error_X, error_Y are the absolute value of the error
        # between the center point of the area and the center point of the frame.
        error_Y = abs(150 - Y)
        error_X = abs(150 - X)
        
        # Draw the size and position of this area.
        cv2.rectangle(imageInput,(int(box_x-radius),int(box_y+radius)),(int(box_x+radius),int(box_y-radius)),(255,255,255),1)
        
        rad = "Radius =" + str(radius)
        imageInput = cv2.putText(imageInput,str(rad),(10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)

        
        if radius < 25:
            robot.forward(0.5)
        elif radius > 30 :
            robot.backward(0.5)
        else:
            robot.stop()
            return imageInput
        
        
        if Y < 150 - error_tor:
            # Camera looks up.
            imageInput = cv2.putText(imageInput,'Looking Up',(10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
            cameraUp(error_Y*PID_P)
        elif Y > 150 + error_tor:
            # Camera looks down.
            imageInput = cv2.putText(imageInput,'Looking Down',(10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
            cameraDown(error_Y*PID_P)
        else:
            # The error in the vertical direction is less than the tolerance,
            # the camera stops moving in the pitch direction.
            imageInput = cv2.putText(imageInput,'Y Axis Locked',(10,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
            tiltStop()

        if X < 150 - error_tor:
            # Camera looks left.
            imageInput = cv2.putText(imageInput,'Looking Left',(10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
#            ptLeft(error_X*PID_P)
            robot.left(0.5)
        elif X > 150 + error_tor:
            # Camera looks right.
            imageInput = cv2.putText(imageInput,'Looking Right',(10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
#            ptRight(error_X*PID_P)
            robot.right(0.5)
        else:
            # The error in the horizontal direction is less than the tolerance,
            # and the camera stops moving in the horizontal direction.
            imageInput = cv2.putText(imageInput,'X Axis Locked',(10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
            panStop()

    # If no area matching the target color is found, the camera stops rotating.
    else:
        imageInput = cv2.putText(imageInput,'Target Detecting',(10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
        tiltStop()
        panStop()
        robot.stop()

    
    return imageInput

TTLServo.servoAngleCtrl(1, 0, 1, 150)
TTLServo.servoAngleCtrl(2, 0, 1, 150)
TTLServo.servoAngleCtrl(3, 0, 1, 150)
TTLServo.servoAngleCtrl(5, 0, 1, 150)

TTLServo.servoAngleCtrl(1, 0, 1, 150)

def execute(change):
    global image_widget
    image = change['new']
    bgr8_to_jpeg(findColor(image))
    
execute({'new': camera.value})
camera.unobserve_all()
camera.observe(execute, names='value')

# camera.unobserve(execute, names='value')
# time.sleep(1)
# tiltStop()
# panStop()
# camera.stop()
# robot.stop()
