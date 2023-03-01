#Colby Hoggard - UCA Engineering Physics - Robotics 1 Final Project - LIDAR Wall Follower Robot - 12/15/22
#Create a robot which can navigate through 3 checkpoints in a closed course using LIDAR, then follow an ArUco marker to the final point


from math import floor, pi
import numpy as np
import matplotlib.pyplot as plt
from adafruit_rplidar import RPLidar
import RPi.GPIO as GPIO
import time
from gpiozero import Robot, PWMLED, Button
import RPi.GPIO as GPIO
from pynput import keyboard
from picamera import PiCamera
import cv2
from imutils.video import VideoStream
import argparse
import imutils
import sys

#declare LED and buttons
led = PWMLED(16)
button = Button(25)

#class obj inheritnece to add some new functions for forward left and forward right
class BetterRobot(Robot):
    def forward_right(self, speed = 0.5):
        self.left_motor.forward(speed * 0.65)
        self.right_motor.forward(speed)
        
    def forward_left(self, speed=0.5):
        self.left_motor.forward(speed)
        self.right_motor.forward(speed*0.7)
    
#create bot object
bot = BetterRobot(left = (20,21,26), right = (6,13,12))

#code for the aruco dict stuff, not currently used
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
    default="DICT_ARUCO_ORIGINAL",
    help="type of ArUCo tag to detect")
args = vars(ap.parse_args)

#camera = PiCamera()
#camera.start_preview()

#dictionary of aruco types
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

#setting to the marker library im using
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
arucoParams = cv2.aruco.DetectorParameters_create()

#create video streaming object for aruco following
print("[INFO] starting video stream")
vs = VideoStream(src=0).start()
time.sleep(2.0)

# Setup the RPLidar
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(None, PORT_NAME, timeout=3)
scan_data = [0] * 360
mode = 0 #0 is wall follow, 1 is aruco

#begin pause mode
led.pulse()
while True:
    if button.is_pressed:
        led.on()
        break

try:
    while True:
	#get frame, check for aruco, if no aruco, follow wall using lidar, if aruco follow marker using camera
        frame = vs.read()
        frame = imutils.resize(frame, width=1000)
        
        (corners,ids,rejected) = cv2.aruco.detectMarkers(frame,
            arucoDict, parameters=arucoParams)
        #print(corners)
        #print(ids)
        if len(corners) > 0 and mode == 1:
            #print("corners detected")
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                # draw the bounding box of the ArUCo detection
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                print(cX)
                # draw the ArUco marker ID on the frame
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                if cX in range(350,650):
                    bot.forward(.5)
                elif cX in range(0,349):
                    bot.forward_left()
                elif cX in range(651, 1000):
                    bot.forward_right()
                else:
                    bot.stop()
                time.sleep(0.1)
        else:
            bot.stop()
            
        if mode == 0:
            #print("no corners")
            #right()
            #time.sleep(0.65)
            #stop()
            for scan in lidar.iter_scans():
                for (_, angle, distance) in scan:
                    scan_data[min([359, floor(angle)])] = distance
                
                left, front, f_left = scan_data[90], scan_data[170], scan_data[140]
                frame = vs.read()
                frame = imutils.resize(frame, width=1000)
                
                (corners,ids,rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
                #print("corners: ", corners)
                #second variable called emergency front, used if bot hits wall
                e_front = front
                print(e_front)
                if len(corners) > 0:
                    mode = 1
                    bot.stop()
                    break
                if left < 450:
                    left = True
                else:
                    left = False
                
                if front < 1000:
                    front = True
                else:
                    front = False

                if f_left < 750:
                    f_left = True
                else:
                    f_left = False
                    
                if e_front < 200:
                    e_front = True
                else:
                    e_front = False
                    
                if e_front == True:
                    bot.stop()
                    bot.left_motor.backward(0.4)
                    time.sleep(0.05)
                    bot.stop()
                elif front == True  and left == False:
                    bot.forward_left()
                    time.sleep(0.05)
                    bot.forward(0.5)
                    time.sleep(0.05)
                elif left == True and f_left == False:
                    bot.forward_left()
                elif front == False  and left == False:
                    bot.forward_left()
                    time.sleep(0.05)
                elif front == True:
                    bot.left_motor.forward(.1)
                    bot.right_motor.forward(0.5)
                    time.sleep(0.05)
                elif front == False and left == True:
                    bot.forward(0.4)
                else:
                    bot.left_motor.forward(0.1)
                    bot.right_motor.forward(0.5)
                    time.sleep(0.05)
                """
                print("left: ", scan_data[90])
                print("front: ", scan_data[180])
                """
except KeyboardInterrupt:
    print("Stopping.")
lidar.stop()
lidar.disconnect()
