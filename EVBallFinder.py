#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#
# This is meant to be used in conjuction with WPILib Raspberry Pi image: https://github.com/wpilibsuite/FRCVision-pi-gen
# ----------------------------------------------------------------------------

import os # Needs this for folder manipulation.
import time
import sys
from threading import Thread
import threading
from cscore import CameraServer, VideoSource
import cv2
import numpy as np
import math
import datetime
from datetime import datetime

PictureNumber = 1 #Used for file naming. Everytime it loops it +='s one.

# Time stuff
# datetime object containing current date and time
now = datetime.now()
# dd/mm/YY H:M:S
dt_string = now.strftime("%d-%m-%Y_%H-%M")
drTitle = dt_string

# Image Camera Size (Pixels)
Camera_Image_Width = 320
Camera_Image_Height = 240

# Aspect Ratio
HorizontalAspect = 4
VerticalAspect = 3
DiagonalAspect = math.hypot(HorizontalAspect, VerticalAspect)

# HSV
hsv_threshold_hue = [55, 75]
hsv_threshold_saturation = [89, 231]
hsv_threshold_value = [102, 255]

class WebcamVideoStream:
    def __init__(self, camera, cameraServer, frameWidth, frameHeight, name="WebcamVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream

        # Automatically sets exposure to 0 to track tape
        self.webcam = camera
        # print("SETTING EXPOSURE ")
        # print(self.webcame.exposure)

        # Make a blank image to write on
        self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
        # Gets the video
        self.stream = cameraServer.getVideo(camera=camera)
        (self.timestamp, self.img) = self.stream.grabFrame(self.img)

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            if self.stopped: # if the thread indicator variable is set, stop the thread
                return
            # Boolean logic we don't keep setting exposure over and over to the same value
            '''
            if self.autoExpose:
                self.webcam.setExposureAuto()
                
            else:
                self.webcam.setExposureManual(0)
            '''
            (self.timestamp, self.img) = self.stream.grabFrame(self.img) # gets the image and timestamp from cameraserver

    def read(self): # return the frame most recently read
        return self.timestamp, self.img

    def stop(self): # indicate that the thread should be stopped
        self.stopped = True

    def getError(self):
        return self.stream.getError()

# image size ratio'ed to 16:9
image_width = 640
image_height = 480

# Lists to make the program happy
cameras = []
streams = []
webcam = cameras[0]
cameraServer = streams[0]

cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()

while True:
    fileTitle = (drTitle + "/samplePicture" + str(PictureNumber) + ".jpg") # This makes the files not override each other, by having it named.
    PictureNumber += 1
    timestamp, img = cap.read()
    cv2.imwrite(fileTitle, img) #Makes the picture. Includes directory name as most unix systems will create a directory if it doesnt exist.
    time.sleep(1.5) # This can be changed
