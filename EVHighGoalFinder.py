#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#
# This is meant to be used in conjuction with WPILib Raspberry Pi image: https://github.com/wpilibsuite/FRCVision-pi-gen
# ----------------------------------------------------------------------------

import os
import json
import time
import sys
from threading import Thread
import socket
import queue
import threading
from cscore import CameraServer, VideoSource
import cv2
import numpy as np
from numpy import mean
import math
import datetime
from datetime import datetime

PictureNumber = 1 #Used for file naming. Everytime it loops it +='s one.

# datetime object containing current date and time
now = datetime.now()
#set path
path = "/year"

# dd/mm/YY H:M:S
drTitle = now.strftime("%d-%m-%Y_%H-%M-%S")
path = drTitle
isFile = os.path.isfile(drTitle)
if isFile is False: # This will never be true as the PI cannot restart in <1 second
    os.mkdir(drTitle)

# Image Camera Size (Pixels)
Camera_Image_Width = 640
Camera_Image_Height = 480

image_width = 640
image_height = 480

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

        # self.webcam.setExposureManual(0)
        # Some booleans so that we don't keep setting exposure over and over to the same value
        # self.autoExpose = False
        # self.prevValue = self.autoExpose
        # Make a blank image to write on
        self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
        # Gets the video
        self.stream = cameraServer.getVideo(camera=camera)
        (self.timestamp, self.img) = self.stream.grabFrame(self.img)

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should
        # be stopped
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
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # Boolean logic we don't keep setting exposure over and over to the same value
            '''
            if self.autoExpose:

                self.webcam.setExposureAuto()
            else:

                self.webcam.setExposureManual(0)
            '''
            # gets the image and timestamp from cameraserver
            (self.timestamp, self.img) = self.stream.grabFrame(self.img)

    def read(self):
        # return the frame most recently read
        return self.timestamp, self.img

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def getError(self):
        return self.stream.getError()

configFile = "/boot/frc.json"


class CameraConfig: pass


team = None
server = False
cameraConfigs = []

def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)


def readCameraConfig(config): 
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]

    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    cam.config = config

    cameraConfigs.append(cam)
    return True



def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True


def startCamera(config):
    print("Starting camera '{}' on {}".format(config.name, config.path))
    cs = CameraServer.getInstance()
    camera = cs.startAutomaticCapture(name=config.name, path=config.path)
    config.config['pixel format'] = 'yuyv'
    print(config.config)
    camera.setConfigJson(json.dumps(config.config))

    return cs, camera

if len(sys.argv) >= 2:
    configFile = sys.argv[1]
# read configuration
if not readConfig():
    sys.exit(1)


# start cameras
cameras = []
streams = []
for cameraConfig in cameraConfigs:
    cs, cameraCapture = startCamera(cameraConfig)
    streams.append(cs)
    cameras.append(cameraCapture)

webcam = cameras[0]
cameraServer = streams[0]
cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()

while True:
    fileTitle = (drTitle + "/highSamplePicture" + str(PictureNumber) + ".jpg") # This makes the files not override each other, by having it named with numbers increasing.
    PictureNumber += 1
    timestamp, img = cap.read()
    cv2.imwrite(fileTitle, img) #Makes the picture. Includes directory name as most unix systems will create a directory if it doesnt exist.
    #print("THE IMAGE WAS SAVED KEK")
    time.sleep(1.5) # This can be changed
