#!/usr/bin/env python3
import json, sys, cv2, os, time, math
import numpy as np 
import socket, queue, threading

#CONFIGURATION VARIABLES
#=========================================================
'''
Modes:
	-High Goal
	-Ball
	-Regression
'''
mVersion = "1.0.0.0"
mMode = "Regression"

#Image Camera Size (Pixels)
CameraWidth = 640
CameraHeight = 480
CenterX = (CameraWidth / 2) - .5
CenterY = (CameraHeight/2) - .5
HorizontalAspect = 4
VerticalAspect = 3
DiagonalAspect = math.hypot(HorizontalAspect, VerticalAspect)

#HSV Values
Ball_HSV_Lower = np.array([13, 67, 188])
Ball_HSV_Upper = np.array([62, 255, 255])


#Sample Vision Folder, used for regressions
#expects to be in the same folder
SampleFolder = "VisionSamples"
OutputFolder = "Output"

#=========================================================

# Queue of Packets
# Thread Safe.. Packets being sent to robot are placed here!
PacketQueue = queue.Queue()


#Create directory if it does not exist
def CreateDirectory(directory):
	if not os.path.exists(directory):
		os.makedirs(directory)

def GetTimestamp():
	ts = time.gmtime()
	return str(time.strftime("%Y-%m-%d %H-%M-%S", ts))


if __name__ == "__main__":
	mMode = str(mMode).strip()
	print("Entropy Vision - " + str(mVersion) + " - " + mMode)

	#Conditional Logic based on Regression or Tracking
	if(mMode == "Regression"):
		#Regression Mode
		#Use the same tracking algorithms on our repo


		if(os.path.exists(SampleFolder)):
			#Vision Output Folder Exists

			#Get all the images in input folders
			BallPath = SampleFolder + "/Ball"
			GoalPath = SampleFolder + "/Goal"
			BallImages = [os.path.join(dp, f) for dp, dn, filenames in os.walk(BallPath) for f in filenames if os.path.splitext(f)[1] == '.jpg']
			GoalImages = [os.path.join(dp, f) for dp, dn, filenames in os.walk(GoalPath) for f in filenames if os.path.splitext(f)[1] == '.jpg']

			#create output folders for Ball and goal
			ts = GetTimestamp()
			OutputPath = SampleFolder + "/" + OutputFolder + "/" 
			OutputPathBall = OutputPath + "Ball"
			OutputPathGoal = OutputPath + "Goal"
			CreateDirectory(OutputPathBall)
			CreateDirectory(OutputPathGoal)

			print("Found " + str(len(BallImages)) + " Ball Images")
			print("Found " + str(len(GoalImages)) + " Goal Images")

			#Loop Goal Images
			for GI in GoalImages:
				frame = cv2.imread(GI,0)


		else:
			#Vision Output Folder does not exist
			print("ERROR: Vision Output Folder \"" + SampleFolder + "\" does not exist")

	else:
		#ball or high goal
		
		#get configuration file
		pass


