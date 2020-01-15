#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

# My 2019 license: use it as much as you want. Crediting is recommended because it lets me know that I am being useful.
# Credit to Screaming Chickens 3997

# This is meant to be used in conjuction with WPILib Raspberry Pi image: https://github.com/wpilibsuite/FRCVision-pi-gen
#----------------------------------------------------------------------------


'''
{"controls":[{"default":"1","id":"1","max":"1","min":"0","name":"connect_verbose","step":"1","type":"2","value":"1"},{"default":"0","id":"2","max":"0","min":"0","name":"","step":"0","type":"0","value":""},{"default":"50","id":"3","max":"100","min":"0","name":"raw_brightness","step":"1","type":"2","value":"50"},{"default":"50","id":"4","max":"100","min":"0","name":"brightness","step":"1","type":"2","value":"50"},{"default":"0","id":"5","max":"100","min":"-100","name":"raw_contrast","step":"1","type":"2","value":"0"},{"default":"50","id":"6","max":"100","min":"0","name":"contrast","step":"1","type":"2","value":"50"},{"default":"0","id":"7","max":"100","min":"-100","name":"raw_saturation","step":"1","type":"2","value":"0"},{"default":"50","id":"8","max":"100","min":"0","name":"saturation","step":"1","type":"2","value":"50"},{"default":"1000","id":"9","max":"7999","min":"1","name":"red_balance","step":"1","type":"2","value":"1000"},{"default":"1000","id":"10","max":"7999","min":"1","name":"blue_balance","step":"1","type":"2","value":"1000"},{"default":"0","id":"11","max":"1","min":"0","name":"horizontal_flip","step":"1","type":"1","value":"0"},{"default":"0","id":"12","max":"1","min":"0","name":"vertical_flip","step":"1","type":"1","value":"0"},{"default":"1","id":"13","max":"3","menu":{"0":"Disabled","1":"50 Hz","2":"60 Hz","3":"Auto"},"min":"0","name":"power_line_frequency","step":"1","type":"8","value":"1"},{"default":"0","id":"14","max":"100","min":"-100","name":"raw_sharpness","step":"1","type":"2","value":"0"},{"default":"50","id":"15","max":"100","min":"0","name":"sharpness","step":"1","type":"2","value":"50"},{"default":"0","id":"16","max":"15","menu":{"0":"None","1":"Black & White","2":"Sepia","3":"Negative","4":"Emboss","5":"Sketch","6":"Sky Blue","7":"Grass Green","8":"Skin Whiten","9":"Vivid","10":"Aqua","11":"Art Freeze","12":"Silhouette","13":"Solarization","14":"Antique","15":"Set Cb/Cr"},"min":"0","name":"color_effects","step":"1","type":"8","value":"0"},{"default":"0","id":"17","max":"360","min":"0","name":"rotate","step":"90","type":"2","value":"0"},{"default":"32896","id":"18","max":"65535","min":"0","name":"color_effects_cbcr","step":"1","type":"2","value":"32896"},{"default":"0","id":"19","max":"1","menu":{"0":"Variable Bitrate","1":"Constant Bitrate"},"min":"0","name":"video_bitrate_mode","step":"1","type":"8","value":"0"},{"default":"10000000","id":"20","max":"25000000","min":"25000","name":"video_bitrate","step":"25000","type":"2","value":"10000000"},{"default":"0","id":"21","max":"1","min":"0","name":"repeat_sequence_header","step":"1","type":"1","value":"0"},{"default":"60","id":"22","max":"2147483647","min":"0","name":"h264_i_frame_period","step":"1","type":"2","value":"60"},{"default":"11","id":"23","max":"11","menu":{"0":"1","1":"1b","2":"1.1","3":"1.2","4":"1.3","5":"2","6":"2.1","7":"2.2","8":"3","9":"3.1","10":"3.2","11":"4"},"min":"0","name":"h264_level","step":"1","type":"8","value":"11"},{"default":"4","id":"24","max":"4","menu":{"0":"Baseline","1":"Constrained Baseline","2":"Main","3":"","4":"High"},"min":"0","name":"h264_profile","step":"1","type":"8","value":"4"},{"default":"0","id":"25","max":"3","menu":{"0":"Auto Mode","1":"Manual Mode","2":"","3":""},"min":"0","name":"auto_exposure","step":"1","type":"8","value":"1"},{"default":"1000","id":"26","max":"10000","min":"1","name":"exposure_time_absolute","step":"1","type":"2","value":"6"},{"default":"0","id":"27","max":"1","min":"0","name":"exposure_dynamic_framerate","step":"1","type":"1","value":"0"},{"default":"12","id":"28","max":"24","menu":{"0":"-4000","1":"-3667","2":"-3333","3":"-3000","4":"-2667","5":"-2333","6":"-2000","7":"-1667","8":"-1333","9":"-1000","10":"-667","11":"-333","12":"0","13":"333","14":"667","15":"1000","16":"1333","17":"1667","18":"2000","19":"2333","20":"2667","21":"3000","22":"3333","23":"3667","24":"4000"},"min":"0","name":"auto_exposure_bias","step":"1","type":"8","value":"12"},{"default":"1","id":"29","max":"10","menu":{"0":"Manual","1":"Auto","2":"Incandescent","3":"Fluorescent","4":"Fluorescent H","5":"Horizon","6":"Daylight","7":"Flash","8":"Cloudy","9":"Shade","10":"Greyworld"},"min":"0","name":"white_balance_auto_preset","step":"1","type":"8","value":"1"},{"default":"0","id":"30","max":"1","min":"0","name":"image_stabilization","step":"1","type":"1","value":"0"},{"default":"0","id":"31","max":"4","menu":{"0":"0","1":"100000","2":"200000","3":"400000","4":"800000"},"min":"0","name":"iso_sensitivity","step":"1","type":"8","value":"0"},{"default":"1","id":"32","max":"1","menu":{"0":"Manual","1":"Auto"},"min":"0","name":"iso_sensitivity_auto","step":"1","type":"8","value":"1"},{"default":"0","id":"33","max":"2","menu":{"0":"Average","1":"Center Weighted","2":"Spot"},"min":"0","name":"exposure_metering_mode","step":"1","type":"8","value":"0"},{"default":"0","id":"34","max":"13","menu":{"0":"None","1":"","2":"","3":"","4":"","5":"","6":"","7":"","8":"Night","9":"","10":"","11":"Sports","12":"","13":""},"min":"0","name":"scene_mode","step":"1","type":"8","value":"0"},{"default":"30","id":"35","max":"100","min":"1","name":"compression_quality","step":"1","type":"2","value":"30"},{"default":"0","id":"36","max":"100","min":"0","name":"exposure_auto","step":"1","type":"2","value":"0"},{"default":"0","id":"37","max":"100","min":"0","name":"exposure_absolute","step":"1","type":"2","value":"0"}],"modes":[]}


'''

import json
import time
import sys
from threading import Thread
import socket
import queue
import threading
from cscore import CameraServer, VideoSource
from networktables import NetworkTablesInstance
import cv2
import numpy as np
from networktables import NetworkTables
import math
########### SET RESOLUTION TO 256x144 !!!! ############


#Image Camera Size (Pixels)
Camera_Image_Width = 320
Camera_Image_Height = 240

centerX = (Camera_Image_Width / 2) - .5


#Aspect Ratio
HorizontalAspect = 4
VerticalAspect = 3
DiagonalAspect = math.hypot(HorizontalAspect, VerticalAspect)

#Upper and Lower HSV Threshold Limits
Tape_HSV_Lower = np.array([57, 115, 85]) #Hue, Saturation, Value
Tape_HSV_Upper = np.array([108, 255, 255]) #Hue, Saturation, Value

#Ball HSV Values
Ball_HSV_Lower = np.array([13,67,188])
Ball_HSV_Upper = np.array([62,255,255])

#ratio values - detects feeder station. Doing and not when doing ratio checks will ignore them
rat_low = 1.5
rat_high = 10

hsv_threshold_hue = [13, 62]
hsv_threshold_saturation = [55, 255]
hsv_threshold_value = [87, 255]

# import the necessary packages
import datetime

#Queue of Packets
#Thread Safe.. Packets being sent to robot are placed here!
PacketQueue = queue.Queue()


#Creates a socket
class SocketWorker(threading.Thread):
	def __init__(self, q, *args, **kwargs):
		self.queue = q
		super().__init__(*args, **kwargs)

		#Initialize Socket Connect
		SocketHost = "10.1.38.2"
		SocketPort = 5800
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.connect((SocketHost, SocketPort))



	def run(self):
		while True:
			try:
				packet = self.queue.get()
				#Convert Packet to JSON
				#Send Message Over Socket
				try:
					data_string = json.dumps(packet)
					self.sock.sendall(data_string.encode())
				except Exception as e:
					print("Socket Exception " + str(e))
			except Exception as e1:
				pass



#Class to examine Frames per second of camera stream. Currently not used.
class FPS:
	def __init__(self):
		# store the start time, end time, and total number of frames
		# that were examined between the start and end intervals
		self._start = None
		self._end = None
		self._numFrames = 0

	def start(self):
		# start the timer
		self._start = datetime.datetime.now()
		return self

	def stop(self):
		# stop the timer
		self._end = datetime.datetime.now()

	def update(self):
		# increment the total number of frames examined during the
		# start and end intervals
		self._numFrames += 1

	def elapsed(self):
		# return the total number of seconds between the start and
		# end interval
		return (self._end - self._start).total_seconds()

	def fps(self):
		# compute the (approximate) frames per second
		return self._numFrames / self.elapsed()


# class that runs separate thread for showing video,
class VideoShow:
	"""
	Class that continuously shows a frame using a dedicated thread.
	"""

	def __init__(self, imgWidth, imgHeight, cameraServer, frame=None, name='stream'):
		self.outputStream = cameraServer.putVideo(name, imgWidth, imgHeight)
		self.frame = frame
		self.stopped = False

	def start(self):
		Thread(target=self.show, args=()).start()
		return self

	def show(self):
		while not self.stopped:
			self.outputStream.putFrame(self.frame)

	def stop(self):
		self.stopped = True

	def notifyError(self, error):
		self.outputStream.notifyError(error)

class WebcamVideoStream:
	def __init__(self, camera, cameraServer, frameWidth, frameHeight, name="WebcamVideoStream"):
		# initialize the video camera stream and read the first frame
		# from the stream

		#Automatically sets exposure to 0 to track tape
		self.webcam = camera
		#print("SETTING EXPOSURE ")
		#print(self.webcame.exposure)

		#self.webcam.setExposureManual(0)
		#Some booleans so that we don't keep setting exposure over and over to the same value
		#self.autoExpose = False
		#self.prevValue = self.autoExpose
		#Make a blank image to write on
		self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
		#Gets the video
		self.stream = cameraServer.getVideo(camera = camera)
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
			#Boolean logic we don't keep setting exposure over and over to the same value
			'''
			if self.autoExpose:

				self.webcam.setExposureAuto()
			else:

				self.webcam.setExposureManual(0)
			'''
			#gets the image and timestamp from cameraserver
			(self.timestamp, self.img) = self.stream.grabFrame(self.img)

	def read(self):
		# return the frame most recently read
		return self.timestamp, self.img

	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True
	def getError(self):
		return self.stream.getError()
###################### PROCESSING OPENCV ################################

#Angles in radians

#image size ratioed to 16:9
image_width = 640
image_height = 480

#Lifecam 3000 from datasheet
#Datasheet: https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf
diagonalView = math.radians(75)

#16:9 aspect ratio
horizontalAspect = 4
verticalAspect = 3

#Reasons for using diagonal aspect is to calculate horizontal field of view.
diagonalAspect = math.hypot(horizontalAspect, verticalAspect)
#Calculations: http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
horizontalView = math.atan(math.tan(diagonalView/2) * (horizontalAspect / diagonalAspect)) * 2
verticalView = math.atan(math.tan(diagonalView/2) * (verticalAspect / diagonalAspect)) * 2

#Focal Length calculations: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_165
H_FOCAL_LENGTH = image_width / (2*math.tan((horizontalView/2)))
V_FOCAL_LENGTH = image_height / (2*math.tan((verticalView/2)))
#blurs have to be odd
green_blur = 0
orange_blur = 27

# define range of green of retroreflective tape in HSV


#lower_green = np.array([0,220,25])
#upper_green = np.array([101, 255, 255])

lower_green = np.array([65, 36.131, 83])
upper_green = np.array([98, 255, 195])

#define range of orange from cargo ball in HSV
lower_orange = np.array([0,193,92])
upper_orange = np.array([23, 255, 255])

#Flip image if camera mounted upside down
def flipImage(frame):
	return cv2.flip( frame, -1 )

#Blurs frame
def blurImg(frame, blur_radius):
	img = frame.copy()
	blur = cv2.blur(img,(blur_radius,blur_radius))
	return blur

# Masks the video based on a range of hsv colors
# Takes in a frame, range of color, and a blurred frame, returns a masked frame
def threshold_video(lower_color, upper_color, blur):
	global hsv_threshold_hue
	global hsv_threshold_saturation
	global hsv_threshold_value

	'''
	# Convert BGR to HSV
	hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)


	print(type(lower_color))
	print(type(upper_color))
	# hold the HSV image to get only red colors
	mask = cv2.inRange(hsv, lower_color, upper_color)
	'''

	out = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(out, (hsv_threshold_hue[0], hsv_threshold_saturation[0], hsv_threshold_value[0]),  (hsv_threshold_hue[1], hsv_threshold_saturation[1], hsv_threshold_value[1]))

	# Returns the masked imageBlurs video to smooth out image

	return mask



# Finds the tape targets from the masked image and displays them on original stream + network tales
def findTargets(frame, mask):

	#print("FIND TARGETS")
	# Finds contours
	_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
	# Take each frame
	# Gets the shape of video
	screenHeight, screenWidth, _ = frame.shape
	# Gets center of height and width
	centerX = (screenWidth / 2) - .5
	centerY = (screenHeight / 2) - .5
	# Copies frame and stores it in image
	image = frame.copy()
	# Processes the contours, takes in (contours, output_image, (centerOfImage)


	if len(contours) != 0:
		image = findTape(contours, image, centerX, centerY)
	else:
		# pushes that it deosn't see vision target to network tables
		networkTable.putBoolean("tapeDetected", False)


	# Shows the contours overlayed on the original video
	return image

# Finds the balls from the masked image and displays them on original stream + network tables
def findCargo(frame, mask):
	# Finds contours
	_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
	# Take each frame
	# Gets the shape of video
	screenHeight, screenWidth, _ = frame.shape
	# Gets center of height and width
	centerX = (screenWidth / 2) - .5
	centerY = (screenHeight / 2) - .5
	# Copies frame and stores it in image
	image = frame.copy()
	# Processes the contours, takes in (contours, output_image, (centerOfImage)
	if len(contours) != 0:
		image = findBall(contours, image, centerX, centerY)
	else:
		# pushes that it doesn't see cargo to network tables
		networkTable.putBoolean("cargoDetected", False)
	# Shows the contours overlayed on the original video
	return image


# Draws Contours and finds center and yaw of orange ball
# centerX is center x coordinate of image
# centerY is center y coordinate of image
def findBall(contours, image, centerX, centerY):
	screenHeight, screenWidth, channels = image.shape;
	#Seen vision targets (correct angle, adjacent to each other)
	cargo = []

	if len(contours) > 0:
		#Sort contours by area size (biggest to smallest)
		cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

		biggestCargo = []
		for cnt in cntsSorted:
			x, y, w, h = cv2.boundingRect(cnt)
			aspect_ratio = float(w) / h
			# Get moments of contour; mainly for centroid
			M = cv2.moments(cnt)
			# Get convex hull (bounding polygon on contour)
			hull = cv2.convexHull(cnt)
			# Calculate Contour area
			cntArea = cv2.contourArea(cnt)
			# Filters contours based off of size
			if (checkBall(cntArea, aspect_ratio)):
				### MOSTLY DRAWING CODE, BUT CALCULATES IMPORTANT INFO ###
				# Gets the centeroids of contour
				if M["m00"] != 0:
					cx = int(M["m10"] / M["m00"])
					cy = int(M["m01"] / M["m00"])
				else:
					cx, cy = 0, 0
				if(len(biggestCargo) < 3):


					##### DRAWS CONTOUR######
					# Gets rotated bounding rectangle of contour
					rect = cv2.minAreaRect(cnt)
					# Creates box around that rectangle
					box = cv2.boxPoints(rect)
					# Not exactly sure
					box = np.int0(box)
					# Draws rotated rectangle
					cv2.drawContours(image, [box], 0, (23, 184, 80), 3)

					# Draws a vertical white line passing through center of contour
					cv2.line(image, (cx, screenHeight), (cx, 0), (255, 255, 255))
					# Draws a white circle at center of contour
					cv2.circle(image, (cx, cy), 6, (255, 255, 255))

					# Draws the contours
					cv2.drawContours(image, [cnt], 0, (23, 184, 80), 1)

					# Gets the (x, y) and radius of the enclosing circle of contour
					(x, y), radius = cv2.minEnclosingCircle(cnt)
					# Rounds center of enclosing circle
					center = (int(x), int(y))
					# Rounds radius of enclosning circle
					radius = int(radius)
					# Makes bounding rectangle of contour
					rx, ry, rw, rh = cv2.boundingRect(cnt)

					# Draws countour of bounding rectangle and enclosing circle in green
					cv2.rectangle(image, (rx, ry), (rx + rw, ry + rh), (23, 184, 80), 1)

					cv2.circle(image, center, radius, (23, 184, 80), 1)

					# Appends important info to array
					if not biggestCargo:
						biggestCargo.append([cx, cy])
					elif [cx, cy, cnt] not in biggestCargo:
						biggestCargo.append([cx, cy])




		# Check if there are cargo seen
		if (len(biggestCargo) > 0):
			#pushes that it sees cargo to network tables
			networkTable.putBoolean("cargoDetected", True)

			# Sorts targets based on x coords to break any angle tie
			biggestCargo.sort(key=lambda x: math.fabs(x[0]))
			closestCargo = min(biggestCargo, key=lambda x: (math.fabs(x[0] - centerX)))
			xCoord = closestCargo[0]
			finalTarget = calculateYaw(xCoord, centerX, H_FOCAL_LENGTH)
			print("Yaw: " + str(finalTarget))
			# Puts the yaw on screen
			# Draws yaw of target + line where center of target is
			cv2.putText(image, "Yaw: " + str(finalTarget), (40, 40), cv2.FONT_HERSHEY_COMPLEX, .6,
						(255, 255, 255))
			cv2.line(image, (int(xCoord), screenHeight), (int(xCoord), 0), (255, 0, 0), 2)

			currentAngleError = finalTarget
			#pushes cargo angle to network tables
			networkTable.putNumber("cargoYaw", currentAngleError)

		else:
			#pushes that it doesn't see cargo to network tables
			networkTable.putBoolean("cargoDetected", False)

		cv2.line(image, (int(centerX), screenHeight), (int(centerX), 0), (255, 255, 255), 2)

		return image

# Draws Contours and finds center and yaw of vision targets
# centerX is center x coordinate of image
# centerY is center y coordinate of image
def findTape(contours, image, centerX, centerY):
	screenHeight, screenWidth, channels = image.shape;
	#Seen vision targets (correct angle, adjacent to each other)
	targets = []
	if len(contours) >= 2:
		#Sort contours by area size (biggest to smallest)
		cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
		biggestCnts = []
		for cnt in cntsSorted:
			# Get moments of contour; mainly for centroid
			M = cv2.moments(cnt)
			# Get convex hull (bounding polygon on contour)
			hull = cv2.convexHull(cnt)
			# Calculate Contour area
			cntArea = cv2.contourArea(cnt)
			# calculate area of convex hull
			hullArea = cv2.contourArea(hull)
			
			x, y, w, h = cv2.boundingRect(cnt)
			ratio = float(w) / h	
			# Filters contours based off of size
			if (checkContours(cntArea, hullArea, ratio)):
       			#Next three lines are for debugging the contouring
                #contimage = cv2.drawContours(image, cnt, -1, (0, 255, 0), 3) 
				#cv2.imwrite("drawncontours.jpg", contimage)
				#time.sleep(1)
				### MOSTLY DRAWING CODE, BUT CALCULATES IMPORTANT INFO ###
				# Gets the centeroids of contour
				if M["m00"] != 0:
					cx = int(M["m10"] / M["m00"])
					cy = int(M["m01"] / M["m00"])
					distCY = 540-cy
					print(calculateDistance138(distCY))
					print(calculateDistanceMethod2(w))
				else:
					cx, cy = 0, 0
				if(len(biggestCnts) < 13):
					#### CALCULATES ROTATION OF CONTOUR BY FITTING ELLIPSE ##########
					rotation = getEllipseRotation(image, cnt)

					# Calculates yaw of contour (horizontal position in degrees)
					yaw = calculateYaw(cx, centerX, H_FOCAL_LENGTH)
					# Calculates yaw of contour (horizontal position in degrees)
					pitch = calculatePitch(cy, centerY, V_FOCAL_LENGTH)

					##### DRAWS CONTOUR######
					# Gets rotated bounding rectangle of contour
					rect = cv2.minAreaRect(cnt)
					# Creates box around that rectangle
					box = cv2.boxPoints(rect)
					# Not exactly sure
					box = np.int0(box)
					# Draws rotated rectangle
					cv2.drawContours(image, [box], 0, (23, 184, 80), 3)


					# Calculates yaw of contour (horizontal position in degrees)
					yaw = calculateYaw(cx, centerX, H_FOCAL_LENGTH)
					# Calculates yaw of contour (horizontal position in degrees)
					pitch = calculatePitch(cy, centerY, V_FOCAL_LENGTH)


					# Draws a vertical white line passing through center of contour
					cv2.line(image, (cx, screenHeight), (cx, 0), (255, 255, 255))
					# Draws a white circle at center of contour
					cv2.circle(image, (cx, cy), 6, (255, 255, 255))

					# Draws the contours
					cv2.drawContours(image, [cnt], 0, (23, 184, 80), 1)

					# Gets the (x, y) and radius of the enclosing circle of contour
					(x, y), radius = cv2.minEnclosingCircle(cnt)
					# Rounds center of enclosing circle
					center = (int(x), int(y))
					# Rounds radius of enclosning circle
					radius = int(radius)
					# Makes bounding rectangle of contour
					rx, ry, rw, rh = cv2.boundingRect(cnt)
					boundingRect = cv2.boundingRect(cnt)
					# Draws countour of bounding rectangle and enclosing circle in green
					cv2.rectangle(image, (rx, ry), (rx + rw, ry + rh), (23, 184, 80), 1)

					cv2.circle(image, center, radius, (23, 184, 80), 1)

					# Appends important info to array
					if not biggestCnts:
						 biggestCnts.append([cx, cy, rotation])
					elif [cx, cy, rotation] not in biggestCnts:
						 biggestCnts.append([cx, cy, rotation])


		# Sorts array based on coordinates (leftmost to rightmost) to make sure contours are adjacent
		biggestCnts = sorted(biggestCnts, key=lambda x: x[0])
		# Target Checking
		for i in range(len(biggestCnts) - 1):
			#Rotation of two adjacent contours
			tilt1 = biggestCnts[i][2]
			tilt2 = biggestCnts[i + 1][2]

			#x coords of contours
			cx1 = biggestCnts[i][0]
			cx2 = biggestCnts[i + 1][0]

			cy1 = biggestCnts[i][1]
			cy2 = biggestCnts[i + 1][1]
			# If contour angles are opposite
			if (np.sign(tilt1) != np.sign(tilt2)):
				centerOfTarget = math.floor((cx1 + cx2) / 2)
				#ellipse negative tilt means rotated to right
				#Note: if using rotated rect (min area rectangle)
				#      negative tilt means rotated to left
				# If left contour rotation is tilted to the left then skip iteration
				if (tilt1 > 0):
					if (cx1 < cx2):
						continue
				# If left contour rotation is tilted to the left then skip iteration
				if (tilt2 > 0):
					if (cx2 < cx1):
						continue
				#Angle from center of camera to target (what you should pass into gyro)
				yawToTarget = calculateYaw(centerOfTarget, centerX, H_FOCAL_LENGTH)
				#Make sure no duplicates, then append
				if not targets:
					targets.append([centerOfTarget, yawToTarget])
				elif [centerOfTarget, yawToTarget] not in targets:
					targets.append([centerOfTarget, yawToTarget])
	#Check if there are targets seen
	if (len(targets) > 0):
		# pushes that it sees vision target to network tables
		networkTable.putBoolean("tapeDetected", True)
		#Sorts targets based on x coords to break any angle tie
		targets.sort(key=lambda x: math.fabs(x[0]))
		finalTarget = min(targets, key=lambda x: math.fabs(x[1]))
		# Puts the yaw on screen
		#Draws yaw of target + line where center of target is
		cv2.putText(image, "Yaw: " + str(finalTarget[1]), (40, 40), cv2.FONT_HERSHEY_COMPLEX, .6,
					(255, 255, 255))
		cv2.line(image, (finalTarget[0], screenHeight), (finalTarget[0], 0), (255, 0, 0), 2)

		currentAngleError = finalTarget[1]
		# pushes vision target angle to network tables
		networkTable.putNumber("tapeYaw", currentAngleError)

		#print("TapeYaw: " + str(currentAngleError))
	else:
		# pushes that it deosn't see vision target to network tables
		networkTable.putBoolean("tapeDetected", False)

	cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), (255, 255, 255), 2)

	#cv2.imwrite("latest.jpg", image);
	return image



# Checks if tape contours are worthy based off of contour area and (not currently) hull area
def checkContours(cntSize, hullSize, aspRatio):
	return cntSize > (image_width / 6) and not (aspRatio < rat_low or aspRatio > rat_high)

# Checks if ball contours are worthy based off of contour area and (not currently) hull area
def checkBall(cntSize, cntAspectRatio):
	return (cntSize > (image_width / 2)) and (round(cntAspectRatio) == 1)

#Forgot how exactly it works, but it works!
def translateRotation(rotation, width, height):
	if (width < height):
		rotation = -1 * (rotation - 90)
	if (rotation > 90):
		rotation = -1 * (rotation - 180)
	rotation *= -1
	return round(rotation)


#distance = (targetHeightInches * ImageWidthPixels) / (2 * targetHeightPixels * tan(cameraHorizontalAngle/2);
def calculateDistance138(targetHeightPixels):
    #Current res is 640x480, up to 75FPS or 320x240 up to 187 FPS both (4:3)
    #FOV is 53.5
    targHeightInch = 27.5
    imageWidthPixels = 480
    camerafov = math.tan(75/2)
    myDist = (targHeightInch*imageWidthPixels)/(2*targetHeightPixels*camerafov)
    return myDist

def calculateDistanceMethod2(targPixelWidth):
	#d = Tft*FOVpixel/(2*Tpixel*tanΘ)

	FOV = 75
	#8 feet, 2.25 inches, actual height of center goal is 96.25, I think the centroid of the tape is ~87.75 inches
	#tape is 1 ft 5inches, 17 inches/2 = 8.5 inches. 96.25-8.5 gives 87.75
	targetHeightActual = 96.25
	camPixelWidth = 480
	#target reflective tape width in feet (3 feet, 3 & 1/4 inch) ~3.27
	Tft = 3.27
	#theta = 1/2 FOV,
	tanFOV = math.tan(FOV/2)
 
	distEst = Tft*camPixelWidth/(2*targPixelWidth*tanFOV)
	return(distEst) 


def calculateDistance(heightOfCamera, heightOfTarget, pitch):
	heightOfTargetFromCamera = heightOfTarget - heightOfCamera

	# Uses trig and pitch to find distance to target
	'''
	d = distance
	h = height between camera and target
	a = angle = pitch
	tan a = h/d (opposite over adjacent)
	d = h / tan a
						 .
						/|
					   / |
					  /  |h
					 /a  |
			  camera -----
					   d
	'''
	distance = math.fabs(heightOfTargetFromCamera / math.tan(math.radians(pitch)))

	return distance


# Uses trig and focal length of camera to find yaw.
# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculateYaw(pixelX, centerX, hFocalLength):
	yaw = math.degrees(math.atan((pixelX - centerX) / hFocalLength))
	return round(yaw)



#Grabs Countours Based on Version
def grab_contours(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]

    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]

    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
            "otherwise OpenCV changed their cv2.findContours return "
            "signature yet again. Refer to OpenCV's documentation "
            "in that case"))

    # return the actual contours array
    return cnts


#Filter out the Tape HSV
def FilterHSVTape(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return cv2.inRange(frame,Tape_HSV_Lower, Tape_HSV_Upper)

#Perform a Mask on the ball
#use a range of colors around the ball color to account for lighting
def MaskBall(frame):
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, Ball_HSV_Lower, Ball_HSV_Upper)
    #remove any small blobs left in the mask
    #mask = cv2.erode(mask, None, iterations=2)
    #mask = cv2.dilate(mask, None, iterations=2)
    return mask

#Find Tape Targets
def findTapeTargets(frame):
    #Find Contours
    Contours, Hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    screenHeight, screenWidth = frame.shape
    # Gets center of height and width
    centerX = (screenWidth / 2) - .5
    centerY = (screenHeight / 2) - .5

    #GProceed if we have contours
    if(len(Contours) > 0):
        for Countour in Contours:
            # Get moments of contour; mainly for centroid
            M = cv2.moments(Countour)
            # Get convex hull (bounding polygon on contour)
            hull = cv2.convexHull(Countour)
            # Calculate Contour area
            CountourArea = cv2.contourArea(Countour)
            # calculate area of convex hull
            hullArea = cv2.contourArea(hull)


def CheckBall(CntSize, CntAspectRatio):
    return (CntSize > (Camera_Image_Width / 2)) and (round(CntAspectRatio) == 1)


#For Finding Power Cells
def findBalls(frame):
	FoundBalls = [] #Store and Return Tracked Balls
	ContourList = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	ContourList = grab_contours(ContourList)

	#Proceed if we have contours
	if len(ContourList) > 0:
		#Sort Contours by Area Size (Largest -> Smallest)
		SortedContours = sorted(ContourList, key=lambda x: cv2.contourArea(x), reverse=True)


		#Goal is to return the largest ball
		FoundBall = None

        #Loop through Contour, Large to Smallest in Area
		for c in SortedContours:
			((x, y), radius) = cv2.minEnclosingCircle(c)
			x_br, y_br, w_br, h_br = cv2.boundingRect(c)
			#Image Moment is a particular weighted average of image pixel intensities
			#We can use it to find the center
			M = cv2.moments(c)
			center = 0
			centerX = 0
			try:
				cy = int(M["m01"] / M["m00"])
				cx = int(M["m10"] / M["m00"])
				center = (cx, cy)
			except:
				continue


			CntArea = cv2.contourArea(c)

			#Ignore small contours
			if CntArea < 150:
				continue


			cnt_aspect_ratio = float(w_br) / h_br
			AspectRatioCheck = (round(cnt_aspect_ratio) == 1)



			OkayBall = AspectRatioCheck
			if OkayBall == True:
				#This is a Target!
				#		Lets calculate now!
				ball = {}
				ball['Type'] = 1
				ball['CX'] = cx
				ball['CY'] = cy

				finalTarget = calculateYaw(cx, centerX, H_FOCAL_LENGTH)
				ball['Yaw'] = finalTarget
				ball['Size'] = CntArea

				#Set our Found Ball
				if FoundBall == None:
					FoundBall = ball
				elif FoundBall['Size'] < ball['Size']:
					FoundBall = ball
				print("Tracking Ball: " + str(ball['Yaw']))

		#Check if we have a found ball for this frame
		if not FoundBall == None:
			#print("Found Ball!")
			#print(FoundBall)(
			PacketQueue.put_nowait(FoundBall)

	return FoundBalls




#Proccesses each frame of the image
#boolean to whether we want ball tracking or tape tracking
def ProcessFrame(frame, tape):
	if(tape == True):
        #Tape Process!
        #APPLY A BLUE TO BLUR THE LINES
		frame = FilterHSVTape(frame) #Filter Frame for HSV Tape
		frame = findTapeTargets(frame)
	else:
        #Ball Tracker!
		original_frame = frame.copy()
		frame = MaskBall(frame) #Filter and Mask by HSV Values
		Targets = findBalls(frame)


		'''
		#Debug Drawing Code
		for Targ in Targets:
			original_frame = cv2.circle(original_frame, (Targ['x'], Targ['y']), Targ['radius'], (255, 0, 0), 2)
			original_frame = cv2.circle(original_frame, Targ['center'], 4, (0, 0, 255), -1)

			font = cv2.FONT_HERSHEY_SIMPLEX
			# org
			org = (Targ['x'], Targ['y'])
			# fontScale
			fontScale = .5
			# Blue color in BGR
			color = (255, 0, 0)
			# Line thickness of 2 px
			thickness = 2
			# Using cv2.putText() method
			original_frame = cv2.putText(original_frame, str(Targ['aspectratio']), org, font, fontScale, color, thickness, cv2.LINE_AA)

		cv2.imwrite("of1" + str(val) +  ".jpg", original_frame)
		'''




		return original_frame

# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculatePitch(pixelY, centerY, vFocalLength):
	pitch = math.degrees(math.atan((pixelY - centerY) / vFocalLength))
	# Just stopped working have to do this:
	pitch *= -1
	return round(pitch)

def getEllipseRotation(image, cnt):
	try:
		# Gets rotated bounding ellipse of contour
		ellipse = cv2.fitEllipse(cnt)
		centerE = ellipse[0]
		# Gets rotation of ellipse; same as rotation of contour
		rotation = ellipse[2]
		# Gets width and height of rotated ellipse
		widthE = ellipse[1][0]
		heightE = ellipse[1][1]
		# Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
		rotation = translateRotation(rotation, widthE, heightE)

		cv2.ellipse(image, ellipse, (23, 184, 80), 3)
		return rotation
	except:
		# Gets rotated bounding rectangle of contour
		rect = cv2.minAreaRect(cnt)
		# Creates box around that rectangle
		box = cv2.boxPoints(rect)
		# Not exactly sure
		box = np.int0(box)
		# Gets center of rotated rectangle
		center = rect[0]
		# Gets rotation of rectangle; same as rotation of contour
		rotation = rect[2]
		# Gets width and height of rotated rectangle
		width = rect[1][0]
		height = rect[1][1]
		# Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
		rotation = translateRotation(rotation, width, height)
		return rotation

#################### FRC VISION PI Image Specific #############
configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []

"""Report parse error."""
def parseError(str):
	print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
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

"""Read configuration file."""
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

"""Start running the camera."""
def startCamera(config):
	print("Starting camera '{}' on {}".format(config.name, config.path))
	cs = CameraServer.getInstance()
	camera = cs.startAutomaticCapture(name=config.name, path=config.path)
	config.config['pixel format'] = 'yuyv'
	print(config.config)
	camera.setConfigJson(json.dumps(config.config))

	return cs, camera

if __name__ == "__main__":
	if len(sys.argv) >= 2:
		configFile = sys.argv[1]
	# read configuration
	if not readConfig():
		sys.exit(1)

	# start NetworkTables
	ntinst = NetworkTablesInstance.getDefault()
	#Name of network table - this is how it communicates with robot. IMPORTANT
	networkTable = NetworkTables.getTable('SmartDashboard')

	if server:
		print("Setting up NetworkTables server")
		ntinst.startServer()
	else:
		print("Setting up NetworkTables client for team {}".format(team))
		ntinst.startClientTeam(team)


	# start cameras
	cameras = []
	streams = []
	for cameraConfig in cameraConfigs:
		cs, cameraCapture = startCamera(cameraConfig)
		streams.append(cs)
		cameras.append(cameraCapture)

	webcam = cameras[0]
	cameraServer = streams[0]
	#Start thread reading camera
	cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()

	#Create Socket Thread and Pass the Socket with Packet Queue
	SocketThread = SocketWorker(PacketQueue).start()

	# (optional) Setup a CvSource. This will send images back to the Dashboard
	# Allocating new images is very expensive, always try to preallocate
	img = np.zeros(shape=(image_height, image_width, 3), dtype=np.uint8)
	#Start thread outputing stream
	streamViewer = VideoShow(image_width,image_height, cameraServer, frame=img, name="SmartDashboard").start()
	#cap.autoExpose=True;
	tape = False
	fps = FPS().start()
	#TOTAL_FRAMES = 200;
	# loop forever
	while True:
		# Tell the CvSink to grab a frame from the camera and put it
		# in the source image.  If there is an error notify the output.
		timestamp, img = cap.read()

		Tape = False
		frame = ProcessFrame(img, Tape)


	#Doesn't do anything at the moment. You can easily get this working by indenting these three lines
	# and setting while loop to: while fps._numFrames < TOTAL_FRAMES
	fps.stop()
	print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
	print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
