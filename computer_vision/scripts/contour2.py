"""
Code from David Elkan, edits by Michelle Sit

To do: incorporate distance detector - center in screen?
Draw circle around ball?

"""

#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

frame = cv2.imread("greenball1.png")
frame_resize = cv2.resize(frame, (800, 800))
#distancem should be initiated as an arbitrarily high value
distancem = 9001

while(frame != None):
	cv2.imshow("CAM",frame_resize)	

	#Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# #Red Ball - Needs to be improved.  Only getting some of the image.
	# #Preferable to get the majority of the ball.
	# #when it updates, the area gets smaller and smaller
	# red_lower = np.array([0, 100, 100])
	# red_upper = np.array([10, 255, 255])

	# #Blue Ball - having problems with the ranges for some reason
	# blue_lower = np.array([210, 100, 100])
	# blue_upper = np.array([250, 255, 255])

	#Green Ball - having problems with the ranges for some reason
	green_lower = np.array([35, 100, 100])
	green_upper = np.array([90, 255, 255])

	# Threshold the HSV image to get only desired colors
	mask = cv2.inRange(hsv, green_lower, green_upper)
	mask_resize = cv2.resize(mask, (800, 800))

	contours, heiarchy = cv2.findContours(mask,1,2)

	if(len(contours) > 0):
		print "found something"
		contour = max(contours, key=lambda x:cv2.contourArea(x))
		cv2.drawContours(frame,contour,-1,(0,255,0))

		# box = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),0)

		#create the rectangle around the stop sign
		x,y,w,h = cv2.boundingRect(contour)
		box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
		#extract rectangle width
		#draw Rectangle
		cv2.drawContours(frame,[box],0,(0,0,255),2)


		#IMPORTANT!!! THE NEXT 3 LINES ARE FOR TESTING ONLY!!
		#COMMENT OUT FOR CONTEST!!!
		cv2.imshow("CAM",frame_resize)
		cv2.imshow("mask",mask_resize)
		cv2.waitKey(5000)

		# distancei = (w - 528.0)/-15.0
		# distancem = distancei*0.0254
		# print "final width:", w
		# print "D inches:", distancei
		# print "D meters:", distancem
		# if(abs(distancem-0.4318) < 0.0254):
		# 	print "STOP in", distancem, "meters"
	if (len(contours) <= 0):
		print "found nothing"
