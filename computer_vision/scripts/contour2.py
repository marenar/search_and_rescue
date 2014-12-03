#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
frame = cv2.imread("11.png")
#distancem should be initiated as an arbitrarily high value
distancem = 9001
while(frame != None):
	cv2.imshow("CAM",frame)	

	#Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# GREEN STOP SIGN
	lower = np.array([30,130,120])
	upper = np.array([75,200,210])


	# Threshold the HSV image to get only green colors
	mask = cv2.inRange(hsv, lower, upper)

	contours, heiarchy = cv2.findContours(mask,1,2)
	if(len(contours) > 0):
		contour = max(contours, key=lambda x:cv2.contourArea(x))
		cv2.drawContours(frame,contour,-1,(0,255,0))

		#create the rectangle around the stop sign
		x,y,w,h = cv2.boundingRect(contour)
		box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
		#extract rectangle width
		#draw Rectangle
		cv2.drawContours(frame,[box],0,(0,0,255),2)


		#IMPORTANT!!! THE NEXT 3 LINES ARE FOR TESTING ONLY!!
		#COMMENT OUT FOR CONTEST!!!
		cv2.imshow("CAM",frame)
		cv2.imshow("mask",mask)
		cv2.waitKey(5000)

		distancei = (w - 528.0)/-15.0
		distancem = distancei*0.0254
		print "final width:", w
		print "D inches:", distancei
		print "D meters:", distancem
		if(abs(distancem-0.4318) < 0.0254):
			print "STOP in", distancem, "meters"

