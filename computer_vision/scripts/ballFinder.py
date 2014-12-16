#!/usr/bin/env python

import rospy
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

"""""""""""""""""""""
To do:
Distance measurement - make more accurate
Get diameter of the ball
Use to calculate distance between ball and neato
Get center of ball (coordinates)
Use horizontal coordinate and distance to calculate offset from center (w)
Rate of expansion on the field of view is x2

Color detection - make more robust/get accurate ranges based off of robot


"""""""""""""""""""""

class BallFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		#self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		#rospy.init_node('oodometry', anonymous=True) #make node 
		#rospy.Subscriber('odom',Odometry,Position)
		self.bridge = CvBridge()
		self.image = None
		self.cmd = Twist()
		#self.stop = False
		#self.pose = odom.pose.pose.position
		#self.odom = None

		#self.top_cutoff = .9

		#self.found_lines = np.zeros((480, 640,3), np.uint8)
		#self.image = cv2.imread("StopSign16in.png")

	def update_image(self,msg):
		try:
			self.image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
			pass
		except CvBridgeError, e:
			print e
	
	def on_mouse(self,event,x,y,flag,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			#print self.image[y][x]
			hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
			print hsv[y][x]


	def detectBall(self):
	 	if self.image != None:
	 	 	frame = self.image.copy()
	 	 	cv2.imshow("CAM",frame)	
			# img = cv2.medianBlur(frame,5)
			# #img = self.bridge.cv2_to_imgmsg(img,"bgr8")
			# cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

			# circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,1,50,param1=60,param2=60,minRadius=50,maxRadius=200)
			# circles = np.uint16(np.around(circles))
			# if circles != None:
			# 	for i in circles[0,:]:
			# 		# draw the outer circle
			# 		cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
			# 		# draw the center of the circle
			# 		cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

			# cv2.imshow('detected circles',cimg)
			# cv2.waitKey(2000)

	 		# Convert BGR to HSV
	 		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			# #Red Ball
			# red_lower = np.array([0, 100, 100])
			# red_upper = np.array([10, 255, 255])

			#HSV range {0-180, 0-255, 0-255}

			# #Blue Ball
			# blue_lower = np.array([80, 100, 0])
			# blue_upper = np.array([100, 255, 100])

			# Green Ball - need to adjust ranges

			# green_lower = np.array([60, 100, 20])
			# green_upper = np.array([70, 255, 150])

			# Yellow Ball
			# yellow_lower = np.array([25, 250, 130])
			# yellow_upper = np.array([30, 255, 255])

			# Threshold the HSV image to get only desired colors
			# red_mask = cv2.inRange(hsv, red_lower, red_upper)
			# blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
			# green_mask = cv2.inRange(hsv, green_lower, green_upper)
			# yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
			mask = cv2.inRange(hsv, red_lower, red_upper)

			contours, heiarchy = cv2.findContours(mask,1,2)
			# redContours, redHeiarchy = cv2.findContours(red_mask,1,2)
			# blueContours, blueHeiarchy = cv2.findContours(blue_mask,1,2)
			# greenContours, greenHeiarchy = cv2.findContours(green_mask,1,2)
			# yellowContours, yellowHeiarchy = cv2.findContours(yellow_mask,1,2)

			if (len(contours) > 0):
			# if(len(redContours) > 0 or len(blueContours) > 0 or len(greenContours) > 0 or len(yellowContours)>0):
				# print "found something"
				contour = max(contours, key=lambda x:cv2.contourArea(x))
				cv2.drawContours(frame,contour,-1,(0,255,0))

				# box = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),0)

				#create the rectangle around the stop sign
				x,y,w,h = cv2.boundingRect(contour)
				box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
				#extract rectangle width
				#draw Rectangle
				cv2.drawContours(frame,[box],0,(0,0,255),2)

				cv2.imshow("CAM",frame)
				cv2.imshow("mask",mask)
				cv2.waitKey(500)

				# distancei = (w - 700.0)/-15.0
				# distancem = distancei*0.0254
				# print "final width:", w
				# print "D inches:", distancei
				# print "D meters:", distancem
				#(abs(distancem-0.4318) < 0.0254):
				#	print "STOP in", distancem, "meters"

			elif (len(contours) <= 0):
				print "found nothing"

	def run(self):
		cv2.namedWindow("CAM")
		cv2.setMouseCallback("CAM",self.on_mouse)

		r=rospy.Rate(10)
		while not rospy.is_shutdown():

				self.detectBall()

				cv2.waitKey(50)
				r.sleep()

if __name__ == '__main__':
	lf = BallFinder()
	lf.run()

	
