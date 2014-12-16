#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from copy import deepcopy

"""""""""""""""""""""
To do:
Distance measurement - make more accurate
Get diameter of the ball
Use to calculate distance between ball and neato
Get center of ball (coordinates)
Use horizontal coordinate and distance to calculate offset from center (w)
Rate of expansion on the field of view is x2


"""""""""""""""""""""

class BallFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.init_node('register')
		markerPublisher = rospy.Publisher('visualization_marker_tutorials', MarkerArray)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		rospy.pose_sub = rospy.Subscriber('slam_out_pose')
		markerArray = MarkerArray()
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

	#update_rviz publishes the markers where the balls have been found.
	#it expects to recieve the balls coordinates as a list: [coordinate_x, coordinate_y]
	#also expects to be told the ball colour as a lower-case string ie: "red"
	def update_rviz(ball_coordinates, ball_colour):
		marker = Marker()
		marker.id = len(markerArray)
		marker.header.frame_id = "/map"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = .2
		marker.scale.y = .2
		marker.scale.z = .2
		marker.color.a = 1.0
		if(ball_colour == "red"):
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
		elif(ball_colour == "blue"):
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
		elif(ball_colour == "green"):
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 1.0
		elif(ball_colour == "yellow")
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 0.0
		else:
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = ball_coordinates[0]
		marker.pose.position.y = ball_coordinates[1]
		marker.pose.position.z = .1
		markerArray.markers.append(marker)
		publisher.publish(markerArray)

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

	 	 	""""HoughCircles detection"""""
	 	 	cv2.imshow("CAM",frame)	
			img = cv2.medianBlur(frame,5)
			#img = self.bridge.cv2_to_imgmsg(img,"bgr8")
			cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

			circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,1,50,param1=60,param2=60,minRadius=50,maxRadius=200)
			# circles = np.uint16(np.around(circles))

			if circles != None:
				print circles.shape
				print circles[0,0,0]
				for i in circles[0,:]:
					# draw the outer circle
					circle_frame = cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
					# draw the center of the circle
					circle_center = cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
					print "circle x: %d circle y: %d" %(i[0], i[1])

			if circles== None:
				print "found nothing"

			cv2.imshow('detected circles',cimg)

			cv2.waitKey(200)

			""""Color Contouring"""""
	 		# Convert BGR to HSV
	 		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			# #Red Ball
			red_lower = np.array([0, 100, 100])
			red_upper = np.array([10, 255, 255])

			#HSV range {0-180, 0-255, 0-255}

			# #Blue Ball
			blue_lower = np.array([80, 100, 0])
			blue_upper = np.array([100, 255, 100])

			# Green Ball - need to adjust ranges
			green_lower = np.array([60, 100, 20])
			green_upper = np.array([70, 255, 150])

			# Yellow Ball
			yellow_lower = np.array([25, 250, 130])
			yellow_upper = np.array([30, 256, 255])

			# Threshold the HSV image to get only desired colors
			red_mask = cv2.inRange(hsv, red_lower, red_upper)
			blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
			green_mask = cv2.inRange(hsv, green_lower, green_upper)
			yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

			# mask = cv2.inRange(hsv, red_lower, red_upper)

			cv2.imshow("mask",yellow_mask)

			""""" Look for the colors with the given ranges"""
			# contours, heiarchy = cv2.findContours(mask,1,2)
			redContours, redHeiarchy = cv2.findContours(red_mask,1,2)
			blueContours, blueHeiarchy = cv2.findContours(blue_mask,1,2)
			greenContours, greenHeiarchy = cv2.findContours(green_mask,1,2)
			yellowContours, yellowHeiarchy = cv2.findContours(deepcopy(yellow_mask),1,2)

			cv2.imshow("mask2",yellow_mask)

			#if (len(redContours) > 0 or len(greenContours)>0):
			if(len(redContours) > 0 or len(blueContours) > 0 or len(greenContours) > 0 or len(yellowContours)>0):
				# print "found something"
				if(len(redContours)>0):
					red_contour = max(redContours, key=lambda x:cv2.contourArea(x))
					cv2.drawContours(frame,red_contour,-1,(0,255,0))
					x,y,w,h = cv2.boundingRect(red_contour)
					box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
					#draw Rectangle
					cv2.drawContours(frame,[box],0,(0,0,255),2)

				if(len(greenContours)>0):
					green_contour = max (greenContours, key=lambda x:cv2.contourArea(x))
					cv2.drawContours(frame,green_contour, -1, (0,255,0))
					x,y,w,h = cv2.boundingRect(green_contour)
					box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
					cv2.drawContours(frame,[box],0,(0,0,255),2)

				if (len(blueContours)>0):
					blue_contour = max (blueContours, key=lambda x:cv2.contourArea(x))
					cv2.drawContours(frame,blue_contour, -1, (0,255,0))
					x,y,w,h = cv2.boundingRect(blue_contour)
					box = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
					cv2.drawContours(frame,[box], 0, (0, 0, 255), 2)

				if (len(yellowContours)>0):
					yellow_contour = max (yellowContours, key=lambda x:cv2.contourArea(x))
					cv2.drawContours(frame,yellow_contour, -1, (0,255,0))
					x,y,w,h = cv2.boundingRect(yellow_contour)
					box = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
					cv2.drawContours(frame,[box], 0, (0, 0, 255), 2)
					print "x: %d" %((x+w/2))
					print "y: %d" %((y+h/2))
					# print "circle x: %d" %circles[0,:,:]

				cv2.imshow("CAM",frame)
				cv2.waitKey(500)

				# distancei = (w - 700.0)/-15.0
				# distancem = distancei*0.0254
				# print "final width:", w
				# print "D inches:", distancei
				# print "D meters:", distancem
				#(abs(distancem-0.4318) < 0.0254):
				#	print "STOP in", distancem, "meters"

			else:
			# elif(len(redContours)=0 or len(blueContours)=0 or len(greenContours)=0 or len(yellowContours)=0):
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

	
