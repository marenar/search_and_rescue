#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion
import math
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from nav_msgs.msg import Odometry
from copy import deepcopy
import time

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
		rospy.init_node('register')
		rospy.pose_sub = rospy.Subscriber('slam_out_pose',PoseStamped,self.update_pose)
		self.markerPublisher = rospy.Publisher('visualization_marker_array', MarkerArray)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		self.markerArray = MarkerArray()
		self.proportion_constant = 0.35 #percentage of overlap between masks required for plotting on rviz
		#self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		#rospy.init_node('oodometry', anonymous=True) #make node 
		#rospy.Subscriber('odom',Odometry,Position)
		self.bridge = CvBridge()
		self.image = None
		self.pose = None
		self.cmd = Twist()
		self.red = {}
		self.blue = {}
		self.green = {}
		self.yellow = {}
		red_location = None;
		blue_location = None;
		green_location = None;
		yellow_location = None;



	def update_pose(self,msg):
		self.pose = msg.pose

	#update_rviz publishes the markers where the balls have been found.
	#it expects to recieve the balls coordinates as a list: [coordinate_x, coordinate_y]
	#also expects to be told the ball colour as a lower-case string ie: "red"
	def update_rviz(self,location,ball_colour,confidence):
		colourDict = {'red':0,'blue':1,'green':2,'yellow':3}
		marker = Marker()
		marker.id = colourDict.get(ball_colour)
#		marker.id = 1
		marker.header.frame_id = "/map"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
		marker.color.a = 1.0
		if(ball_colour == "red"):
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			self.red[confidence] = location
			max_key = max(k for k, v in self.red.iteritems())
			#red_location = self.red[max_key]
		elif(ball_colour == "blue"):
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 1.0
			self.blue[confidence] = location
			max_key = max(k for k, v in self.blue.iteritems())
			#blue_location = self.blue[max_key]
		elif(ball_colour == "green"):
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			self.green[confidence] = location
			max_key = max(k for k, v in self.green.iteritems())
			#green_location = self.green[max_key]
		elif(ball_colour == "yellow"):
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			self.yellow[confidence] = location
			max_key = max(k for k, v in self.yellow.iteritems())
			#yellow_location = self.yellow[max_key]
		else:
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 0.0

		# markerSum = marker.color.r + marker.color.g + marker.color.b
		# if markerSum > 0:
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = location[0]
		marker.pose.position.y = location[1]
		marker.pose.position.z = .1
		self.markerArray.markers.append(marker)
		self.markerPublisher.publish(self.markerArray)
		#print "red location", red_location
		#print "blue location", blue_location
		#print "green location", green_location
		#print "yellow location", yellow_location



	#besides self expects the diameter of the ball in pixels(based on circle transform)
	#Also expects the X-Y coordinates of the ball in the camera frame
	#returns the real map location of detected balls
	def get_distance(self,ball_x,ball_y,diameter):
		#the field of view equals the distance from the neato
		#the pixel width of the screen is 640
		pixels_x = 640.0 # the width, in pixels, of the image
		initialDiameter = 22.6 # diameter, in cm, of the physical ball
		y_offset = (pixels_x * initialDiameter) / diameter # distance in cm in the y direction relative to robot
		pixelDist = y_offset / pixels_x # cm/pixel
		x_offset = ((pixels_x/2.0) - ball_x)*pixelDist #X offset from center of camera in cm + is right - is left
		print "x_offset", x_offset
		print "y_offset", y_offset
		total_offset = math.sqrt(x_offset**2 + y_offset**2)
		#print "x_offset", x_offset
		#print "y_offset", y_offset
#		try:
		robot_ball_theta = math.atan2(x_offset,y_offset) #-angle of ball relative to robot
		rotation = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
		euler_angle = euler_from_quaternion(rotation)
		robot_theta =euler_angle[2] #angle of robot
		#print "robot_theta",robot_theta
		#print "robot_ball_theta",robot_ball_theta
		ball_theta = robot_theta + robot_ball_theta  #angle of ball relative to map
		#print "ball_theta",ball_theta
		#x_distance = (total_offset * math.sin(ball_theta))/100  #distance to ball in map reference meters
		#y_distance = (total_offset * math.cos(ball_theta))/100  #distance to ball in map reference meters
		x_distance = (total_offset * math.cos(ball_theta))/100  #distance to ball in map reference meters
		y_distance = (total_offset * math.sin(ball_theta))/100  #distance to ball in map reference meters

		#roughOffsetX = (y_offset/100.0) * math.cos(robot_theta)
		#roughOffsetY = (y_offset/100.0) * math.sin(robot_theta)

		location = [self.pose.position.x + x_distance, self.pose.position.y + y_distance]
		print "location", location
		print "robot_theta", robot_theta
		return location
#		except:
#			return False

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

	def processImage(self,mask_colour,circle_frame,circle_center,i):
		if np.sum(mask_colour) > 0:
			circle_mask = np.zeros(mask_colour.shape)
			cv2.circle(circle_mask,(i[0],i[1]),i[2],255,-1)
			combined = cv2.bitwise_and(circle_mask.astype(np.uint8),mask_colour.astype(np.uint8))
			return (np.sum(combined)/255.0)/(math.pi*i[2]**2)

	def validateImage(self,proportion_overlap,circles,index):
		iterateColour = ["red","blue","green","yellow"]
		if 1 > max(proportion_overlap) > self.proportion_constant:
			best_circle = np.argmax(proportion_overlap)
			i = circles[0,best_circle,:]
			totalDistance = self.get_distance(i[0],i[1],2*i[2])
			if totalDistance != False:
				self.update_rviz(totalDistance,iterateColour[index],max(proportion_overlap))
			else:
				print "FAILED TO FIND ORIENTATION"


	def detectBall(self):
	 	if self.image != None:
	 	 	frame = self.image.copy()

	 	 	""""HoughCircles detection"""""
	 	 	cv2.imshow("CAM",frame)	
			img = cv2.medianBlur(frame,5)
			#img = self.bridge.cv2_to_imgmsg(img,"bgr8")
			cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

			circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,1,50,param1=60,param2=60,minRadius=15,maxRadius=200)
			# circles = np.uint16(np.around(circles))

			cv2.imshow('detected circles',cimg)

			cv2.waitKey(50)

			""""Color Contouring"""""
	 		# Convert BGR to HSV
	 		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			# #Red Ball
			red_lower = np.array([0, 100, 100])
			red_upper = np.array([10, 255, 255])

			#HSV range {0-180, 0-255, 0-255}

			# #Blue Ball
			blue_lower = np.array([70, 80, 0])
			blue_upper = np.array([100, 255, 100])

			# Green Ball - need to adjust ranges
			green_lower = np.array([60, 100, 20])
			green_upper = np.array([70, 255, 150])

			# Yellow Ball
			yellow_lower = np.array([25, 200, 100])
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
			redContours, redHeiarchy = cv2.findContours(deepcopy(red_mask),1,2)
			blueContours, blueHeiarchy = cv2.findContours(deepcopy(blue_mask),1,2)
			greenContours, greenHeiarchy = cv2.findContours(deepcopy(green_mask),1,2)
			yellowContours, yellowHeiarchy = cv2.findContours(deepcopy(yellow_mask),1,2)

			maskArray = [red_mask,blue_mask,green_mask,yellow_mask]

			if circles != None:

				proportion_overlap_red = []
				proportion_overlap_blue = []
				proportion_overlap_green = []
				proportion_overlap_yellow = []
				proportion_overlap = []
				overlord_overlap = [proportion_overlap_red,proportion_overlap_blue,proportion_overlap_green,proportion_overlap_yellow]
				for i in circles[0,:]:

					# draw the outer circle
					circle_frame = cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
					# draw the center of the circle
					circle_center = cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

					for index,item in enumerate(maskArray):
						overlord_overlap[index].append(self.processImage(item,circle_frame,circle_center,i))

					for index,item in enumerate(overlord_overlap):
						self.validateImage(item,circles,index)


			if circles== None:
				print "found nothing"



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


				cv2.imshow("CAM",frame)
				cv2.waitKey(500)


			else:
			# elif(len(redContours)=0 or len(blueContours)=0 or len(greenContours)=0 or len(yellowContours)=0):
				print "found nothing"

	def run(self):
		cv2.namedWindow("CAM")
		cv2.setMouseCallback("CAM",self.on_mouse)

		r=rospy.Rate(10)
		while not rospy.is_shutdown():

				self.detectBall()

				cv2.waitKey(100)
				r.sleep()

if __name__ == '__main__':
	lf = BallFinder()
	lf.run()

	
