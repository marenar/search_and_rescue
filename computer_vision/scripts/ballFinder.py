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
		self.markerPublisher = rospy.Publisher('visualization_marker_array', MarkerArray)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		rospy.pose_sub = rospy.Subscriber('slam_out_pose',PoseStamped,self.update_pose)
		self.markerArray = MarkerArray()
		self.proportion_constant = 0.35 #percentage of overlap between masks required for plotting on rviz
		#self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		#rospy.init_node('oodometry', anonymous=True) #make node 
		#rospy.Subscriber('odom',Odometry,Position)
		self.bridge = CvBridge()
		self.image = None
		self.pose = None
		self.cmd = Twist()
		self.red = False
		self.green = False
		self.yellow = False
		self.blue = False
		#self.stop = False
		#self.pose = odom.pose.pose.position
		#self.odom = None

		#self.top_cutoff = .9

		#self.found_lines = np.zeros((480, 640,3), np.uint8)
		#self.image = cv2.imread("StopSign16in.png")

	def update_pose(self,msg):
		self.pose = msg.pose

	#update_rviz publishes the markers where the balls have been found.
	#it expects to recieve the balls coordinates as a list: [coordinate_x, coordinate_y]
	#also expects to be told the ball colour as a lower-case string ie: "red"
	def update_rviz(self,location,ball_colour):
		colorDict = {'red':0,'blue':1,'green':2,'yellow':3}
		marker = Marker()
		#marker.id = colorDict.get(ball_colour)
 		marker.id = 1
		marker.header.frame_id = "/map"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 0.2
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
		elif(ball_colour == "yellow"):
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 0.0
		else:
			marker.color.r = 0.0
			marker.color.g = 0.0
			marker.color.b = 0.0
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = location[0]
		marker.pose.position.y = location[1]
		print "MarkerPose:",marker.pose.position
		marker.pose.position.z = .1
		self.markerArray.markers.append(marker)
		self.markerPublisher.publish(self.markerArray)

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
		total_offset = math.sqrt(x_offset**2 + y_offset**2)
		robot_ball_theta = math.atan2(y_offset,x_offset) #angle of ball relative to robot
		rotation = (self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w)
		euler_angle = euler_from_quaternion(rotation)
		robot_theta =euler_angle[2] #angle of robot
		ball_theta = robot_theta + robot_ball_theta  #angle of ball relative to map
		x_distance = (total_offset * math.sin(ball_theta))/100  #distance to ball in map reference meters
		y_distance = (total_offset * math.cos(ball_theta))/100  #distance to ball in map reference meters
		location = [self.pose.orientation.x + x_distance, self.pose.orientation.y + y_distance]
		return location

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

			circles = cv2.HoughCircles(cimg,cv.CV_HOUGH_GRADIENT,1,50,param1=60,param2=1,minRadius=15,maxRadius=200)
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
			blue_lower = np.array([80, 100, 0])
			blue_upper = np.array([100, 255, 100])

			# Green Ball - need to adjust ranges
			green_lower = np.array([60, 100, 20])
			green_upper = np.array([70, 255, 150])

			# Yellow Ball
			yellow_lower = np.array([25, 200, 130])
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

			if circles != None:
				#print circles.shape
				#print circles[0,0,:]
				proportion_overlap = []
				for i in circles[0,:]:
					# draw the outer circle
					circle_frame = cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
					# draw the center of the circle
					circle_center = cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
					#if self.red = False:
					#self.update_rviz(self.get_distance(i[0],i[1],2*i[2]),"green")

					circle_mask = np.zeros(yellow_mask.shape)
					cv2.circle(circle_mask,(i[0],i[1]),i[2],255,-1)

					#print circle_mask.dtype

					#print circle_mask.shape
					#print yellow_mask.shape
					combined = cv2.bitwise_and(circle_mask.astype(np.uint8),yellow_mask.astype(np.uint8))
					proportion_overlap.append( (np.sum(combined)/255.0)/(math.pi*i[2]**2) )
					cv2.imshow('circle_mask',combined)
					print circles
					# [x coordinate, y coor, radius]
					# print "circle x: %d circle y: %d" %(i[0], i[1])

				if 1 > max(proportion_overlap) > self.proportion_constant:
					best_circle = np.argmax(proportion_overlap)
					i = circles[0,best_circle,:]
					#print proportion_overlap
					self.update_rviz(self.get_distance(i[0],i[1],2*i[2]),"yellow")

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
					#print "x: %d" %((x+w/2))
					#print "y: %d" %((y+h/2))
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

				cv2.waitKey(100)
				r.sleep()

if __name__ == '__main__':
	lf = BallFinder()
	lf.run()

	
