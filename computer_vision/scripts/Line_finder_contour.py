#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)
		rospy.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.update_image)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.init_node('oodometry', anonymous=True) #make node 
		rospy.Subscriber('odom',Odometry,Position)
		self.bridge = CvBridge()
		self.image = None
		self.cmd = Twist()
		self.stop = False
		self.pose = odom.pose.pose.position
		self.odom = None

		self.top_cutoff = .9



		#self.found_lines = np.zeros((480, 640,3), np.uint8)
		self.image = cv2.imread("StopSign16in.png")

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
			#print hsv[y][x]

	def follow_line(self):
		if self.image != None:
			frame = self.image.copy()
			#cv2.imshow("CAM",frame)	

			
			# Convert BGR to HSV
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

			
			#RED MASKING TAPE HSV
			lower = np.array([172,100,150])
			upper = np.array([180,255,255])

			lower2 = np.array([-1,100,150])
			upper2 = np.array([5,255,255])
			mask2 = cv2.inRange(hsv, lower2, upper2)

			#BGR
			lower = np.array([0,0,220])
			upper = np.array([150,150,255])

			# GREEN STOP SIGN Threshholds in HSV
			stop_lower = np.array([30,130,120])
			stop_upper = np.array([75,200,210])
			#Green Stop Sign Mask
			stop_mask = cv2.inRange(hsv, lower, upper)
			stop_contours, stop_heiarchy = cv2.findContours(mask,1,2)

			if(len(stop_contours) > 0 and self.stop != True):
				#we assume the largest contour by area is the one we want
				stop_contour = max(stop_contours, key = lambda x:cv2.contourArea(x))
				#define the rectangle around contour
				x,y,w,h = cv2.boundingRect(stop_contour)
				box = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
				#Draw Rectangle
				cv2.drawContours(frame,[box],0,(0,0,255),2)
				#IMPORTANT!!! THE NEXT 3 LINES ARE FOR TESTING ONLY!!
				#COMMENT OUT FOR CONTEST!!!

				#Calculate the Distance in inches
				distance_in = (w - 528.0)/-15.0
				distance_m = distance_in*0.0254
				if(abs(distance_m - 0.4218)< 0.0254):
					self.cmd = Twist()
					self.stop = True
					print "stop command sent"


			mask = cv2.inRange(frame, lower, upper)

			top = mask[:int(len(mask)*self.top_cutoff),:]
			
			bottom = mask[int(len(mask)*self.top_cutoff):,:]

			def is_red(bgr):
				return bgr
				if bgr[2] > 150 and bgr[1]<150 and bgr[0]<150:
					print bgr
					return bgr
				else:
					return [0,0,0]

			#mask = np.array(map(lambda x:map(is_red,x),frame))

			#cv2.imshow("mask",mask)

			t_contours, t_heiarchy = cv2.findContours(top,1,2)
			#print map(lambda x:cv2.contourArea(x),t_contours)

			if len(t_contours) > 0  and self.stop != True:
				t_contour = max(t_contours, key = lambda x:cv2.contourArea(x))
				cv2.drawContours(frame,t_contour,-1,(255,0,0))
				M = cv2.moments(t_contour)
				if M['m00'] == 0:
					cx = 0
					cy = 0
					print "WARNING: m00 == 0"
				else:
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
				cv2.circle(frame,(cx,cy),2,(0,0,255))

			contours, heiarchy = cv2.findContours(bottom,1,2)

			if len(contours) > 0  and self.stop != True:
				contour = max(contours, key = lambda x:cv2.contourArea(x))
				#print map(lambda x:cv2.contourArea(x),contours)
				#print str(cv2.contourArea(contour))
				cv2.drawContours(frame,contour,-1,(255,0,0), offset = (0,int(len(frame)*self.top_cutoff)))

				M = cv2.moments(contour)
				if M['m00'] == 0:
					cx = 0
					cy = 0
					print "WARNING: m00 == 0"
				else:
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])

				cv2.circle(frame,(cx,cy + int(len(frame)*self.top_cutoff)),2,(0,0,255))

				width = len(frame[0])
				center =  width/2

				ang_vel = .002 * (center-cx)
				lin_vel = ((2**2) - (ang_vel)**2)**(.5)

				self.cmd = Twist(linear = Vector3(x=.1),angular = Vector3(z=ang_vel))
			else:
				self.cmd = Twist()
			cv2.imshow("CAM",frame)
		while(stop == True):
			self.odom = self.pose


	def run(self):
		cv2.namedWindow("CAM")
		cv2.setMouseCallback("CAM",self.on_mouse)

		r=rospy.Rate(10)
		while not rospy.is_shutdown():

				self.follow_line()


				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()

	
