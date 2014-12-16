#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
frame = cv2.imread("StopSign16in.png")
#distancem should be initiated as an arbitrarily high value
distancem = 9001

class LineFinder:
	def __init__(self):
		rospy.init_node('line_finder', anonymous = True)

	def on_mouse(self,event,x,y,flag,param):
		if event == cv2.EVENT_LBUTTONDOWN:
			hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			print "hsv:", hsv[y][x]
			print "rgb:", frame[y][x]
	def dostuff(self):
		while(frame != None):
			cv2.imshow("CAM",frame)	
			cv2.waitKey(5000)



	def run(self):
		cv2.namedWindow("CAM")
		cv2.setMouseCallback("CAM",self.on_mouse)

		r=rospy.Rate(10)
		while not rospy.is_shutdown():

				self.dostuff()


				cv2.waitKey(50)
				r.sleep()


if __name__ == '__main__':
	lf = LineFinder()
	lf.run()


