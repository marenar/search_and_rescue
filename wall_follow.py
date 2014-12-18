#!/usr/bin/env python
# Marena Richardson, December 10, 2014. Final Project 

import rospy 
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import datetime

class Wall_Follow: 
	def __init__(self):
		self.distance_to_wall = -1.0
		self.angle_of_wall = -1.0
		self.avoid = False
		self.straight_ahead = -1.0
		self.direct = 0
		self.update = datetime.datetime.now() - datetime.timedelta(seconds = 3)
		self.vals = []

	def scan_received(self, msg):
		forward_measurements = []
		valid_measurements = []
		ninety = []
		twoseventy = []
		self.avoid = False
		for i in range(360):
			try:
				if msg.ranges[i] != 0 and msg.ranges[i] < 7:
					valid_measurements.append(msg.ranges[i])
				if i < 7 or i > 353:
					forward_measurements.append(msg.ranges[i])
				elif 85 < i < 95:
					ninety.append(msg.ranges[i])
				elif 265 < i < 275:
					twoseventy.append(msg.ranges[i])
			except IndexError: pass
		if len(forward_measurements):
			self.straight_ahead = sum(forward_measurements) / len(forward_measurements)
			# if there is an obstacle within 1 meter straight ahead, set self.avoid to True, triggering the obstacle avoidance part of the finite state controller.
			if self.straight_ahead != 0.0 and self.straight_ahead < 0.8:
				self.avoid = True
				# if the direction has not been set within the last 3 seconds (avoids noise), turn towards the direction (90 or 270 degrees) with more space:
				if datetime.datetime.now() - self.update > datetime.timedelta(seconds = 2): 
					self.vals = [sum(ninety) / len(ninety), sum(twoseventy) / len(twoseventy)]
					if (self.vals[0] > self.vals[1] and self.vals[1] != 0.0) or self.vals[0] == 0.0:
						self.direct = 1
					else:
						self.direct = -1
				self.update = datetime.datetime.now()
		if len(valid_measurements) and self.avoid == False:
			# wall following variables set, triggering wall following part of the finite state controller.
			self.distance_to_wall = min(valid_measurements)
			self.angle_of_wall = msg.ranges.index(self.distance_to_wall)
		else:
			self.angle_of_wall = -1.0


	def main(self): 
		""" Run loop for the wall follow node """
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		sub = rospy.Subscriber('/scan', LaserScan, self.scan_received)
		rospy.init_node('wall_follow', anonymous=True)
		r = rospy.Rate(12) #12 Hz
		while not rospy.is_shutdown():
			print "DIRECT, 90, 270", self.direct, self.vals
			print self.straight_ahead
			#print "TIME SINCE UPDATE", datetime.datetime.now() - self.update
			#print "AVOID", self.avoid
			#print "ANGLE", self.angle_of_wall
			if self.avoid == True:
				print "AVOID"
				# Object avoidance. Turn away from the object.
				msg = Twist(linear=Vector3(x=0),angular=Vector3(z=self.direct * 0.2))
			elif self.avoid == False and 0 < self.angle_of_wall < 180:
				print "90 DEGREES"
				# Wall following. Go forward and course correct so the wall stays 90 degrees to the robot.
				msg = Twist(linear=Vector3(x=0.2), angular=Vector3(z=(self.angle_of_wall - 90) * 0.01))
			elif self.avoid == False and 180 < self.angle_of_wall < 360: 
				print "270 DEGREES"
				# Wall following. Go forward and course correct so the wall stays 270 degrees to the robot. 
				msg = Twist(linear=Vector3(x=0.2), angular=Vector3(z=(self.angle_of_wall - 270) * 0.01))
			else:
				print "STRAIGHT"
				# There are no obstacles and no walls. Go straight until something appears. 
				msg = Twist(linear=Vector3(x=0.1))
			pub.publish(msg)
			r.sleep()

if __name__ == '__main__':
	try:
		myProgram = Wall_Follow()
		myProgram.main()
	except rospy.ROSInterruptException: pass


