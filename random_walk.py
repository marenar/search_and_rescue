#!/usr/bin/env python
# Marena Richardson, December 17, 2014. Final Project.

import rospy 
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
import datetime
import random

class Wall_Follow: 
	def __init__(self):
		self.avoid = False
		self.straight_ahead = -1.0
		self.angle = None
		self.degrees = None
		self.direct = 0
		self.update = datetime.datetime.now()
		self.time = 0

	def scan_received(self, msg):
		forward_measurements = []
		clear_directions = []
		self.avoid = False
		for i in range(360):
			try:
				if msg.ranges[i] > 2 and msg.ranges[i] < 7:
					clear_directions.append(msg.ranges[i])
				if i < 10 or i > 350:
					forward_measurements.append(msg.ranges[i])
			except IndexError: pass
		self.straight_ahead = sum(forward_measurements) / len(forward_measurements)

		if datetime.datetime.now() - self.update > datetime.timedelta(seconds = self.time):
			print self.straight_ahead
			# if there is an obstacle within 1 meter straight ahead, set self.avoid to True, triggering the obstacle avoidance part of the finite state controller.
			if self.straight_ahead != 0.0 and self.straight_ahead < 0.8:
				self.avoid = True
				if len(clear_directions):
					x = random.choice(clear_directions)
					self.degrees = msg.ranges.index(x)
					self.angle = math.radians(msg.ranges.index(x))
					self.direct = 1
					if math.pi < self.angle < 2 * math.pi:
						self.angle = math.fabs(self.angle - 2 * math.pi)
						self.direct = -1
				else:
					self.degrees = 180
					self.angle = math.pi
					self.direct = 1
			else:
				self.angle = None
				self.direct = 0

	def main(self): 
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		sub = rospy.Subscriber('/scan', LaserScan, self.scan_received)
		rospy.init_node('wall_follow', anonymous=True)
		r = rospy.Rate(10) #12 Hz
		while not rospy.is_shutdown():
			if self.avoid == True:
				self.time = self.angle / 0.2
				print "DEGREES", self.degrees
				print "TURN", self.direct
				print "TIME", self.time
				self.update = datetime.datetime.now()
				print "start"
				while datetime.datetime.now() - self.update < datetime.timedelta(seconds = self.time):
					msg = Twist(linear=Vector3(x=0),angular=Vector3(z=self.direct * 0.2))
					pub.publish(msg)
				print "stop"
			else:
				msg = Twist(linear=Vector3(x=0.1))
				pub.publish(msg)
			r.sleep()

if __name__ == '__main__':
	try:
		myProgram = Wall_Follow()
		myProgram.main()
	except rospy.ROSInterruptException: pass


