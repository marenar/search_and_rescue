#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray)

rospy.init_node('register')

markerArray = MarkerArray()


MARKERS_MAX = 4

while not rospy.is_shutdown():
	count = 0
	while count < MARKERS_MAX:
		marker = Marker()
		marker.id = count
		marker.header.frame_id = "/map"
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = .2
		marker.scale.y = .2
		marker.scale.z = .2
		marker.color.a = 1.0
		marker.color.r = 0.08
		marker.color.g = 0.67
		marker.color.b = 0.14
		marker.pose.orientation.w = 1.0
		marker.pose.position.x = count
		marker.pose.position.z = .1
		marker.pose.position.y = count
		#marker.pose.position.x = math.cos(count / 50.0)
		#marker.pose.position.y = math.cos(count / 40.0) 
		#marker.pose.position.z = math.cos(count / 30.0) 
		markerArray.markers.append(marker)


		# We add the new marker to the MarkerArray, removing the oldest
		# marker from it when necessary
		#if(count > MARKERS_MAX):
		#	markerArray.markers.pop(0)
		#markerArray.markers.append(marker)

		#Renumber the marker IDs
		#id = 0
		#for m in markerArray.markers:
		#	m.id = id
		#	id += 1
		publisher.publish(markerArray)
		count += 1
		rospy.sleep(1)

