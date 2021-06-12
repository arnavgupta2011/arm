#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from std_msgs.msg import Header
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size = 100)

rospy.init_node('register')

markerArray = MarkerArray()
marker = Marker()
marker.header.frame_id = "world"
marker.type = marker.SPHERE
marker.action = marker.ADD
marker.scale.x = 0.1
marker.scale.y = 0.1
marker.scale.z = 0.1
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.pose.orientation.w = 1.0
id = 1

while not rospy.is_shutdown():

   
   marker.pose.position.x = 0.3
   marker.pose.position.y = 0.3
   marker.pose.position.z = -0.1
   marker.ns = "goal"
   markerArray.markers.append(marker)
   marker.id = id 
   publisher.publish(markerArray)
   rospy.sleep(1)
   marker.pose.position.x = 0.6
   marker.pose.position.y = 0
   marker.pose.position.z = -0.1
   marker.ns = "origin"
   marker.id = id
   markerArray.markers.append(marker)
   publisher.publish(markerArray)
   id += 1

   rospy.sleep(1)
