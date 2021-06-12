#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from std_msgs.msg import Header
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray,queue_size = 100)

rospy.init_node('register_1')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

while not rospy.is_shutdown():

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
   marker.pose.position.x = 0.6
   marker.pose.position.y = 0
   marker.pose.position.z = -0.02
   marker.ns = "origins"
   # We add the new marker to the MarkerArray, removing the oldest
   # marker from it when necessary
 #  if(count > MARKERS_MAX):
  #     markerArray.markers.pop(0)

   markerArray.markers.append(marker)

   # Renumber the marker IDs
   id = 0
   for m in markerArray.markers:
       m.id = id
       id += 1

   # Publish the MarkerArray
   publisher.publish(markerArray)

   count += 1

   rospy.sleep(0.1)
