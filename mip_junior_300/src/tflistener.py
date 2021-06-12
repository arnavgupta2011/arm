#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
	rospy.init_node('tf2_turtle_listener')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		turtle_name = rospy.get_param('turtle', 'turtle2')
		trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
		
rate.sleep()
