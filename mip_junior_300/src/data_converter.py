#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
rospy.init_node("joint_state_publisher_gui")
joint = JointState()

def converter(msg,pub):
	joint.header = msg.header
	joint.header.frame_id = "world"
	joint.header.stamp = rospy.Time.now()
	joint.name = msg.name
	joint.velocity = msg.velocity
	joint.position = msg.position
	joint.effort = msg.effort
	pub.publish(joint)

rate = rospy.Rate(18)

while not rospy.is_shutdown():
	pub = rospy.Publisher("joint_states",JointState,queue_size = 500)
	sub = rospy.Subscriber("arm/joint_states",JointState,converter,pub)
	rate.sleep()
