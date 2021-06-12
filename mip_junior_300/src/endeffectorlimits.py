#!/usr/bin/env python3
import rospy,tf,time
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion

def max_value(trans,a):
	if(a==0):
		[x1,x2,y1,y2,z1,z2]=[0.0,0.0,0.0,0.0,0.0,0.0]
	print("limits resetted")
	
	if (x2<(trans[0])):
		x2=trans[0]
	elif (x1<(trans[0])):
		x1=trans[0]
	if (y2<(trans[0])):
		y2=trans[0]
	elif (y1<(trans[0])):
		y1=trans[0]
	if (z2<(trans[0])):
		z2=trans[0]
	elif (z1<(trans[0])):
		z1=trans[0]
	print("limit: {("+str(x1)+","+str(x2)+"),("+str(y1)+","+str(y2)+"),("+str(z1)+","+str(z2)+")")


	
rospy.init_node('fk', anonymous=True)
r = rospy.Rate(1)
listener = tf.TransformListener() 
a=0
while not rospy.is_shutdown():
	try:
		(trans,rot) = listener.lookupTransform("/base_link","/gripper_rev", rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
   		continue
	max_val(trans,a)
	a+=1
					

