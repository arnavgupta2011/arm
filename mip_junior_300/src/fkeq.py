#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np
def fk(th1,th2,z):
	r1 = 0.0008
	r2 = 0.3
	r3 = 0.3
	d1 = z + 0.16925
	d2 = -0.055
	d3 = -0.045
	pt = [[0,0,r1,(d1)],[th1,0,r2,d2],[th2,0,r3,d3]]
	h0_1 = [[np.cos(pt[0][0]),-np.sin(pt[0][0])*np.cos(pt[0][1],),np.sin(pt[0][0])*np.sin(pt[0][1]),pt[0][2]*np.cos(pt[0][0])],[np.sin(pt[0][0]),np.cos(pt[0][0])*np.cos(pt[0][1]),-np.cos(pt[0][0])*np.sin(pt[0][1]),pt[0][2]*np.sin(pt[0][0])],[0,np.sin(pt[0][1]),np.cos(pt[0][1]),pt[0][3]],[0,0,0,1]]
	h1_2 = [[np.cos(pt[1][0]),-np.sin(pt[1][0])*np.cos(pt[1][1],),np.sin(pt[1][0])*np.sin(pt[1][1]),pt[1][2]*np.cos(pt[1][0])],[np.sin(pt[1][0]),np.cos(pt[1][0])*np.cos(pt[1][1]),-np.cos(pt[1][0])*np.sin(pt[1][1]),pt[1][2]*np.sin(pt[1][0])],[0,np.sin(pt[1][1]),np.cos(pt[1][1]),pt[1][3]],[0,0,0,1]]
	h2_3 = [[np.cos(pt[2][0]),-np.sin(pt[2][0])*np.cos(pt[2][0],),np.sin(pt[2][0])*np.sin(pt[2][1]),pt[2][2]*np.cos(pt[2][0])],[np.sin(pt[2][0]),np.cos(pt[2][0])*np.cos(pt[2][1]),-np.cos(pt[2][0])*np.sin(pt[2][1]),pt[2][2]*np.sin(pt[2][0])],[0,np.sin(pt[2][1]),np.cos(pt[2][1]),pt[2][3]],[0,0,0,1]]
	
	h0_2 = np.dot(h0_1,h1_2)
	h0_3 = np.dot(h0_2,h2_3)
	x = h0_3[0][3]
	y = h0_3[1][3]
	z = h0_3[2][3]
	#print (np.matrix(h0_3))
	return [x,y,z] 
	
def jointvalue(data):
	z = data.position[0]
	t1 = data.position[1]
	t2 = data.position[2]
	th1 = t1
	th2 = t2
	th = th1 + th2
	
	ans = fk(th1,th2,z)
	print (ans)

rospy.init_node('fk', anonymous=True)
r = rospy.Rate(1)
while not rospy.is_shutdown():
	sub = rospy.Subscriber('joint_states', JointState, jointvalue)
	r.sleep()
