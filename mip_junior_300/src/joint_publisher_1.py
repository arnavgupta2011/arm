#!/usr/bin/env python3
import rospy
import time
from invLib import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import sys, select, termios, tty
joint = JointState()

goal = [0.6008,0,0.2]
inc =[0.008,0.01,0.01]

def getkey(key_timeout):
	settings = termios.tcgetattr(sys.stdin)
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key
def set_goal(msg,pub):
	global goal, joint
	goal = msg.point
	print("calculating inverse")
	joint_pos = ik(goal)
	joint = change_pos(joint_pos,joint,pub)
	print ("position changed")

def f(inp = []): #Forward Function
	global goal
	th1 = inp[0]
	th2 = inp[1]
	z = inp[2]
	[r1,r2,r3] = [0.0008,0.3,0.3]
	[d1,d2,d3] = [z + 0.16925,-0.055,-0.045]
	pt = [[0,0,r1,(d1)],[th1,0,r2,d2],[th2,0,r3,d3]]
	h0_1 = [[np.cos(pt[0][0]),-np.sin(pt[0][0])*np.cos(pt[0][1]),np.sin(pt[0][0])*np.sin(pt[0][1]),pt[0][2]*np.cos(pt[0][0])],[np.sin(pt[0][0]),np.cos(pt[0][0])*np.cos(pt[0][1]),-np.cos(pt[0][0])*np.sin(pt[0][1]),pt[0][2]*np.sin(pt[0][0])],[0,np.sin(pt[0][1]),np.cos(pt[0][1]),pt[0][3]],[0,0,0,1]]
	h1_2 = [[np.cos(pt[1][0]),-np.sin(pt[1][0])*np.cos(pt[1][1]),np.sin(pt[1][0])*np.sin(pt[1][1]),pt[1][2]*np.cos(pt[1][0])],[np.sin(pt[1][0]),np.cos(pt[1][0])*np.cos(pt[1][1]),-np.cos(pt[1][0])*np.sin(pt[1][1]),pt[1][2]*np.sin(pt[1][0])],[0,np.sin(pt[1][1]),np.cos(pt[1][1]),pt[1][3]],[0,0,0,1]]
	h2_3 = [[np.cos(pt[2][0]),-np.sin(pt[2][0])*np.cos(pt[2][0]),np.sin(pt[2][0])*np.sin(pt[2][1]),pt[2][2]*np.cos(pt[2][0])],[np.sin(pt[2][0]),np.cos(pt[2][0])*np.cos(pt[2][1]),-np.cos(pt[2][0])*np.sin(pt[2][1]),pt[2][2]*np.sin(pt[2][0])],[0,np.sin(pt[2][1]),np.cos(pt[2][1]),pt[2][3]],[0,0,0,1]]
	
	h0_2 = np.dot(h0_1,h1_2)
	h0_3 = np.dot(h0_2,h2_3)
	x = h0_3[0][3]
	y = h0_3[1][3]
	z = h0_3[2][3]
	outputerror = squareadd(3,[(x-goal[0]),(y-goal[1]),(z-goal[2])])
	return outputerror
	
def f_gol(d):
    	global search,past_point
    	inp = past_point + np.array(search)*d
    	return f(inp)

    
def ik_solve(pointp = []):
	global search,past_point
	k = 0
	flag = True
	while(flag):
		past_point = pointp
		grad_point = grad(f,pointp)
		search = -1*np.array(grad_point)
        
		if (squareadd(3,grad_point) <= 0.001 or k>=10):
			flag = False
            
		else:
    			[pointp,d] = go_search(f_gol,pointp,search)
    			delta = pointp - past_point
    			if squareadd(3,delta)/squareadd(3,past_point)<=0.001:
        			flag = False
		k+=1
	return pointp

def ik(goal):
	
	
	#pub = rospy.Publisher("/joint_states",JointState,queue_size = 500)
	#sub = rospy.Subscriber('/arm/goal/command',Float64MultiArray,set_goal)	
	sliceAccuracy = 60 	#Higher the accuracy better the point more the time
	shuffleConstant = 7 	#Higher the constant more the chances of hitting better point
	minError = 0.1 	#initial Error , will be used in future to compare and extract min error

	[upper,lower] = getLimit("/home/arnav/arm/src/mip_junior_300/urdf/model.urdf")

	rangeMatrix = sliceRange(sliceAccuracy,lower,upper)
	
	for i in rangeMatrix:	
		for j in range(3):
			i.pop()

	pointMatrix = transpose(rangeMatrix)
	pointMatrix = shuffleMatrix(shuffleConstant,pointMatrix)
	bucket = []
	#index = 0
	for i in range (len(pointMatrix[0])):
		initialPoint = [pointMatrix[0][i],pointMatrix[1][i],pointMatrix[2][i]]
		angles = ik_solve(initialPoint)
		bucket.append(angles)
		error = f(angles)
		if (error < minError):
			minError = error
			index = i
	#print(index)
		
	print("\nJoint Angles required: "+ str(bucket[index]))
	print("\nPoint Reached after computation: "+str(goal_(bucket[index])))
	print("\nRMS Error (%): "+str(f(bucket[index])*100))	
	prism1 = bucket[index][2]
	rev1 = bucket[index][0]
	rev2 = bucket[index][1]
		#prism1 = (int(prism1*100)/100)
		#rev1 = (int(rev1*100)/100)
		#rev2 = (int(rev2*100))
	prism1 = round(prism1,2)
	rev1 = round(rev1,2)
	rev2 = round(rev2,2)
		
		
	return [prism1,rev1,rev2]		

def get_pos():
	print("Enter coordinates")
	print("enter the x coordinate")
	x = float(input())
	print("enter the y coordinate")
	y = float(input())
	print("enter the z coordinate")
	z = float(input())
	goal = [x,y,z]
	if x == "":
		x = 0.6008
		if y == "":
			y = 0
			if z == "":
				z = 0
	return goal


def change_pos(a,joint,pub):
	m=0
	[x,y,z] = joint.position[0:3:1]
	while abs(x-a[0])>=0.001 or abs(y-a[1])>=0.001 or abs(z-a[2]) >= 0.001 :
		if a[0]>x:
			x = x + inc[0]
			if x>0.225:
				x = 0.225
				
		elif a[0]<x:
			x = x-inc[0]
			if x<0:
				x = 0
		if a[1]>y:
			y = y + inc[1]
			if y> 2.05948851735:
				y = 2.05948851735
		elif a[1]<y:
			y = y - inc[1]
			if y<-2.05948851735:
				y = -2.05948851735
		if a[2]>z:
			z = z + inc[2]
			if z>2.05948851735:
				z = 2.05948851735
		elif a[2]<z:
			z = z - inc[2]
			if z < -2.05948851735:
				z = -2.05948851735
		joint.position[0] = x
		joint.position[1] = y
		joint.position[2] = z
		joint.header.stamp = rospy.Time.now()
		pub.publish(joint)
		time.sleep(0.1)
	return joint
			
		
	
	
	
if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node('joint_pub', anonymous=True)
	r = rospy.Rate(20)
	pub = rospy.Publisher("/joint_states",JointState,queue_size = 500)
	joint = JointState()
	joint.header.stamp = rospy.Time.now()
	joint.name = ["prismatic1","revolute1","revolute2","revolute3","EE_right","EE_left"]
	joint.velocity = []
	joint.position = [0,0,0,0,0,0]
	joint.effort = []
	pub.publish(joint)
	command = "c"
	while not rospy.is_shutdown():
		sub = rospy.Subscriber('/clicked_point',PointStamped,set_goal,pub)
		joint.header.stamp = rospy.Time.now()
		pub.publish(joint)
		
