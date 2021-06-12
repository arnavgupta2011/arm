#!/usr/bin/env python3
import rospy
from invLib import *	
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
joint = JointState()
goal = [0.6008,0,0.2]
def set_goal(msg):
	global goal
	goal = msg.data

def move_joint(msg):
	topic = "/arm/joint_angles"
	pub = rospy.Publisher(topic,Float64MultiArray,queue_size = 100)
	goal.data = msg
	pub.publish(goal)
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

    
def ik_solve(point = []):
	global search,past_point
	k = 0
	flag = True
	while(flag):
		past_point = point
		grad_point = grad(f,point)
		search = -1*np.array(grad_point)
        
		if (squareadd(3,grad_point) <= 0.001 or k>=10):
			flag = False
            
		else:
    			[point,d] = go_search(f_gol,point,search)
    			delta = point - past_point
    			if squareadd(3,delta)/squareadd(3,past_point)<=0.001:
        			flag = False
		k+=1
	return point

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
		#prism1 = round(prism1,2)
		#rev1 = round(rev1,2)
		#rev2 = round(rev2,2)
		
		
	return [prism1,rev1,rev2]
		
		

