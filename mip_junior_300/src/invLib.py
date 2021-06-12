import numpy as np
import xml.dom.minidom as parser
from random import shuffle
from derivative_library import *


"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------Class Joint and functions to store Joint Data --------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"

class Joint:
	def __init__(self,name,number,type_):
		self.jointNumber = number
		self.jointName = name
		self.jointType = type_
		
	def param(self,xyz,rpy,axis):
		self.co_ordinates = xyz
		self.orientation = rpy[0]
		self.axis = axis
		
	def tree(self,parent,child):
		self.parent = parent
		self.child = child
		
	def limit(self,lower,upper,effort,velocity):
		self.upper = upper
		self.lower = lower
		self.effort = effort
		self.velocity = velocity

def getinfo(type_,jointnumber,info,jointarray=[]):		
	array = []
	flag = False
	for i in range(30):
		try:
			data = getdata(type_,jointnumber,i,info,jointarray)
		except:
			flag = True
		finally:
			if flag == False:
				array.append(data)
			flag = False
	return array
	
	
def getdata(type_,num,nodenum,attri,jointarray = []):
	strlist = list((jointarray[num].childNodes[nodenum].getAttribute(attri)).split(" "))
	if type_ == "string":
		return strlist
	else:
		outlist = [float(numeric_string) for numeric_string in strlist]
		return outlist
		
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
"-------------------------------------Function to get upper and lower limit of joints---------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
		
def getLimit(urdf_path):
	data = parser.parse(urdf_path)  						
	jointinfo = []
	for joint in data.getElementsByTagName( "joint" ):
		if joint.getAttribute( "type" ) != "" and joint.getAttribute( "name" ) != "dummy" and joint.getAttribute( "type" ) != "fixed":
			jointinfo.append(joint)

	joint = [0 for i in range(len(jointinfo))]
	for i in range(0,len(jointinfo)):
		joint[i] = Joint(jointinfo[i].getAttribute("name"),i+1,jointinfo[i].getAttribute("type"))
		data3 = getinfo("float",i,"upper",jointinfo)
		data4 = getinfo("float",i,"lower",jointinfo)
		data5 = getinfo("float",i,"velocity",jointinfo)
		data6 = getinfo("float",i,"effort",jointinfo)
		joint[i].limit(data4[0][0],data3[0][0],data6,data5)

	upper = []
	lower = []
	for i in range (len(jointinfo)):
		upper.append(joint[i].upper)
		lower.append(joint[i].lower)
	return [upper,lower]
	
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
"-------------------------------------------------Forward Kinematic function of Robot---------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"

def goal_(inp = []):
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
	answer  = [x,y,z]
	return answer
	
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"	
"----------------------Matrix Manipulation functions to genrate random initial point for optimisation Algorithm-------------------"
"---------------------------------------------------------------------------------------------------------------------------------"
"---------------------------------------------------------------------------------------------------------------------------------"

def transpose(m=[]):
	outM = [[m[j][i] for j in range(len(m))] for i in range(len(m[0]))]
	return outM

def shuffleMatrix(n,matrix = []):
	for i in matrix:
		for j in range(n):
			shuffle(i)
	return matrix
	
def sliceRange(arrayDivider,lowerLimit=[],upperLimit=[]):
	n = len(lowerLimit)
	array = [[] for k in range (n)]
	point = [[] for k in range (arrayDivider)]
	for i in range (n):
		difference = upperLimit[i]-lowerLimit[i]
		step = difference/arrayDivider
		for j in range (0,arrayDivider+1):
			array[i].append(lowerLimit[i]+ j*step)	

	for i in range (0,arrayDivider):
		for j in range (0,len(array)):
			point[i].append(array[j][i])
	return point
		



	

