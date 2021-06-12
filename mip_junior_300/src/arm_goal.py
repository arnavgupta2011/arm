#!/usr/bin/env python3
import rospy,time 
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

goal=Float64MultiArray()
inc =[0.01,0.01,0.01]
x_lim = [-0.14,0.6008]
y_lim = [0,0.6008]
z_lim = [0,0.32425]
dist = 0.6008
global x,y,z
	
rospy.init_node('Arm_Command')
rate = rospy.Rate(20)
[x,y,z] = [0.6008,0,0]

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
	
def move_joint():
	global x,y,z
	topic = "/arm/goal/command"
	pub = rospy.Publisher(topic,Float64MultiArray,queue_size = 100)
	goal.data = [x,y,z]
	pub.publish(goal)	
def inc_x():
	global x,y
	x = x+inc[0]
	if x>x_lim[1]:
		x = x_lim[1]
	a = ((x*x)+(y*y))
	a = (dist*dist) - a
	if a < 0:
		x = x- inc[0]
		
def dec_x():
	global x,y
	x = x-inc[0]
	if x<x_lim[0]:
		x = x_lim[0]
	a = ((x*x)+(y*y))
	a = (dist*dist) - a
	if a < 0:
		x = x+ inc[0]
def inc_y():
	global x,y
	y = y+inc[1]
	if y>y_lim[1]:
		y = y_lim[1]
	a = ((x*x)+(y*y))
	a = (dist*dist) - a
	if a < 0:
		y = y- inc[1]
def dec_y():
	global x,y
	y = y-inc[1]
	if y<y_lim[0]:
		y = y_lim[0]
	a = ((x*x)+(y*y))
	a = (dist*dist) - a
	if a < 0:
		y = y+ inc[1]
def inc_z():
	global z
	z = z+inc[2]
	if z>z_lim[1]:
		z = z_lim[1]
def dec_z():
	global z
	z = z-inc[2]
	if z<z_lim[1]:
		z = z_lim[0]

			
##def scan_axis():
##	print("choose Axis  ->    1(X axis)	2(Y axis)	3(Z axis)")
##	time.sleep(1)
##	joint_num = int(getkey(100))
##	if joint_num >=1 and joint_num <=3:
##		selected_joint = joint_num
##	else:
##		print("Invalid selection::Defaut axis is 1\n")
##		selected_joint = 1
##	return selected_joint

command = "p"
while not rospy.is_shutdown():
	if command != "p":
		print("Control: ")
		while(command != "p"):
			if command == "w":	
				inc_x()
				print([x,y,z])
			elif command == "s":
				dec_x()
				print([x,y,z])
			elif command == "a":
				dec_y()
				print([x,y,z])
			elif command == "d":
				inc_y()
				print([x,y,z])
			elif command == "8":
				inc_z()
				print([x,y,z])
			elif command == "2":
				dec_z()
				print([x,y,z])
			command = getkey(0.5)
	elif command == "p":
		print("Publishing Goal")
		time.sleep(1)
		move_joint()
		command = "t"
	elif command == "c":
		break
	else:
		print("ideal ")
		command = "p"
	rate.sleep()
