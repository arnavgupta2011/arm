#!usr/bin/env python3

import rospy,time 
from std_msgs.msg import Float64
import sys, select, termios, tty
global force
force =[8000,300,60,60,20,20]
f=Float64()
rospy.init_node('Arm_Command')
rate = rospy.Rate(2)

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
def move_joint(joint_num,force):
	topic = "/arm/joint"+str(joint_num)+"/command"
	pub = rospy.Publisher(topic,Float64,queue_size = 100)
	f.data = force
	pub.publish(f)	

def scan_joint():
	print("Press from 1 to 6: ")
	time.sleep(1)
	joint_num = int(getkey(100))
	if joint_num >=1 and joint_num <=6:
		selected_joint = joint_num
	else:
		print("Invalid selection::Defaut joint is 1\n")
		selected_joint = 1
	return selected_joint

command = "c"
while not rospy.is_shutdown():
	if command != "c":
		print("Control: ")
		while(command != "c"):
			index = int(joint)-1
			if command == "w":	
				move_joint(joint,force[index])
				print("Incrementing")
			elif command == "s":
				move_joint(joint,-force[index])
				print("decrementing")
			command = getkey(0.5)
	elif command == "c":
		print("select joint ")
		time.sleep(1)
		joint = str(scan_joint())
		command = "t"
	else:
		print("ideal ")
		command = "c"
	rate.sleep()






