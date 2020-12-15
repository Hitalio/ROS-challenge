#!/usr/bin/env python

from __future__ import print_function
import rospy
import sys
from std_srvs.srv import Empty
from turtlesim.srv import Kill
from turtlesim.srv import Spawn

def usage():
	return "Accepted arguments: clear reset spawn kill"

# Reset service
def reset_service():
	rospy.wait_for_service('/reset')
	try:
		serv = rospy.ServiceProxy('/reset',Empty)
		resp = serv()
		rospy.loginfo ("Executed reset service")

	except rospy.ServiceException, e:
		rospy.loginfo ("Service call failed: %s" %e)

# Clear service
def clear_service():
	rospy.wait_for_service('/clear')
	try:
		serv = rospy.ServiceProxy('/clear',Empty)
		resp = serv()
		rospy.loginfo ("Executed clear service")
	except rospy.ServiceException, e:
		rospy.loginfo ("Service call failed: %s" %e)

# Kill service
def kill_service():
	turtle_name = raw_input ("Enter the name of the turtle: ")
	rospy.wait_for_service('/kill')
	try:
		serv = rospy.ServiceProxy('/kill', Kill)
		resp = serv(turtle_name)
		rospy.loginfo ("Executed kill service: " + turtle_name)
	except rospy.ServiceException, e:
		rospy.loginfo ("Service call failed: %s" %e)

# Spawn service
def spawn_service():
	turtle_name = raw_input ("Enter the name of the turtle: ")
	xPos = raw_input ("Enter x position of the turtle: ")
	yPos = raw_input ("Enter y position of the turtle: ")

	rospy.wait_for_service('/spawn')
	try:
		serv = rospy.ServiceProxy('/spawn', Spawn)
		resp = serv( float(xPos), float(yPos), float(0), turtle_name)
		rospy.loginfo ("Executed spawn service: " + turtle_name + " at " + xPos + ";" + yPos)
	except rospy.ServiceException, e:
		rospy.loginfo ("Service call failed: %s" %e)

def process_input():

	rospy.loginfo ("Starting service call node")
	rospy.init_node('services_node', anonymous=False)

	rate = rospy.Rate(2)

	print ("Press " + "q" + " to exit services node")
	print(usage())

	input_string = " "

	while (input_string != "q"):

		input_string = raw_input("Enter the service to run: ")
		if (input_string == "clear"):
			clear_service()
		if (input_string == "reset"):
			reset_service()
		if (input_string == "kill"):
			kill_service()
		if (input_string == "spawn"):
			spawn_service()
		rate.sleep()

	rospy.loginfo("Terminating services node")

if __name__ == '__main__':
	process_input()


