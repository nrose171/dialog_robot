#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import *

#used to communicate with recognizer node so dialog node knows when to listen and when not to listen

def pause():
	rospy.wait_for_service('/recognizer/pause')
	try:
		recognizer_pause_fc = rospy.ServiceProxy('recognizer/pause',Empty)
		recognizer_pause_fc()
	except rospy.ServiceException, e:
		rospy.loginfo ("Service call pause failed: " + e)


def continuing():
	rospy.wait_for_service('/recognizer/continue')
	try:
		recognizer_continue_fc = rospy.ServiceProxy('recognizer/continue',Empty)
		recognizer_continue_fc()
	except rospy.ServiceException, e:
		rospy.loginfo ("Service call continue failed: " + e)

def stop():
	rospy.wait_for_service('/recognizer/stop')
	try:
		recognizer_stop_fc = rospy.ServiceProxy('recognizer/stop',Empty)
		recognizer_stop_fc()
	except rospy.ServiceException, e:
		rospy.loginfo ("Service call stop failed: " + e)

if __name__ == "__main__":

	print("Requesting pause...")
	pause()
	print("Requesting continue...")
	continuing()
	
