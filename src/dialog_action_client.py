#!/usr/bin/env python

import sys
import rospy
from dummy_speech.srv import *

def dialog_action_client(yaml, node):
	rospy.wait_for_service('dialog_action')
	try:
		dialog_action = rospy.ServiceProxy('dialog_action',DialogAction)
		resp = dialog_action(yaml, node)
		return resp.response
	except rospy.ServiceException, e:
		rospy.loginfo ("Service call failed: " + e)

if __name__ == "__main__":
	yaml = """	NodeList: ['ROOT_4_0_000', 'PLACE_3_0_001' ]
				Nodes: 
  					ROOT_4_0_000:
    					mask:	
      						type: 4
	      					robot: 0
    	  					node: 0
    					parent: 'NONE'
    					children: ['PLACE_3_0_001']
    					peers: ['NONE']
  					PLACE_3_0_001:
    					mask:
      						type: 3
      						robot: 0
      						node: 1
    					parent: ROOT_4_0_000
    					peers: ['NONE']
				    object: Cup"""
	node = "THEN_0_0_001"

	print("Requesting action...")
	print(dialog_action_client(yaml, node))