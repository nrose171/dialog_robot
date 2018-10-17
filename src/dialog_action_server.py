#!/usr/bin/env python

from dummy_speech.srv import *
import rospy
import roslaunch
from std_msgs.msg import String
from robotics_task_tree_msgs.msg import *

path = "/home/nataliereu/catkin_ws/src/Distributed_Collaborative_Task_Tree/table_setting_demo/params"

flag = False
close = False
TOPIC = "NONE"

def stateCallback(data):
	global close
	#rospy.loginfo(rospy.get_caller_id() + "I heard")
	#rospy.loginfo("%d", data.done)
	if data.done == 1:
		close = True

def handle_dialog_action(req):

	global flag, close, TOPIC
	F = open(path+"/NodeDescription.yaml", "w")
	rospy.loginfo("Handling YAML:\n" + req.action + "\nFirst Node: " + req.firstNode)
	F.write(req.action)
	rospy.sleep(1)
	F.close()
	#launch the required node
	flag = True
	TOPIC = req.firstNode
	while close == False:
		#rospy.loginfo("Doing the job, not done yet.")
		rospy.sleep(1)
	close = False
	rospy.loginfo("I am done!! for good : -)")
	return DialogActionResponse("Done")

def dialog_action_server():
	global flag, close
	
	rospy.init_node('dialog_action_server')
	rate = rospy.Rate(10) # 10hz
	s = rospy.Service('dialog_action', DialogAction, handle_dialog_action)
	rospy.loginfo ("Ready to handle dialog requests")

	
	while not rospy.is_shutdown():
		if flag == True:

			#launching remote mutex
			uuid_m = roslaunch.rlutil.get_or_generate_uuid(None, False)
			roslaunch.configure_logging(uuid_m)
			launch_m = roslaunch.parent.ROSLaunchParent(uuid_m, ["/home/nataliereu/catkin_ws/src/Distributed_Collaborative_Task_Tree/remote_mutex/launch/table_setting_mutex.launch"])
			
			rospy.set_param('/root_topic', TOPIC + "_state") 

			launch_m.start()
			rospy.sleep(2.0)
			rospy.loginfo("Launched remote mutex")
			#flag = False

			#launching multi_robot_demo
			uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
			roslaunch.configure_logging(uuid)

			#PR2 running version
			launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nataliereu/catkin_ws/src/Distributed_Collaborative_Task_Tree/table_setting_demo/launch/multi_robot_task_demo_visionManip.launch"])

			#No PR2 version
			#launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nataliereu/catkin_ws/src/Distributed_Collaborative_Task_Tree/table_task_sim/launch/dummy_multi_demo.launch"])

			act = robotics_task_tree_msgs.msg.ControlMessage()
			act.sender.type = 4
			act.sender.robot = 0
			act.sender.node = 0
			act.activation_level = 5000000000000000.0
			act.activation_potential = 0.0
			act.done = False
			pub = rospy.Publisher(TOPIC+"_parent", ControlMessage, queue_size=10)
			rospy.Subscriber(TOPIC+"_state", State, stateCallback)
			rospy.loginfo("In loop")

			launch.start()
			rospy.sleep(2.0)
			pub.publish(act)
			rospy.loginfo("Published activation")
			flag = False


		if close == True:
			launch.shutdown()
			launch_m.shutdown()	
			#close = False
			rospy.loginfo("I am done!! and closing!!!!")
	

	#rospy.spin()

if __name__== "__main__":
	dialog_action_server()
