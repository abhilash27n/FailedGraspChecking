#!/usr/bin/env python
 
# Code for retrying grasp if grasp is empty
# Empty grasp decided on threshold value in check_grasp function
 
import rospy
from std_srvs.srv import Empty
from copy import deepcopy
 
# Imports the code to control the hand using smart actions
from reflex import reflex_smarts
from reflex_msgs.msg import Hand, RadianServoPositions

# grasp_success 0 = false, 1 = true
grasp_success = 0;

def check_grasp(Hand_msg):

	global grasp_success

	#threshold for deciding successful or unsuccessful grasp
	threshold = 3.3 
	spool_value = [0, 0, 0]
	hand = deepcopy(Hand_msg)
	for i in range(3):
		rospy.loginfo("Spool of hand %s is %s",i,hand.finger[i].spool)
		spool_value[i] = hand.finger[i].spool

	if spool_value[0]>threshold or spool_value[1]>threshold or spool_value[2]>threshold:
		rospy.loginfo("Unsuccessful Grasp")
		grasp_success = 0
		return
	else:
		rospy.loginfo("Successful Grasp")
		grasp_success = 1
		return



def start_grasp():

	global grasp_success
	rospy.init_node('Grasp_Hand_Node')

	# Starts up hand and publishes the command data
	reflex_hand = reflex_smarts.ReFlex_Smarts()
	rospy.sleep(1)

	# Zeroing the tactile data is necessary if you wish to use a mode
	# that employs tactile data (like guarded_move) because the
	# sensors drift over time. This creates a callable service proxy 
	zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)
	zero_fingers = rospy.ServiceProxy('/zero_fingers', Empty)

	 
	#zero_tactile()
	##################################################################
	
	while grasp_success == 0:
		rospy.loginfo("Entering Start Grasp")
		rospy.sleep(2)

		#Opening hands before starting
		reflex_hand.command_smarts('open')
		rospy.loginfo("Opened Hand") 
		rospy.sleep(3);
		reflex_hand.command_smarts('open')
		rospy.loginfo("Opened Hand") 
		rospy.sleep(3);
		
		#reset tactile sensors
		zero_tactile()

		# Close hands using 'close' or 'guarded_move'
		reflex_hand.command_smarts('guarded_move')

		#subscribe to reflex_hand topic to get radian information and tactile sensor information
		#hand = Hand()
		topic = '/reflex_hand'
		rospy.loginfo('ReFlex class is subscribing to topic %s', topic)
		rospy.Subscriber(topic, Hand, check_grasp)

		# Note: this message isn't printed until the command completes
		#rospy.loginfo("Closed Hand")
		rospy.sleep(5)

	
	#rospy.spin()




if __name__ == '__main__':
	#initialize_grasp()
	start_grasp()