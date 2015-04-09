#!/usr/bin/env python
 
# Code for retrying grasp if grasp is empty
# Success of a grasp decided on threshold value and number of contacts points in check_grasp function
 
import rospy
import time
from std_srvs.srv import Empty
#from copy import deepcopy
 
# Imports the code to control the hand using smart actions
#from reflex import reflex_smarts
from reflex_msgs.msg import Hand, RadianServoPositions

# grasp_success 0 = false, 1 = true
grasp_success = 0;

#open configuration for finger


#spool_length = [0, 0, 0]

# def check_grasp(Hand_msg):

# 	global grasp_success

# 	#threshold for deciding successful or unsuccessful grasp
# 	threshold = 3.3 
# 	spool_value = [0, 0, 0]
# 	hand = deepcopy(Hand_msg)
# 	for i in range(3):
# 		rospy.loginfo("Spool of hand %s is %s",i,hand.finger[i].spool)
# 		spool_value[i] = hand.finger[i].spool

# 	if spool_value[0]>threshold or spool_value[1]>threshold or spool_value[2]>threshold:
# 		rospy.loginfo("Unsuccessful Grasp")
# 		grasp_success = 0
# 		return
# 	else:
# 		rospy.loginfo("Successful Grasp")
# 		grasp_success = 1
# 		return


def check_grasp(reflex_hand):
	
	global grasp_success
	threshold = 3
	minContactPoints = 1;
	contacts_finger0 = 0;
	contacts_finger1 = 0;
	contacts_finger2 = 0;

	# check if each finger has closed more than threshold value
	if reflex_hand.hand.finger[0].spool>threshold or reflex_hand.hand.finger[1].spool>threshold or reflex_hand.hand.finger[2].spool>threshold:
 		rospy.loginfo("Unsuccessful Grasp because atlease one finger completely closed")
 		grasp_success = 0
 		return
 	else:

 		for i in range(9):
 			if(reflex_hand.hand.finger[0].contact[i]==True):
 				contacts_finger0 = contacts_finger0 + 1;
 			if(reflex_hand.hand.finger[1].contact[i]==True):
 				contacts_finger1 = contacts_finger1 + 1;
 			if(reflex_hand.hand.finger[2].contact[i]==True):
 				contacts_finger2 = contacts_finger2 + 1;


 			# checking if minimum contact points acheived
 			# can try to close more if not, and then retract
 			if(contacts_finger0>=minContactPoints and contacts_finger1>=minContactPoints and contacts_finger2>=minContactPoints):
 				grasp_success = 1
 				rospy.loginfo("Successful Grasp")
 				return


 		rospy.loginfo("Unsuccessful Grasp because not enought contact points")
 		grasp_success = 0
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
		#reflex_hand.command_smarts('open')
		#rospy.loginfo("Opened Hand") 
		#rospy.sleep(3);
		#reflex_hand.command_smarts('open')
		#rospy.loginfo("Opened Hand") 
		open_fingers(reflex_hand);
		rospy.loginfo("Opened Hand")
		rospy.sleep(3);
		
		#reset tactile sensors
		zero_tactile()

		# Close hands using 'close' or 'guarded_move'
		#reflex_hand.command_smarts('solid_contact')

		close_fingers(reflex_hand)

		#subscribe to reflex_hand topic to get radian information and tactile sensor information
		#hand = Hand()
		#topic = '/reflex_hand'
		#rospy.loginfo('ReFlex class is subscribing to topic %s', topic)
		#rospy.Subscriber(topic, Hand, check_grasp)
		check_grasp(reflex_hand)

		# Note: this message isn't printed until the command completes
		#rospy.loginfo("Closed Hand")
		rospy.sleep(10)

	
	#rospy.spin()


def open_fingers(reflex_hand):
	
	openFingerTo = 1.2;
	reflex_hand.move_finger(0, openFingerTo)
	reflex_hand.move_finger(1, openFingerTo)
	reflex_hand.move_finger(2, openFingerTo)
	return;

def close_fingers(reflex_hand):

	rospy.loginfo("Closing fingers")	
	speed = 0.05
	close_threshold = 3;
	close_for_time = 6
	start_time = time.time();
	elapsed_time = 0;

	while ( elapsed_time < close_for_time ):
		if (reflex_hand.hand.finger[0].spool < close_threshold):
			reflex_hand.cmd_spool[0] = reflex_hand.hand.finger[0].spool+speed
		if (reflex_hand.hand.finger[1].spool < close_threshold):
			reflex_hand.cmd_spool[1] = reflex_hand.hand.finger[1].spool+speed
		if (reflex_hand.hand.finger[2].spool < close_threshold):
			reflex_hand.cmd_spool[2] = reflex_hand.hand.finger[2].spool+speed

		elapsed_time = time.time() - start_time

	rospy.loginfo("Exiting close fingers")
	return;





if __name__ == '__main__':
	#initialize_grasp()
	start_grasp()