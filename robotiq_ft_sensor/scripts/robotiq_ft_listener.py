#!/usr/bin/env python
import rospy
from robotiq_force_torque_sensor.msg import ft_sensor

# GraspStatus
# 0 = ignore
# 1 = before grasp
# 2 = after grasp
graspStatus = 0;

def callback(data):

    global graspStatus;
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.Rf)
    #rospy.loginfo("Resultant force: %s", data.Rf)
    # if(graspStatus == 0):
    #     rospy.loginfo("IGNORING MSG");
    # elif(graspStatus == 1):
    #     rospy.loginfo("Resultant force before grasp: %s", data.Rf)
    # elif(graspStatus == 2):
    #     rospy.loginfo("Resultant force after grasp: %s", data.Rf)
    
def listener():

    rfBeforeGrasp = 0;
    rfAfterGrasp = 0;
    rfDiff = 0;
    global graspStatus;
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robotiq_ft_listener_py', anonymous=True)

    #rospy.Subscriber("robotiq_force_torque_sensor", ft_sensor, callback, queue_size = 1)

    rospy.loginfo("Ignore message");
    graspStatus = 0;
    rospy.sleep(2);
    msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor);
    rospy.loginfo("Ignored force: %f",msg.Rf);

    rospy.loginfo("Before grasp");
    graspStatus = 1;
    rospy.sleep(2);
    msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor);
    rfBeforeGrasp = msg.Rf;
    rospy.loginfo("Before grasp: %f",msg.Rf);

    rospy.loginfo("After grasp");
    graspStatus = 2;
    rospy.sleep(2);
    msg = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor);
    rfAfterGrasp = msg.Rf;
    rospy.loginfo("After grasp: %f",msg.Rf);

    rfDiff = rfAfterGrasp - rfBeforeGrasp;
    rospy.loginfo("Rf Diff: %f", rfDiff)

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    listener()

def checkGrasp(msgBeforeGrasp, msgAfterGrasp, expectedWeight = 0):
    #TODO
