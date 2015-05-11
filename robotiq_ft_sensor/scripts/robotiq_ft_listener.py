#!/usr/bin/env python
import rospy
from robotiq_force_torque_sensor.msg import ft_sensor
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.loginfo("##STARTING##");
    rospy.init_node('robotiq_ft_listener_py', anonymous=True)

    #rospy.Subscriber("robotiq_force_torque_sensor", ft_sensor, callback, queue_size = 1)

    
    rospy.loginfo("Before grasp");
    rospy.sleep(2);
    msgBeforeGrasp = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor);
    rospy.loginfo("Before grasp: %f",msgBeforeGrasp.Rf);

    rospy.loginfo("After grasp");
    graspStatus = 2;
    rospy.sleep(2);
    msgAfterGrasp = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor);
    rospy.loginfo("After grasp: %f",msgAfterGrasp.Rf);

    checkGrasp(msgBeforeGrasp, msgAfterGrasp);
    rospy.loginfo("##DONE##");


if __name__ == '__main__':
    listener()

def checkGrasp(msgBeforeGrasp, msgAfterGrasp, expectedWeight = 0):

    #For now will use only Resultant for Rf from received msg, though the msg has all the sensor information
    threhsold = 10;
    error = 2;

    #diffRf before and after grasp to use for computation
    diffRf = msgAfterGrasp.Rf - msgBeforeGrasp.Rf;

    #if no weight is sent will use threshold to make decision, else will use value within error.
    if(expectedWeight == 0):

        #if rfDiff > threshold too much weight, if < 0, anamoly
        if(rfDiff > threshold):
            rospy.loginfo("Looks like I'm carrying too much weight");
        elif(rfDiff < 0):
            rospy.loginfo("Anamoly. After weight < Before weight");
        else:
            rospy.loginfo("Grasp looks good");

    else:
        #if rfDiff within error of weight, good, > weight, too heavy, < weight, too light

        if((rfDiff > expectedWeight - error) and (rfDiff < expectedWeight + error)):
            rospy.loginfo("Grasp looks good"):
        elif(rfDiff <= expectedWeight - error):
            rospy.loginfo("Less than expected weight. Haven't picked up object");
        elif(rfDiff >= expectedWeight + error):
            rospy.loginfo("More than expected weight. Picked the wrong object or hit something");

