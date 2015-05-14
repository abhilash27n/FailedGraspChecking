#!/usr/bin/env python
import rospy
from robotiq_force_torque_sensor.msg import ft_sensor
    
# expectedWeight in "GRAMS"    
# returns 1 on success, 0 on failed
def checkGrasp(msgBeforeGrasp, msgAfterGrasp, expectedWeight = 0):


    #For now will use only Resultant for Rf from received msg, though the msg has all the sensor information
    threshold = 10;
    error = 2;

    #getExpectedWeight from given object
    #TODO here or from where function is called?

    #rfDiff before and after grasp to use for computation
    rfDiff = msgAfterGrasp.Rf - msgBeforeGrasp.Rf;

    #convert expectedWeight to Newtons
    rospy.loginfo("Expected weight in grams: %f", expectedWeight);
    expectedForce = expectedWeight * 0.00980665;
    rospy.loginfo("Expected force in newtons: %f", expectedForce);

    #if no object is sent will use threshold to make decision, else will use converted force within errors in measurement.
    if(expectedForce == 0):

        #if rfDiff > threshold too much force, if < 0, anamoly
        if(rfDiff > threshold):
            rospy.loginfo("Without weight info: Looks like I'm carrying too much weight");
            return 0;
        elif(rfDiff < 0 - error):
            rospy.loginfo("Without weight info: Anamoly. After weight < Before weight");
            return 0;
        else:
            rospy.loginfo("Without weight info: Grasp looks good");
            return 1;

    else:
        #if rfDiff within error of computed force, good; > weight, too heavy; < weight, too light

        if((rfDiff > expectedForce - error) and (rfDiff < expectedForce + error)):
            rospy.loginfo("With weight info: Grasp looks good");
            return 1;
        elif(rfDiff <= expectedForce - error):
            rospy.loginfo("With weight info: Less than expected weight. Haven't picked up object or picked lighter object");
            return 0;
        elif(rfDiff >= expectedForce + error):
            rospy.loginfo("With weight info: More than expected weight. Picked the heavier object or hit something");
            return 0;



def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('robotiq_ft_listener_py', anonymous=True)

    #rospy.Subscriber("robotiq_force_torque_sensor", ft_sensor, callback, queue_size = 1)

    rospy.loginfo("##STARTING##");
    rospy.loginfo("Before grasp");
    rospy.sleep(2);
    msgBeforeGrasp = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor);
    rospy.loginfo("Before grasp: %f",msgBeforeGrasp.Rf);

    rospy.loginfo("After grasp");
    graspStatus = 2;
    rospy.sleep(10);
    msgAfterGrasp = rospy.wait_for_message("robotiq_force_torque_sensor", ft_sensor);
    rospy.loginfo("After grasp: %f",msgAfterGrasp.Rf);

    checkGrasp(msgBeforeGrasp, msgAfterGrasp);
    checkGrasp(msgBeforeGrasp, msgAfterGrasp, 10);
    rospy.loginfo("##DONE##");


if __name__ == '__main__':
    listener()

