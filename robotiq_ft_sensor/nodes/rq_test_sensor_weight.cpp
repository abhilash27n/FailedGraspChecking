/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/**
 * \file rq_test_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Nicolas Lauzier <nicolas@robotiq.com>
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"
#include "math.h"
#include <sstream>

//GraspStatus
//0 = ignore
//1 = before grasp
//2 = after grasp
int graspStatus = 0;

float rfBeforeGrasp;
float rfAfterGrasp;


/*void receiveCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

void reCallback(const robotiq_force_torque_sensor::ft_sensor& msg)
{

  if(graspStatus == 0){
    ROS_INFO("IGNORING MSGS");
  }
  else if(graspStatus == 1){
    rfBeforeGrasp = msg.Rf;
    ROS_INFO("Before Grasp %f", msg.Rf);
  }
  else if(graspStatus == 2){
    rfAfterGrasp = msg.Rf;
    ROS_INFO("After Grasp: %f", msg.Rf);
  }
  //ROS_INFO("HELLO FROM CALLBACK");
  //float total_force;
  //ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz);
  //total_force = (msg.Rf);
  //ROS_INFO("Forces,FX,%f,FY,%f,FZ,%f,MX,%f,MY,%f,MZ,%f,Resultant,%f,", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz,msg.Rf);
  //ROS_INFO("Resultant force: %f", total_force);

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "rq_test_sensor_weight");


  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<robotiq_force_torque_sensor::sensor_accessor>("robotiq_force_torque_sensor_acc");
  ros::Subscriber sub1 = n.subscribe("robotiq_force_torque_sensor",1,reCallback);

  robotiq_force_torque_sensor::sensor_accessor srv;


  ROS_INFO("Zeroing sensor values");
  //set force values to zero
  srv.request.command = "SET ZRO";
  if(client.call(srv)){
      ROS_INFO("ret: %s", srv.response.res.c_str());
  }


  sleep(5);

  ROS_INFO("Waiting for grasp");
  graspStatus = 0;
  //srv.request.command = "GET";
  //client.call(srv);
  ros::spinOnce();
  
  sleep(5);

  ROS_INFO("Before Grasp");
  graspStatus = 1;
  //client.call(srv);
  ros::spinOnce();

  sleep(5);

  ROS_INFO("After Grasp");
  graspStatus = 2;
  //client.call(srv);
  ros::spinOnce();
  
  ROS_INFO("Total weight: %f - %f = %f ",rfAfterGrasp,rfBeforeGrasp,rfAfterGrasp-rfBeforeGrasp);

  return 0;
}
