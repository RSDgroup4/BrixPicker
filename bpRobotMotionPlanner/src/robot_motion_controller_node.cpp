/*************************************************************************************
# File:     robot_motion_controller_node.cpp
# Purpose:  
# Project:  RSD - group 4
# Author:   Jeppe Pedersen <jepe009@student.sdu.dk>
# Created:  2013/02/18 - Jeppe Pedersen
# Version:  0.1 Initial version
*************************************************************************************/
#include <string>

#include "ros/ros.h"
#include "bpDrivers/command.h"
#include <math.h>

int main(int argc, char **argv)
{
  /* ros messages */

  /* parameters */
  std::string plc_serial_tx_publisher_topic;
  std::string plc_serial_rx_subscriber_topic;
  std::string plc_command_service_name;

  /* initialize ros usage */
  ros::init(argc, argv, "robot_motion_controller");

  /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("plc_serial_tx_publisher_topic", plc_serial_tx_publisher_topic, "S0_rx_msg");
  n.param<std::string> ("plc_serial_rx_subscriber_topic", plc_serial_rx_subscriber_topic, "S0_tx_msg");
  n.param<std::string> ("plc_command_service_name", plc_command_service_name, "plc_command");

  bpDrivers::commandRequest cmdGoHome;
  cmdGoHome.command_number = bpDrivers::commandRequest::SET_JOINT_CONFIGURATION;
  cmdGoHome.joint1 = 0;
  cmdGoHome.joint2 = 0;
  cmdGoHome.joint3 = 0;
  cmdGoHome.joint4 = 0;
  cmdGoHome.joint5 = 0;
  cmdGoHome.joint6 = 0;

  bpDrivers::commandRequest cmdIdle;
  cmdIdle.command_number = bpDrivers::commandRequest::SET_JOINT_CONFIGURATION;
  cmdIdle.joint1 = 0;
  cmdIdle.joint2 = 45;
  cmdIdle.joint3 = 90;
  cmdIdle.joint4 = 0;
  cmdIdle.joint5 = 45;
  cmdIdle.joint6 = 90;

  bpDrivers::commandRequest cmdPickup;
  cmdPickup.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
  cmdPickup.x = 415;
  cmdPickup.y = 0;
  cmdPickup.z = -75;
  cmdPickup.theta_x = 0;
  cmdPickup.theta_y = 180;
  cmdPickup.theta_z = 90;

  bpDrivers::commandRequest cmdDropoff;
  cmdDropoff.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
  cmdDropoff.x = 600;
  cmdDropoff.y = 0;
  cmdDropoff.z = 0;
  cmdDropoff.theta_x = 0;
  cmdDropoff.theta_y = 135;
  cmdDropoff.theta_z = 0;

  ros::Rate loop_rate(10);

  ros::ServiceClient client = n.serviceClient<bpDrivers::command>("/bpDrivers/rx60_controller/rx60_command");

  std::cout << "RobotMotionPlanner node started! - waiting for RX60 controller service" << std::endl;

  client.waitForExistence();

  std::cout << "RX60 controller service found! - Going to idle position" << std::endl;

  bpDrivers::commandRequest cmdReq;
  bpDrivers::commandResponse cmdRes;

  int move = 0;

  while (ros::ok())
  {
    ros::spinOnce();

    // check if robot is settled
    cmdReq.command_number = bpDrivers::commandRequest::IS_SETTLED;
    client.call(cmdReq,cmdRes);

    if (cmdRes.is_settled)
    {
        cmdReq.command_number = bpDrivers::commandRequest::GET_TOOL_FLANGE_POSITION;
        client.call(cmdReq,cmdRes);
        std::cout << "Robot settled!! at pos: X: " << cmdRes.x << " Y: " << cmdRes.y << " Z: " << cmdRes.z << std::endl;
        switch (move) {
			case 0:
				  client.call(cmdIdle,cmdRes);
				  move = 1;
				break;
			case 1:
				  ros::Duration(0.5).sleep();
				  cmdPickup.z = -75;
				  cmdPickup.y = -200 + (double)rand()/((double)RAND_MAX/(200+200));
				  client.call(cmdPickup,cmdRes);
				  cmdPickup.z = -150;
				  client.call(cmdPickup,cmdRes);
				  move = 2;
				break;
			case 2:
				  ros::Duration(0.5).sleep();
				  cmdPickup.z = -75;
				  client.call(cmdPickup,cmdRes);
				  client.call(cmdDropoff,cmdRes);
				  move = 1;
				break;
			default:
				break;
		}
    }
    else
    {
        cmdReq.command_number = bpDrivers::commandRequest::GET_TOOL_FLANGE_POSITION;
        client.call(cmdReq,cmdRes);
        std::cout << "Robot Moving!! at pos: X: " << cmdRes.x << " Y: " << cmdRes.y << " Z: " << cmdRes.z << std::endl;
    }


    loop_rate.sleep();
  }

  return 0;
}

