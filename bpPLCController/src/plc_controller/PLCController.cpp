/*************************************************************************************
# File:     PLCController.cpp
# Purpose:  Create a interface node to handle communication with the PLC
# Project:  RSD - group 4
# Author:   Jeppe Pedersen <jepe009@student.sdu.dk>
# Created:  2013/02/18 - Jeppe Pedersen
# Version:  0.1 Initial version
*************************************************************************************/

#include "PLCController.h"

#define RECIEVE_DATA 128
#define SAFETY_MODE 1
#define GRIPPER_SENSOR 2
#define BELT 4

using namespace std;

PLCController::PLCController()
{
	  isBeltOn = false;
	  isGripperSensorActive = false;
	  isSafetyHigh = false;
	  setBelt = false;
	  setSafety = false;
}

bool PLCController::commandServiceHandler(	bpPLCController::plc_command::Request  &req,
         	 	 	 	 	 	 	 	 	bpPLCController::plc_command::Response &res)
{
	switch (req.command_number) {

			case bpPLCController::plc_command::Request::START_BELT:
				ROS_INFO("PLC service: Start belt");
				setBelt = true;
				res.result = true;
				break;
			case bpPLCController::plc_command::Request::STOP_BELT:
				ROS_INFO("PLC service: Stop belt");
				setBelt = false;
				res.result = true;
				break;
			case bpPLCController::plc_command::Request::TOGGLE_EMERGENCY_STOP:
				ROS_INFO("PLC service: Toggle emergency stop");
				setSafety = true;
				res.result = true;
				break;
			case bpPLCController::plc_command::Request::RELEASE_EMERGENCY_STOP:
				ROS_INFO("PLC service: Toggle emergency stop");
				setSafety = false;
				res.result = true;
				break;
			case bpPLCController::plc_command::Request::GET_EMERGENCY_STOP_STATUS:
				ROS_INFO("PLC service: Get emergency stop status");
				res.result = isSafetyHigh;
				res.value = isSafetyHigh;
				break;
			case bpPLCController::plc_command::Request::GET_GRIPPER_SENSOR_STATUS:
				res.result = isGripperSensorActive;
				res.value = isGripperSensorActive;
				break;
			case bpPLCController::plc_command::Request::GET_BELT_STATUS:
				res.result = isBeltOn;
				res.value = isBeltOn;
				break;
			default:
				ROS_ERROR("PLCController service called with unknown command number");
				break;
	}
	return true;
}

void PLCController::recieveSerialDataHandler(const std_msgs::ByteMultiArrayConstPtr& msg)
{
	ROS_INFO("BELT: %d, Gripper: %d, SAFETY: %d", msg->data[0] & BELT, msg->data[0] & GRIPPER_SENSOR, msg->data[0] & SAFETY_MODE);
	if (msg->data[0] & BELT)
		isBeltOn = true;
	else
		isBeltOn = false;
	if (msg->data[0] & GRIPPER_SENSOR)
		isGripperSensorActive = true;
	else
		isGripperSensorActive = false;
	if (msg->data[0] & SAFETY_MODE)
	{
		if (isSafetyHigh == false)
		{
			isSafetyHigh = true;
			emergency_stop_msg.data = true;
			emergency_stop_publisher.publish(emergency_stop_msg);
		}
	}
	else
	{
		if (isSafetyHigh == true)
		{
			isSafetyHigh = false;
			emergency_stop_msg.data = false;
			emergency_stop_publisher.publish(emergency_stop_msg);
		}
	}

}

void PLCController::mainLoop(int loopRate)
{
	ros::Rate loop_rate(loopRate);
	while (ros::ok())
	{
		ros::spinOnce();
		std_msgs::ByteMultiArray tx_msg;
		char tx_byte = RECIEVE_DATA + setBelt*BELT + setSafety*SAFETY_MODE;
		tx_msg.data.push_back(tx_byte);
		plc_serial_publisher.publish(tx_msg);
		loop_rate.sleep();
	}
}

PLCController::~PLCController()
{
}
