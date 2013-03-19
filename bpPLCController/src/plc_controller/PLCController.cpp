/*************************************************************************************
# File:     PLCController.cpp
# Purpose:  Create a interface node to handle communication with the PLC
# Project:  RSD - group 4
# Author:   Jeppe Pedersen <jepe009@student.sdu.dk>
# Created:  2013/02/18 - Jeppe Pedersen
# Version:  0.1 Initial version
*************************************************************************************/

#include "PLCController.h"

using namespace std;

PLCController::PLCController()
{
}

bool PLCController::commandServiceHandler(	bpPLCController::command::Request  &req,
         	 	 	 	 	 	 	 	 	bpPLCController::command::Response &res)
{
	bpMsgs::serial msg;
	switch (req.command_number) {
			case bpPLCController::command::Request::SET_BELT_SPEED:
				ROS_INFO("PLC service: Setting beltspeed to: %f", req.value);
				break;
			case bpPLCController::command::Request::GET_BELT_SPEED:
				ROS_INFO("PLC service: Get belt speed");
				break;
			case bpPLCController::command::Request::START_BELT:
				ROS_INFO("PLC service: Start belt");
				msg.data = "BELT_START";
				plc_serial_publisher.publish(msg);
				break;
			case bpPLCController::command::Request::STOP_BELT:
				ROS_INFO("PLC service: Stop belt");
				msg.data = "BELT_STOP";
				plc_serial_publisher.publish(msg);
				break;
			case bpPLCController::command::Request::TOGGLE_EMERGENCY_STOP:
				ROS_INFO("PLC service: Toggle emergency stop");
				break;
			case bpPLCController::command::Request::SET_BELT_DIRECTION:
				ROS_INFO("PLC service: Set belt direction");
				if (req.value == 1)
					msg.data = "BELT_FORWARD";
				else if (req.value == -1)
					msg.data = "BELT_REVERSE";
				else
					break;
				plc_serial_publisher.publish(msg);
				break;
			case bpPLCController::command::Request::GET_EMERGENCY_STOP_STATUS:
				ROS_INFO("PLC service: Get emergency stop status");
				break;

			default:
				ROS_ERROR("PLCController service called with unknown command number");
				break;
	}
	return true;
}

void PLCController::recieveSerialDataHandler(const bpMsgs::serial::ConstPtr& msg)
{
	ROS_INFO("PLCcontroller received a message from the PLC saying: %s", msg->data.c_str());
}

PLCController::~PLCController()
{
}
