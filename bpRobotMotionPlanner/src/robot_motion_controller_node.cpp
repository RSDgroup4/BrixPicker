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
#include "bpMsgs/robot_pick.h"
#include "bpPLCController/plc_command.h"
#include "std_msgs/Bool.h"

#define M2MM 1000.0

enum state {
	idle,
	wait_for_wait_position,
	go_down_to_pick_brick,
	wait_for_grip_position,
	wait_for_brick,
	go_to_delivery_position,
	wait_for_delivery_position,
	go_to_safe_position,
	wait_for_safe_position,
	failed,
	succeded,
	emergency_stop
};

double robot_min_x = 0.260;
double robot_max_x = 0.440;
double robot_center_x = 0.350;
double robot_min_y = -0.400;
double robot_max_y = 0.400;
double belt_speed = 0;
double time_to_pick = 1.5;

state robot_state;
ros::ServiceClient robot_client;
bpDrivers::commandRequest cmdReq;
bpDrivers::commandResponse cmdRes;

std::vector<bpMsgs::robot_pick> pending_bricks;

std::vector<bpDrivers::commandRequest> cmdOrder;
std::vector<bpDrivers::commandRequest> cmdOrderDropoff;
bpDrivers::commandRequest cmdIdle;
bpDrivers::commandRequest cmdShutdown;
bpDrivers::commandRequest cmdWaitPosition;
bpDrivers::commandRequest cmdGripPosition;
bpDrivers::commandRequest cmdCloseGripper;
bpDrivers::commandRequest cmdOpenGripper;
bpDrivers::commandRequest cmdResetMotion;



void setKnownConfs(void)
{
	cmdIdle.command_number = bpDrivers::commandRequest::SET_JOINT_CONFIGURATION;
	cmdIdle.joint1 = -1.53;
	cmdIdle.joint2 = 27.62;
	cmdIdle.joint3 = 116.53;
	cmdIdle.joint4 = 0;
	cmdIdle.joint5 = 35.93;
	cmdIdle.joint6 = -1.6;
//	cmdIdle.joint1 = -7.67;
//	cmdIdle.joint2 = 19.28;
//	cmdIdle.joint3 = 100.87;
//	cmdIdle.joint4 = 0;
//	cmdIdle.joint5 = 59.84;
//	cmdIdle.joint6 = -7.67;

	cmdResetMotion.command_number = bpDrivers::commandRequest::RESET_MOTION;

	cmdShutdown.command_number = bpDrivers::commandRequest::SET_JOINT_CONFIGURATION;
	cmdShutdown.joint1 = 0;
	cmdShutdown.joint2 = 0;
	cmdShutdown.joint3 = 0;
	cmdShutdown.joint4 = 0;
	cmdShutdown.joint5 = 0;
	cmdShutdown.joint6 = 0;

	bpDrivers::commandRequest temp;
	temp.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	temp.x = 0.480*M2MM;
	temp.y = -0.066*M2MM;
	temp.z = -0.072*M2MM;
	temp.theta_x = 0;
	temp.theta_y = 150;
	temp.theta_z = 0;

	cmdOrder.push_back(temp);

	temp.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	temp.x = 0.480*M2MM;
	temp.y = 0.030*M2MM;
	temp.z = -0.072*M2MM;
	temp.theta_x = 0;
	temp.theta_y = 150;
	temp.theta_z = 0;

	cmdOrder.push_back(temp);

	temp.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	temp.x = 0.480*M2MM;
	temp.y = 0.130*M2MM;
	temp.z = -0.072*M2MM;
	temp.theta_x = 0;
	temp.theta_y = 150;
	temp.theta_z = 0;

	cmdOrder.push_back(temp);

	temp.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	temp.x = 0.544*M2MM;
	temp.y = -0.066*M2MM;
	temp.z = -0.072*M2MM;
	temp.theta_x = 0;
	temp.theta_y = 150;
	temp.theta_z = 0;

	cmdOrderDropoff.push_back(temp);

	temp.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	temp.x = 0.544*M2MM;
	temp.y = 0.030*M2MM;
	temp.z = -0.072*M2MM;
	temp.theta_x = 0;
	temp.theta_y = 150;
	temp.theta_z = 0;

	cmdOrderDropoff.push_back(temp);

	temp.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	temp.x = 0.544*M2MM;
	temp.y = 0.130*M2MM;
	temp.z = -0.072*M2MM;
	temp.theta_x = 0;
	temp.theta_y = 150;
	temp.theta_z = 0;

	cmdOrderDropoff.push_back(temp);

	cmdCloseGripper.command_number = bpDrivers::commandRequest::SET_VALVE1;
	cmdCloseGripper.output_state = true;

	cmdOpenGripper.command_number = bpDrivers::commandRequest::SET_VALVE1;
	cmdOpenGripper.output_state = false;

}

void brickHandler(bpMsgs::robot_pick msg)
{
	pending_bricks.push_back(msg);
	belt_speed = msg.belt_speed;
	ROS_INFO("Brick received. Bricks pending: %d", pending_bricks.size());
}

void emergencyStopHandler(std_msgs::Bool msg)
{
	if (msg.data)
	{
		pending_bricks.clear();
		robot_client.call(cmdResetMotion,cmdRes);
		robot_state = emergency_stop;
	}
	else
	{
		robot_client.call(cmdOpenGripper,cmdRes);
		robot_state = idle;
	}
}

bool calcPositions(bpMsgs::robot_pick& brick)
{
	double x = brick.x;
	double angle = brick.angle;

	double y = brick.y + belt_speed * ( ros::Time::now().toSec() - brick.header.stamp.toSec() );

	if ((y + time_to_pick * belt_speed) < robot_min_y)
	{
		brick.header.stamp = ros::Time::now() + ros::Duration((robot_min_y - y) / belt_speed);
		y = robot_min_y;
	}
	else
	{
		y += time_to_pick * belt_speed;
		brick.header.stamp = ros::Time::now() + ros::Duration(time_to_pick);
	}

	if (x < robot_min_x || x > robot_max_x || y < robot_min_y || y > robot_max_y)
		return false;

	ROS_INFO("X: %f, Y: %f, Time: %f", x, y, brick.header.stamp.toSec());

	// Calculate y-Angle offset
	double theta_y_offset = (x - 0.35) / (robot_max_x - robot_min_x)*2.0*12.0;
	if (theta_y_offset < 0)
		theta_y_offset = -180.0 - theta_y_offset;
	else
		theta_y_offset = 180.0 - theta_y_offset;

	// Calculate Z
	double z_pickup = (1.2963*pow(x,2.0) - 0.8685*x - 0.0738);
	if (y > 0.1)
		z_pickup += y * 0.015;
	

	ROS_INFO("Theta: %f, X: %f, Y: %f, Z: %f", theta_y_offset, x, y, z_pickup);

	cmdWaitPosition.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	cmdWaitPosition.x = x*M2MM;
	cmdWaitPosition.y = y*M2MM;
	cmdWaitPosition.z = (z_pickup + 0.015)*M2MM;
	cmdWaitPosition.theta_x = 0;
	cmdWaitPosition.theta_y = theta_y_offset;
	cmdWaitPosition.theta_z = brick.angle;

	cmdGripPosition.command_number = bpDrivers::commandRequest::SET_TOOL_FLANGE_CARTESIAN_POSITION;
	cmdGripPosition.x = x*M2MM;
	cmdGripPosition.y = y*M2MM;
	cmdGripPosition.z = (z_pickup - 0.006)*M2MM;
	cmdGripPosition.theta_x = 0;
	cmdGripPosition.theta_y = theta_y_offset;
	cmdGripPosition.theta_z = brick.angle;

	return true;
}

int main(int argc, char **argv)
{
	pending_bricks = std::vector<bpMsgs::robot_pick>();

	/* ros messages */

	/* parameters */
	std::string brick_subscriber_topic;
	std::string brick_publisher_topic;
	std::string emergency_stop_subscriber_topic;
	int loop_rate_param;

	/* initialize ros usage */
	ros::init(argc, argv, "robot_motion_controller");
	ros::Publisher brick_publisher;
	ros::Subscriber brick_subscriber;
	ros::Subscriber emergency_stop_subscriber;
	bpPLCController::plc_command plcCommand;
	plcCommand.request.command_number = bpPLCController::plc_command::Request::GET_GRIPPER_SENSOR_STATUS;

	/* private nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<std::string> ("brick_subscriber_topic", brick_subscriber_topic, "/bpRobotMotionController/brick_command_topic");
	n.param<std::string> ("brick_publisher_topic", brick_publisher_topic, "/bpRobotMotionController/brick_response_topic");
	n.param<std::string> ("emergency_stop_subscriber_topic", emergency_stop_subscriber_topic, "/emergency_stop");
	n.param<int> ("loop_rate", loop_rate_param, 1);


	brick_publisher = nh.advertise<bpMsgs::robot_pick> (brick_publisher_topic.c_str(), 10);
	brick_subscriber = nh.subscribe<bpMsgs::robot_pick> (brick_subscriber_topic.c_str(), 20, brickHandler);
	emergency_stop_subscriber = nh.subscribe<std_msgs::Bool> (emergency_stop_subscriber_topic.c_str(),20,emergencyStopHandler);

	setKnownConfs();

	robot_client = n.serviceClient<bpDrivers::command>("/bpDrivers/rx60_controller/rx60_command");
	ros::ServiceClient plc_client = n.serviceClient<bpPLCController::plc_command>("/plc_controller/plc_command");

	std::cout << "RobotMotionPlanner node started! - waiting for RX60 controller service" << std::endl;
	robot_client.waitForExistence();
	std::cout << "RX60 controller service found! - Going to idle position" << std::endl;

	robot_state = idle;

	// Go to Idle position
	robot_client.call(cmdIdle,cmdRes);
	robot_client.call(cmdOpenGripper, cmdRes);

	bpMsgs::robot_pick brickToPick;
	bpMsgs::robot_pick returnMessage;

	ros::Rate loop_rate(loop_rate_param);
	while (ros::ok())
	{
		ros::spinOnce();

		switch (robot_state) {
			case idle:
				if (pending_bricks.size() > 0)
				{
					brickToPick = pending_bricks[0];
					pending_bricks.erase(pending_bricks.begin());
					if (!calcPositions(brickToPick))
					{
						// Cant reach brick
						robot_state = failed;
					}
					else
					{
						robot_client.call(cmdWaitPosition,cmdRes);
						ros::Duration(0.05).sleep();
						robot_state = wait_for_wait_position;
					}
				}
				else
				{
					robot_client.call(cmdIdle,cmdRes);
				}
				break;
			case wait_for_wait_position:
				cmdReq.command_number = bpDrivers::commandRequest::IS_SETTLED;
				robot_client.call(cmdReq,cmdRes);
				if (cmdRes.is_settled)
				{
					robot_state = go_down_to_pick_brick;
				}
				break;

			case go_down_to_pick_brick:
				if (brickToPick.header.stamp.toSec() < (ros::Time::now().toSec()) )
				{
					robot_state = failed;
				}
				else
				{
					ros::Duration( (brickToPick.header.stamp.toSec() - ros::Time::now().toSec()) - 0.20).sleep();
					robot_client.call(cmdGripPosition,cmdRes);
					robot_state = wait_for_brick;//wait_for_grip_position;
					ros::Duration(0.05).sleep();
				}
				break;

			case wait_for_grip_position:
				cmdReq.command_number = bpDrivers::commandRequest::IS_SETTLED;
				robot_client.call(cmdReq,cmdRes);
				if (cmdRes.is_settled)
				{
					robot_state = wait_for_brick;
				}
				break;

			case wait_for_brick:
				if (brickToPick.header.stamp.toSec() < (ros::Time::now().toSec() - 0.0) )
				{
					robot_client.call(cmdCloseGripper,cmdRes);
					ros::Duration(0.3).sleep();
					plc_client.call(plcCommand);
					ROS_INFO("Gripper1: %d", plcCommand.response.result);
					if (!plcCommand.response.result)
					{
						robot_state = go_to_delivery_position;
					}
					else
					{
						robot_state = failed;
						robot_client.call(cmdWaitPosition,cmdRes);
						robot_client.call(cmdOpenGripper,cmdRes);
					}
				}
				break;

			case go_to_delivery_position:
				robot_client.call(cmdWaitPosition,cmdRes);				
				robot_client.call(cmdOrder[brickToPick.order-1],cmdRes);
				robot_client.call(cmdOrderDropoff[brickToPick.order-1],cmdRes);
				robot_state = wait_for_delivery_position;
				ros::Duration(0.1).sleep();
				break;

			case wait_for_delivery_position:
				cmdReq.command_number = bpDrivers::commandRequest::IS_SETTLED;
				robot_client.call(cmdReq,cmdRes);
				if (cmdRes.is_settled)
				{
					plc_client.call(plcCommand);
					ROS_INFO("Gripper2: %d", plcCommand.response.result);
					if (!plcCommand.response.result)
					{
						robot_client.call(cmdOpenGripper,cmdRes);
						ros::Duration(0.15).sleep();
						robot_state = go_to_safe_position;
					}
					else
					{
						robot_client.call(cmdOpenGripper,cmdRes);
						robot_client.call(cmdOrder[brickToPick.order-1],cmdRes);
						ros::Duration(0.1).sleep();
						robot_state = failed;
					}
				}
				break;

			case go_to_safe_position:
				robot_client.call(cmdOrder[brickToPick.order-1],cmdRes);
				ros::Duration(0.1).sleep();
				robot_state = wait_for_safe_position;
				break;

			case wait_for_safe_position:
				cmdReq.command_number = bpDrivers::commandRequest::IS_SETTLED;
				robot_client.call(cmdReq,cmdRes);
				if (cmdRes.is_settled)
				{
					robot_state = succeded;
				}
				break;

			case succeded:
				returnMessage = brickToPick;
				returnMessage.succes = true;
				brick_publisher.publish(returnMessage);
				robot_state = idle;
				break;

			case failed:
				ROS_INFO("ERROR cant reach brick");
				returnMessage = brickToPick;
				returnMessage.succes = false;
				brick_publisher.publish(returnMessage);
				robot_state = idle;
				break;

			case emergency_stop:
				break;

			default:
				break;
		}
		// check if robot is settled
		//cmdReq.command_number = bpDrivers::commandRequest::IS_SETTLED;
		//client.call(cmdReq,cmdRes);

		//if (cmdRes.is_settled);

		loop_rate.sleep();
	}

	return 0;
}

