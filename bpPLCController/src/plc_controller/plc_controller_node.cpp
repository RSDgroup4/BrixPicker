/*************************************************************************************
# File:     plc_controller_node.cpp
# Purpose:  Create a interface node to handle communication with the PLC
# Project:  RSD - group 4
# Author:   Jeppe Pedersen <jepe009@student.sdu.dk>
# Created:  2013/02/18 - Jeppe Pedersen
# Version:  0.1 Initial version
*************************************************************************************/
#include <string>

#include "ros/ros.h"

#include "PLCController.h"

int main(int argc, char **argv)
{
  /* ros messages */
  bpMsgs::serial s_rx_msg_;
  bpMsgs::serial s_tx_msg_;

  /* parameters */
  std::string plc_serial_tx_publisher_topic;
  std::string plc_serial_rx_subscriber_topic;
  std::string plc_command_service_name;

  /* initialize ros usage */
  ros::init(argc, argv, "plc_controller");
  ros::Subscriber plc_serial_subscriber;

  /* private nodehandlers */
  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  /* read parameters from ros parameter server if available otherwise use default values */
  n.param<std::string> ("plc_serial_tx_publisher_topic", plc_serial_tx_publisher_topic, "S0_rx_msg");
  n.param<std::string> ("plc_serial_rx_subscriber_topic", plc_serial_rx_subscriber_topic, "S0_tx_msg");
  n.param<std::string> ("plc_command_service_name", plc_command_service_name, "plc_command");

  PLCController pc;

  pc.plc_service = n.advertiseService(plc_command_service_name, &PLCController::commandServiceHandler, &pc);

  pc.plc_serial_publisher = nh.advertise<bpMsgs::serial> (plc_serial_tx_publisher_topic.c_str(), 20,1);

  plc_serial_subscriber = nh.subscribe<bpMsgs::serial> (plc_serial_rx_subscriber_topic.c_str(), 20, &PLCController::recieveSerialDataHandler, &pc);

  ros::spin();

  return 0;
}

