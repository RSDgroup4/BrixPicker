/*************************************************************************************
# File:     PLCController.h
# Purpose:  Create a interface node to handle communication with the PLC
# Project:  RSD - group 4
# Author:   Jeppe Pedersen <jepe009@student.sdu.dk>
# Created:  2013/02/18 - Jeppe Pedersen
# Version:  0.1 Initial version
*************************************************************************************/

#ifndef PLCCONTROLLER_H_
#define PLCCONTROLLER_H_

#include <iostream>
#include <string>

#include "ros/ros.h"
#include "bpMsgs/serial.h"
#include "bpPLCController/command.h"

class PLCController
{
private:

  /* private variables */

  bpMsgs::serial serial_rx_msg;

  /* private methods */

public:

  /* public variables */
  ros::Publisher plc_serial_publisher;
  ros::ServiceServer plc_service;

  /* public methods */
  PLCController();

  bool commandServiceHandler(bpPLCController::command::Request  &req,
           bpPLCController::command::Response &res);

  void recieveSerialDataHandler(const bpMsgs::serial::ConstPtr& msg);

  virtual ~PLCController();

};

#endif /* PLCCONTROLLER_H_ */
