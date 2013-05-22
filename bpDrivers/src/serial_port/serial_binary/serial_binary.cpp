/*************************************************************************************
 # Copyright (c) 2011, Søren Hundevadt Nielsen
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 # 1. Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 # 2. Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 # 3. All advertising materials mentioning features or use of this software
 #    must display the following acknowledgement:
 #    This product includes software developed by the University of Southern Denmark.
 # 4. Neither the name of the University of Southern Denmark nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY SØREN HUNDEVADT NIELSEN "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL SØREN HUNDEVADT NIELSEN BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************************
 # File:     serial_binary.cpp
 # Purpose:  Create a interface node to handle binary serial communication
 # Project:  fmCSP/Serial_Port/serial_binary
 # Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>
 # Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
 *************************************************************************************/
#include <string>

#include "ros/ros.h"
#include "serial_binary_interface.h"

int main(int argc, char **argv) {

	/* initialize ros usage */
	ros::init(argc, argv, "serial_binary");

	/* ros nodehandlers */
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	/* create serialBinaryInterface instance */
	serialBinaryInterface sbi;

	/* read parameters from ros parameter server if available otherwise use default values */
	n.param<std::string> ("device", sbi.device_, "/dev/ttyUSB0");
	n.param<std::string> ("rx", sbi.rx_, "/bpDrivers/plc_rx");
	n.param<std::string> ("tx", sbi.tx_, "/bpDrivers/plc_tx");
	n.param<int> ("baud_rate", sbi.baud_rate_, 19200);
	n.param<int> ("rx_timeout", sbi.rx_timeout_, 100);
	n.param<int> ("rx_max_chars", sbi.rx_max_chars_, 10);


	sbi.rx_pub_ = nh.advertise<std_msgs::ByteMultiArray> (sbi.rx_.c_str(), 20,1);
	if(!sbi.openDevice()){

		sbi.tx_sub_ = nh.subscribe<std_msgs::ByteMultiArray> (sbi.tx_.c_str(), 20, &serialBinaryInterface::writeHandler, &sbi);
		ros::spin();
	}

	return 0;
}

