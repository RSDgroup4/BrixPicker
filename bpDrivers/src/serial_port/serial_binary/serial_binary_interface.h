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
 # 4. Neither the name of the <organization> nor the
 #    names of its contributors may be used to endorse or promote products
 #    derived from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY SØREN HUNDEVADT NIELSEN ''AS IS'' AND ANY
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
 # File:     serial_binary_interfaceh.h
 # Purpose:  Create a interface node to handle binary serial communication
 # Project:  fmCSP/Serial_Port/serial_binary_interface.h
 # Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>
 # Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
 *************************************************************************************/

#ifndef SERIALIBINNTERFACE_H_
#define SERIALIBINNTERFACE_H_

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <iomanip>

#include "ros/ros.h"
#include "std_msgs/ByteMultiArray.h"
#include <boost/asio/deadline_timer.hpp>

class serialBinaryInterface {
private:

	/* private variables */
	boost::asio::io_service io_;
	boost::asio::serial_port serial_;
	boost::asio::deadline_timer deadlineTimer_;

	bool keepRunning;

	char readbuffer_[1024];

	std_msgs::ByteMultiArray b_rx_msg;
	//fmMsgs::serial serial_rx_msg;

	/* private methods */
	void readSome();
	void readHandler(const boost::system::error_code& error,
			size_t bytes_transferred);
	void waitHandler(boost::asio::serial_port& ser_port,
			const boost::system::error_code& error);

public:

	/* public variables */
	ros::Publisher rx_pub_;
	ros::Subscriber tx_sub_;

	/* ros parameters */
	std::string device_, rx_, tx_;
	int baud_rate_, rx_timeout_, rx_max_chars_;

	/* public methods */
	serialBinaryInterface();
	~serialBinaryInterface();
	bool openDevice();
	void writeHandler(const std_msgs::ByteMultiArray::ConstPtr & msg);

};

#endif /* SERIALIBINNTERFACE_H_ */
