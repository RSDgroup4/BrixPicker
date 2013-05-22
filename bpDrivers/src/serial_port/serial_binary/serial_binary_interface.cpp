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
 # THIS SOFTWARE IS PROVIDED BY SØREN HUNDEVADT NIESLSEN ''AS IS'' AND ANY
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
 # File:     serialBinaryInterface.cpp
 # Purpose:  Create a interface node to handle serial communication
 # Project:  vic_interfaces
 # Author:   Søren Hundevadt Nielsen <soeni05@gmail.com>
 # Created:  Apr 29, 2011 Søren Hundevadt Nielsen, Source written
 *************************************************************************************/

#include "serial_binary_interface.h"

serialBinaryInterface::serialBinaryInterface() :
	serial_(io_), deadlineTimer_(io_) {

	keepRunning = true;
}

void serialBinaryInterface::readHandler(const boost::system::error_code& error, size_t bytes_transferred) {

	deadlineTimer_.cancel();

	if(bytes_transferred){
		b_rx_msg.data.clear();

		for(size_t i = 0; i < bytes_transferred; i++){
			//std::cout << std::hex << (int)readbuffer_[i] << " ";
			b_rx_msg.data.push_back(readbuffer_[i]);
		}

		rx_pub_.publish(b_rx_msg);
	}

	if(keepRunning){
		readSome();
	}
}

void serialBinaryInterface::waitHandler(boost::asio::serial_port& ser_port, const boost::system::error_code& error)
{
  if (error)
  {
    return;
  }
  ser_port.cancel();
}


void serialBinaryInterface::readSome() {

	if (ros::ok()) {

		boost::asio::async_read(serial_, boost::asio::buffer(readbuffer_,rx_max_chars_),boost::bind(&serialBinaryInterface::readHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

		deadlineTimer_.expires_from_now(boost::posix_time::milliseconds(rx_timeout_));
		deadlineTimer_.async_wait(boost::bind(&serialBinaryInterface::waitHandler, this, boost::ref(serial_),boost::asio::placeholders::error));

	}else{
		ROS_WARN("ROS !OK ");
	}
}

bool serialBinaryInterface::openDevice() {

	try {
		boost::asio::serial_port_base::baud_rate BAUD(baud_rate_);
		serial_.open(device_);
		serial_.set_option(BAUD);
	} catch (boost::system::system_error &e) {
		ROS_ERROR("Connection to device %s failed; %s", device_.c_str(),e.what());
		return 1;
	}

	/* kickstart the read from the serial device */
	readSome();
	boost::thread t(boost::bind(&boost::asio::io_service::run, &io_));

	return 0;
}

void serialBinaryInterface::writeHandler(const std_msgs::ByteMultiArray::ConstPtr & msg) {
	if (serial_.is_open()) {
		serial_.write_some(boost::asio::buffer(msg->data,msg->data.size()));
	}
}

serialBinaryInterface::~serialBinaryInterface() {

	keepRunning = false;

	serial_.cancel();
    serial_.close();

}
