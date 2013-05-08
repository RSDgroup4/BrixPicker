/* my_publisher.cpp
 *
 *  Created on: Feb 17, 2012
 *      Author: klaus
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <bpMsgs/brick.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>

namespace enc = sensor_msgs::image_encodings;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;






  ros::Rate loop_rate(0.1);
  while (nh.ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }
}

