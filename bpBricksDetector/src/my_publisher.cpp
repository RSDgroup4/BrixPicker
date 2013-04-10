/* my_publisher.cpp
 *
 *  Created on: Feb 17, 2012
 *      Author: klaus
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvwimage.h>

namespace enc = sensor_msgs::image_encodings;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

 cv_bridge::CvImagePtr cv_ptr;
 //image.copyto(img_ptr->image);
 cv_ptr->image = cv::imread(argv[1]);
 //cv::Mat img = cv::imread(argv[1]);
 //cv_ptr->image = img;
 //cv_ptr = SubscriberTools::matToImage(cv::Mat img)
 //cv_ptr = cv_bridge::toCvCopy(img, CV_8UC3);
 sensor_msgs::ImagePtr ros_image;
 //ros_image = cv_ptr->toImageMsg();
  //sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");

 // cv::WImageBuffer3_b image( cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR) );
 // sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");

  ros::Rate loop_rate(0.1);
  while (nh.ok()) {
    pub.publish(ros_image);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
