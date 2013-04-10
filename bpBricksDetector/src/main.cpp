#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;


//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("camera::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	int b,g,r;
	cv::imshow("image processed1", cv_ptr->image);
	//Invert Image
	//Go through all the rows
	for(int i=0; i<cv_ptr->image.cols; i++)
	{
		//Go through all the columns
		for(int j=0; j<cv_ptr->image.rows; j++)
		{
			//Go through all the channels (b, g, r)
			for(int k=0; k<cv_ptr->image.channels(); k++)
			{
				//Invert the image by subtracting image data from 255				
				if (cv_ptr->image.data[j*cv_ptr->image.rows*4+i*3 + k] > 30){
					cv_ptr->image.data[j*cv_ptr->image.rows*4+i*3 + k] -= 30;
				}
				else
					cv_ptr->image.data[j*cv_ptr->image.rows*4+i*3 + k] = 0;
			}

		}

	}
	
	cv::imshow("image processed", cv_ptr->image);
	cv::waitKey(3);
	
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
	
        ros::init(argc, argv, "image_processor");
	
        ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);
	
	cv::namedWindow("image processed", CV_WINDOW_AUTOSIZE);
	
     //   image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	 image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
	cv::destroyWindow("image processed");
	
        pub = it.advertise("camera/image_processed", 1);
	
        ros::spin();
	
	ROS_INFO("camera::main.cpp::No error.");

}
