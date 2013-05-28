#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <bpMsgs/brick.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>

using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

#define drawCross( center, color, d, drawing ) \
line(drawing, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line(drawing, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 );

bool imgshow = true;
double oldarea = 0;
int max_thresh = 255;
RNG rng(12345);
// High threshold values
int huered(255), 	saturationred(255), 	valuered(255);
int hueyellow(48), 	saturationyellow(255), 	valueyellow(255);
int hueblue(170), 	saturationblue(255), 	valueblue(106);
// low threshold values
int hueredL(0), 	saturationredL(124), 	valueredL(68);
int hueyellowL(18), saturationyellowL(68), 	valueyellowL(158);
int hueblueL(71),	saturationblueL(90),	valueblueL(29);

double pixel_m = 2596;

int counter = 0;
int brick_id = 0;

vector< vector<bpMsgs::brick> > bricks;

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher brick_pub;

void HSVToBW(const Mat &imgSrc, Mat &imgDst, int a)
{
	imgDst = Mat(imgSrc.rows, imgSrc.cols, CV_8U);
	// Iterate over the image line by line
	for(int i = 0 ; i < imgSrc.rows ; i++ )
	  {
		// Iterate over the elements in the current line
	    for(int j = 0 ; j < imgSrc.cols ; j++ )
    	{
	    	if (j < 0 || j > 640) {
	    				imgDst.data[i*imgDst.cols+j] = 0;
	    	}
	    	else {
			int bB =  imgSrc.data[i*imgSrc.cols*3+j*3 + 0];	// Blue component
			int bG =  imgSrc.data[i*imgSrc.cols*3+j*3 + 1];	// Green component
			int bR =  imgSrc.data[i*imgSrc.cols*3+j*3 + 2];	// Red component

			int iMax, iMin;
			if (bB < bG)
			{
				if (bB < bR)
				{
					iMin = bB;
					if (bR > bG)
					{
						iMax = bR;
					}
					else
					{
						iMax = bG;
					}
				}
				else
				{
					iMin = bR;
					iMax = bG;
				}
			}
			else
			{
				if (bG < bR)
				{
					iMin = bG;
					if (bB > bR)
					{
						iMax = bB;
					}
					else
					{
						iMax = bR;
					}
				}
				else
				{
					iMin = bR;
					iMax = bB;
				}
			}

			float iDelta = iMax - iMin;
						int bS;
						int	bV = iMax;
						float fH;
						if (iMax != 0) {			// Make sure its not pure black.
							bS = (int)(0.5f + (iDelta / iMax) * 255.0f);	// Saturation.
								float ANGLE_TO_UNIT = 1.0f / (6.0f * iDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
								if (iMax == bR) {		// between yellow and magenta.
									fH = (bG - bB) * ANGLE_TO_UNIT;
								}
								else if (iMax == bG) {		// between cyan and yellow.
									fH = (2.0f/6.0f) + ( bB - bR ) * ANGLE_TO_UNIT;
								}
								else {				// between magenta and cyan.
									fH = (4.0f/6.0f) + ( bR - bG ) * ANGLE_TO_UNIT;
								}
								// Wrap outlier Hues around the circle.
								if (fH < 0.0f)
									fH += 1.0f;
								if (fH >= 1.0f)
									fH -= 1.0f;
							}
							else
							{
								fH = 0;
							}

						int bH = (int)(0.5f + fH * 255.0f);

									if (bH > 255)
										bH = 255;
									if (bH < 0)
										bH = 0;
									if (bS > 255)
										bS = 255;
									if (bS < 0)
										bS = 0;
									if (bV > 255)
										bV = 255;
									if (bV < 0)
										bV = 0;

				if ((bS > saturationredL && bS <= saturationred) && (bH > hueredL && bH <= huered) && (bV > valueredL && bV <= valuered)) {
					imgDst.data[i*imgDst.cols+j] = 255;
				}
				else if ((bS > saturationyellowL && bS <= saturationyellow) && (bH > hueyellowL && bH <= hueyellow) && (bV > valueyellowL && bV <= valueyellow)) {
					imgDst.data[i*imgDst.cols+j] = 255;
				}
				else if ((bS > saturationblueL && bS <= saturationblue) && (bH > hueblueL && bH <= hueblue) && (bV > valueblueL && bV <= valueblue)) {
					imgDst.data[i*imgDst.cols+j] = 255;
				}
				else {
					imgDst.data[i*imgDst.cols+j] = 0;
				}


		}
	        }
	  }

}

void callback( int, void* )
{

}

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	counter++;
	if (counter > 2)
		counter = 0;
	else
		return;

	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("camera::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	Mat imgdest;

	HSVToBW(cv_ptr->image,imgdest,2);

	GaussianBlur(imgdest, imgdest, Size(3,3), 1.5, 1.5);
	if (imgshow) {
		cv::imshow("grey", imgdest);
	}
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	double area;

	  /// Find contours
	  findContours( imgdest, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	 vector<RotatedRect> minRect( contours.size() );

	  for( int i = 0; i < contours.size(); i++)
	  {
		  minRect[i] = minAreaRect( Mat(contours[i]) );
		  area = contourArea(contours[i],false);
		   if (area < 500 || area > 30000){
			   contours.erase(contours.begin()+i);
			   minRect.erase(minRect.begin()+i);
			   i--;
		   }
		   else {

				double difference_x, difference_y;
				difference_x = (minRect[i].center.x-320)/pixel_m;
				difference_y = (480 - minRect[i].center.y)/pixel_m;

				bpMsgs::brick tmpBrick;
				tmpBrick.header.stamp.sec = original_image->header.stamp.sec;
				tmpBrick.header.stamp.nsec = original_image->header.stamp.nsec;
				tmpBrick.x = difference_x;
				tmpBrick.y = difference_y;

				tmpBrick.angle = minRect[i].angle;

				if (minRect[i].size.width > minRect[i].size.height)
					tmpBrick.angle += 90.0;

				Scalar color;
				if (area > 1100 && area < 1900) {
				   tmpBrick.type = 2;
				   if (tmpBrick.angle < -45)
					   tmpBrick.angle += 90.0;
				   else if (tmpBrick.angle > 45)
					   tmpBrick.angle -= 90.0;
				   color = Scalar( 255, 0, 0 );
				}
				else if (area > 2500 && area < 3300){
				   tmpBrick.type = 1;
				   color = Scalar( 0, 0, 255 );
				}
				else if(area > 4300 && area < 6500){
				   tmpBrick.type = 3;
				   color = Scalar( 0, 255, 255 );
				}
				else{
				   tmpBrick.type = 0;
				   color = Scalar( 255, 255, 255 );
				}
				drawCross(minRect[i].center,color, 5, cv_ptr->image);
				Point2f rect_points[4];
				minRect[i].points( rect_points );
				for( int j = 0; j < 4; j++ ) {
					line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], color, 3, 8 );
				}


				ROS_INFO("Area: %ld x: %f, xp %ld,  y: %f, yp: %ld, RectOrient: %f, RectH: %f, RectW: %f, type %d", (long int)area, (float)difference_x, (long int)minRect[i].center.x, (float)difference_y, (long int)minRect[i].center.y, tmpBrick.angle, minRect[i].size.height, minRect[i].size.width, tmpBrick.type);


				bool oldBrick = false;
				for (int i = 0; i < bricks.size(); i++)
				{
					// Check if the brick is already known
					if (abs(tmpBrick.x - bricks[i].back().x) < 0.01 && ((tmpBrick.y - bricks[i].back().y) > 0.003 && (tmpBrick.y - bricks[i].back().y) < 0.02) && tmpBrick.type == bricks[i].back().type )
					{
						ROS_INFO("The same brick!");
						oldBrick = true;
						// last time we detect it - publish it!
						if (tmpBrick.y > 0.13 && bricks[i].size() >= 5)
						{
							tmpBrick.id = brick_id++;
							double tmpX = 0;
							for (int j = 0; j < bricks[i].size(); j++)
								tmpX += bricks[i][j].x;
							tmpX /= bricks[i].size();
							brick_pub.publish(tmpBrick);
							ROS_INFO("Brick published");
							bricks.erase(bricks.begin()+i);
						}
						// adding the brick to its predecessors vector
						else
						{
							bricks[i].push_back(tmpBrick);
						}

					}
					else if (ros::Time::now().toSec() - bricks[i].front().header.stamp.toSec() > 10)
					{
						bricks.erase(bricks.begin()+i);
						i--;
					}
				}
				// Check if it is a new brick
				if (tmpBrick.y < 0.13 && tmpBrick.y > 0.03 && !oldBrick)
				{
					ROS_INFO("New Brick Found!");
					vector< bpMsgs::brick > tmpBrickVec;
					tmpBrickVec.push_back(tmpBrick);
					bricks.push_back(tmpBrickVec);
				}
		   }

	  }
	cv::waitKey(3);
	
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
	pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processor");
	
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	n.param<bool>("show_images", imgshow, false);
	//Create an ImageTransport instance, initializing it with our NodeHandle.
	image_transport::ImageTransport it(nh);
	brick_pub = nh.advertise<bpMsgs::brick>("/bpBrickDetector/bricks", 10);
	if (imgshow) {
	cv::namedWindow("grey", CV_WINDOW_NORMAL);

	cv::namedWindow("TreshHigh", CV_WINDOW_NORMAL);
	cv::resizeWindow("TreshHigh", 400, 300);
	cv::namedWindow("TreshLow", CV_WINDOW_NORMAL);
	cv::resizeWindow("TreshLow", 400, 300);

	createTrackbar( "Red Hue", "TreshHigh", &huered, max_thresh, callback );
	createTrackbar( "Red Saturation", "TreshHigh", &saturationred, max_thresh, callback);
	createTrackbar( "Red Value", "TreshHigh", &valuered, max_thresh, callback);
	createTrackbar( "Yellow Hue", "TreshHigh", &hueyellow, max_thresh, callback );
	createTrackbar( "Yellow Saturation", "TreshHigh", &saturationyellow, max_thresh, callback );
	createTrackbar( "Yellow Value", "TreshHigh", &valueyellow, max_thresh, callback );
	createTrackbar( "Blue Hue", "TreshHigh", &hueblue, max_thresh, callback );
	createTrackbar( "Blue Saturation", "TreshHigh", &saturationblue, max_thresh, callback );
	createTrackbar( "Blue Value", "TreshHigh", &valueblue, max_thresh, callback );

	createTrackbar( "Red Hue", "TreshLow", &hueredL, max_thresh, callback );
	createTrackbar( "Red Saturation", "TreshLow", &saturationredL, max_thresh, callback);
	createTrackbar( "Red Value", "TreshLow", &valueredL, max_thresh, callback);
	createTrackbar( "Yellow Hue", "TreshLow", &hueyellowL, max_thresh, callback );
	createTrackbar( "Yellow Saturation", "TreshLow", &saturationyellowL, max_thresh, callback );
	createTrackbar( "Yellow Value", "TreshLow", &valueyellowL, max_thresh, callback );
	createTrackbar( "Blue Hue", "TreshLow", &hueblueL, max_thresh, callback );
	createTrackbar( "Blue Saturation", "TreshLow", &saturationblueL, max_thresh, callback );
	createTrackbar( "Blue Value", "TreshLow", &valueblueL, max_thresh, callback );
	callback(0,0);
	}

//  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	image_transport::Subscriber sub = it.subscribe("camera/usb_cam/image_raw", 1, imageCallback);
	pub = it.advertise("camera/image_processed", 1);

	ros::spin();

	if (imgshow) {
		cv::destroyWindow("grey");
		cv::destroyWindow("TreshLow");
		cv::destroyWindow("TreshHigh");
	}
}
