#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <bpMsgs/brick.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
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

typedef float T;
typedef cv::Vec<T,3> VecT; // 3-vector
typedef cv::Mat_<VecT> ImgT; // 3-channel image
typedef cv::Mat_<T> MatT; // 1-channel container

double oldarea = 0;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
// High threshold values
int huered(255), 	saturationred(255), 	valuered(255);
int hueyellow(48), 	saturationyellow(152), 	valueyellow(255);
int hueblue(170), 	saturationblue(255), 	valueblue(106);
// low threshold values
int hueredL(211), 	saturationredL(141), 	valueredL(87);
int hueyellowL(10), saturationyellowL(16), 	valueyellowL(184);
int hueblueL(117),	saturationblueL(143),	valueblueL(45);

int huered1,saturationred1,valuered1,hueyellow1,saturationyellow1,valueyellow1,hueblue1,saturationblue1,valueblue1;
int huered1L,saturationred1L,valuered1L,hueyellow1L,saturationyellow1L,valueyellow1L,hueblue1L,saturationblue1L,valueblue1L;
int randomst = 255;

double pixel_m = 1161.0771;

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
	    	if (j < 100 || j > 500) {
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

				if ((bS > saturationred1L && bS <= saturationred1) && (bH > huered1L && bH <= huered1) && (bV > valuered1L && bV <= valuered1)) {
					imgDst.data[i*imgDst.cols+j] = 255;
				}
				else if ((bS > saturationyellow1L && bS <= saturationyellow1) && (bH > hueyellow1L && bH <= hueyellow1) && (bV > valueyellow1L && bV <= valueyellow1)) {
					imgDst.data[i*imgDst.cols+j] = 255;
				}
				else if ((bS > saturationblue1L && bS <= saturationblue1) && (bH > hueblue1L && bH <= hueblue1) && (bV > valueblue1L && bV <= valueblue1)) {
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
	huered1 = huered;
	saturationred1 = saturationred;
	valuered1 = valuered;
	hueyellow1 = hueyellow;
	saturationyellow1 = saturationyellow;
	valueyellow1 = valueyellow;
	hueblue1 = hueblue;
	saturationblue1 = saturationblue;
	valueblue1 = valueblue;

	huered1L = hueredL;
	saturationred1L = saturationredL;
	valuered1L = valueredL;
	hueyellow1L = hueyellowL;
	saturationyellow1L = saturationyellowL;
	valueyellow1L = valueyellowL;
	hueblue1L = hueblueL;
	saturationblue1L = saturationblueL;
	valueblue1L = valueblueL;

}

void filter(Mat img){
	vector<int> rect;
	int vardi = 0;
	for(int i = 3 ; i < img.rows -3 ; i++ )
	 {

			    for(int j = 3 ; j < img.cols-3 ; j++ ) {

			    		rect[0] = img.data[(i-1)*img.cols+(j-1)];
			    		rect[1] = img.data[i*img.cols+(j-1)];
			    		rect[2] = img.data[(i+1)*img.cols+(j-1)];
			    		rect[3] = img.data[(i-1)*img.cols+j];
			    		rect[4] = img.data[i*img.cols+j];
			    		rect[5] = img.data[(i+1)*img.cols+j];
			    		rect[6] = img.data[(i-1)*img.cols+(j+1)];
			    		rect[7] = img.data[i*img.cols+(j+1)];
			    		rect[8] = img.data[(i+1)*img.cols+(j+1)];
			    	for (int k = 0; k<9;k++){
			    		vardi =+ rect[k];
			    	}
			    	vardi = vardi/255;
			    	if (vardi > 6) {
			    		img.data[i*img.cols+j] = 255;
			    	}
			    	else {
			    		img.data[i*img.cols+j] = 0;
			    	}
			    }
}
}
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
	counter++;
	if (counter > 2)
		counter = 0;
	else
		return;

	bpMsgs::brick brick;
	brick.header.stamp.sec = original_image->header.stamp.sec;
	brick.header.stamp.nsec = original_image->header.stamp.nsec;

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
	Mat imgdest;

	HSVToBW(cv_ptr->image,imgdest,2);

	//filter(imgdest);

	//erode(imgdest,imgdest,Size(3,3),Point(0,0),3);
	//erode(imgdest,imgdest,Mat(),Point(-1,-1),3);
	GaussianBlur(imgdest, imgdest, Size(3,3), 1.5, 1.5);
	cv::imshow("grey", imgdest);
	  Mat canny_output;
	  vector<vector<Point> > contours;
	  vector<Vec4i> hierarchy;
	  double area;
	  int k;
	  /// Detect edges using canny
	  //Canny( src_gray, canny_output, 400, 1000, 3 );

	  //imshow( "After Canny", canny_output );

	  /// Find contours
	  findContours( /*canny_output*/ imgdest, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	  vector<Moments> _mu(contours.size());
	  vector<Point2f> _mc(contours.size());
	  vector<double> _ma(contours.size());
	  vector<RotatedRect> minRect( contours.size() );
	//  vector<RotatedRect> minEllipse( contours.size() );

	  for( int i = 0; i < contours.size(); i++)
	  {
		  minRect[i] = minAreaRect( Mat(contours[i]) );
		  area = contourArea(contours[i],false);
		   if (area < 500 || area > 30000){
			  // cout << "sletter: " << area << endl;
			   contours.erase(contours.begin()+i);
			   minRect.erase(minRect.begin()+i);
			   //area = contourArea(contours[i+1],false);
			   //contours.pop_back();
			   //cout << i << endl;
			   i--;
		   }
		   else {
				//   minEllipse[i] = fitEllipse( contours[i] );

				_mu[i] = moments( Mat(contours[i]), false );
				//  _mu[i] = moments( Mat(minRect[i]), false );
				_mc[i] = Point2f( _mu[i].m10/_mu[i].m00 , _mu[i].m01/_mu[i].m00);
				_ma[i] = (0.5*atan2(2*_mu[i].mu11,_mu[i].mu20-_mu[i].mu02))*180/M_PI;
				//ROS_INFO("Contour no: %ld Area: %ld Center x: %ld  y: %ld  Orientation: %ld ", (long int)i,(long int)area,(long int)_mc[i].x, (long int)_mc[i].y, (long int)_ma[i]);
				//ROS_INFO("Contour no: %ld Area: %ld Center x: %ld  y: %ld  Orientation: %ld ", (long int)i,(long int)area,(long int)minRect[i].center.x, (long int)minRect[i].center.y, (long int)_ma[i]);

				double difference_x, difference_y;
				difference_x = (minRect[i].center.x-320)/pixel_m;
				difference_y = (480 - minRect[i].center.y)/pixel_m;
				// brick.x = minRect[i].center.x;
				// brick.y = minRect[i].center.y;

				ROS_INFO("Contour found! - Area: %ld Center x: %f  y: %f  Orientation: %ld", (long int)area, (float)difference_x, (float)difference_y, (long int)_ma[i]);

				bpMsgs::brick tmpBrick;
				tmpBrick.header.stamp.sec = original_image->header.stamp.sec;
				tmpBrick.header.stamp.nsec = original_image->header.stamp.nsec;
				tmpBrick.x = difference_x;
				tmpBrick.y = difference_y;
				tmpBrick.angle = _ma[i];

				if (area > 1100 && area < 1800) {
				   tmpBrick.type = 2;
				}
				else if (area > 2100 && area < 3000){
				   tmpBrick.type = 1;
				}
				else if(area > 4000 && area < 6000){
				   tmpBrick.type = 3;
				}
				else{
				   tmpBrick.type = 0;
				}

				bool oldBrick = false;
				for (int i = 0; i < bricks.size(); i++)
				{
					// Check if the brick is already known
					if (abs(tmpBrick.x - bricks[i].back().x) < 0.01 && ((tmpBrick.y - bricks[i].back().y) > 0.005 && (tmpBrick.y - bricks[i].back().y) < 0.03) && tmpBrick.type == bricks[i].back().type )
					{
						ROS_INFO("The same brick!");
						oldBrick = true;
						// last time we detect it - publish it!
						if (tmpBrick.y > 0.2)
						{
							tmpBrick.id = brick_id++;
							brick_pub.publish(tmpBrick);
							bricks.erase(bricks.begin()+i);
						}
						// adding the brick to its predecessors vector
						else
						{
							bricks[i].push_back(tmpBrick);
						}

					}
				}
				// Check if it is a new brick
				if (tmpBrick.y < 0.2 && tmpBrick.y > 0.05 && !oldBrick)
				{
					ROS_INFO("New Brick Found!");
					vector< bpMsgs::brick > tmpBrickVec;
					tmpBrickVec.push_back(tmpBrick);
					bricks.push_back(tmpBrickVec);
				}
		   }
		  // cout << "Contour no: " << i << " Area: " << area << " Center x: " << _mc[i].x << " y: " << _mc[i].y << " Orientation: " << _ma[i] << endl;
		 //  cout << "Ellipse Orientation: " << minEllipse[i].angle << endl;

		   //   cout << oldarea << endl;
	  }

		   Mat drawing = Mat::zeros( imgdest.size(), CV_8UC3 );
		     for( int i = 0; i< contours.size(); i++ )
		        {
		          Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		       //   drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		        //  drawContours( drawing, contours, i, color, CV_FILLED );
		        //  ellipse( drawing, minEllipse[i], color, 2, 8 );
		        //  Point2f rect_points[4]; minRect[i].points( rect_points );
		          drawCross(_mc[i],Scalar(0,0,255), 5, drawing);
		          Point2f rect_points[4];
		          minRect[i].points( rect_points );
		          for( int j = 0; j < 4; j++ ) {
		                    line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
		          }
		          char buff[255];
		          //sprintf(buff, "%d", i+1);
		          string text = std::string(buff);
		          cv::putText(drawing,text,_mc[i],0,0.5,Scalar(0,0,255),1,8,false);
		         // for( int j = 0; j < 4; j++ )
		        //            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
		        }
		     //ROS_INFO("size: %d", contours.size());

	cv::imshow("image processed", cv_ptr->image);
	cv::imshow("contours", drawing);
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
	brick_pub = nh.advertise<bpMsgs::brick>("brick_pub", 10);

	cv::namedWindow("image processed", CV_WINDOW_NORMAL);
	cv::namedWindow("contours", CV_WINDOW_NORMAL);
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

//  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
	image_transport::Subscriber sub = it.subscribe("camera/usb_cam/image_raw", 1, imageCallback);
	pub = it.advertise("camera/image_processed", 1);

	ros::spin();

	cv::destroyWindow("image processed");
	cv::destroyWindow("grey");
	cv::destroyWindow("contours");
	cv::destroyWindow("TreshLow");
	cv::destroyWindow("TreshHigh");
	
}
