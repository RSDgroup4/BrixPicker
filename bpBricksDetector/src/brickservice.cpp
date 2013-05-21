/* brickservice.cpp
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
#include <bpMsgs/brick.h>
#include <bpBricksDetector/command.h>



using namespace std;
namespace enc = sensor_msgs::image_encodings;

vector<bpMsgs::brick> brick_obs;
vector< vector<bpMsgs::brick> > vector_update;
vector<bpMsgs::brick> vector_complete;

double x,y, angle;

bool lister(bpBricksDetector::command::Request &req,bpBricksDetector::command::Response& bricks) {

	for (int i = 0; i < vector_complete.size(); i++) {
		bricks.bricks.push_back(vector_complete.at(i));
	}

return true;
}

void callback(bpMsgs::brick Brick) {

	double speed;
	double x,y,angle;

	if (vector_update.empty()) {

		brick_obs.push_back(Brick);
		vector_update.push_back(brick_obs);
		brick_obs.clear();
	}

	else {
		for (int i = 0; i < vector_update.size(); i++) {

			//vector_update.at(i).at(vector_update.at(i).size()-1).header.seq;
			//Erase if older then 20 sec
			if (Brick.header.stamp.toSec() > vector_update.at(i).at(vector_update.at(i).size()-1).header.stamp.toSec()+20) {
				vector_update.at(i).erase(vector_update.at(i).begin(),vector_update.at(i).end());
			}

			for (int k = 0; k < vector_update.at(i).size();k++) {

				if (Brick.header.stamp.toSec() - vector_update.at(i).at(k).header.stamp.toSec() < 0.5 ) {
					if (Brick.y - vector_update.at(i).at(k).y < 0.015) {
						if (Brick.x < 0.001+vector_update.at(i).at(k).x && Brick.x > 0.001-vector_update.at(i).at(k).x) {
							vector_update.at(i).push_back(Brick);
						}
					}
				}
				else {
					brick_obs.push_back(Brick);
					vector_update.push_back(brick_obs);
					brick_obs.clear();
				}
			}
		}

		for (int i = 0; i < vector_update.size(); i++) {
			if (vector_update.at(i).size() > 15) {
					for (int k = 0 ; k < vector_update.at(i).size(); k++) {
						x = x + vector_update.at(i).at(k).x;
						angle = angle + vector_update.at(i).at(k).angle;
					}
					x = x / vector_update.at(i).size();
					angle = angle / vector_update.at(i).size();
					Brick.angle = angle;
					Brick.x = x;
					vector_complete.push_back(Brick);
			}
		}
	}


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle n;

  ros::Subscriber sub = nh.subscribe("brick_pub",1 , callback);
  ros::ServiceServer service = n.advertiseService("Bricks_in_system", lister);

  ros::spin();


  return 0;

}

