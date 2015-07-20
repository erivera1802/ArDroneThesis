#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/video.hpp>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <sys/time.h>
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <ros/package.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <geometry_msgs/Point.h>
#include <fstream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/video.hpp>
using namespace std;

//Values for Trackbar, Hue, Saturation Value
float posY,posZ,posX;
geometry_msgs::Point point_msg;
geometry_msgs::Point pospt;
std_msgs::Empty emp_msg;

	//Kalman Filter Initialization
   int stateSize = 6;
   int measSize = 3;
   int contrSize = 0;
   unsigned int type = CV_32F;
	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
	cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z]
  	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_z]

	int found=0;
void posCallback(const geometry_msgs::Point& msg)
{
	ofstream myfile("/home/edrone/pos1.txt",ios_base::app);
	ros::NodeHandle neu;
	ros::Rate loop_rate(50);
	ros::Publisher pub_point=neu.advertise<geometry_msgs::Point>("position_estimation",1000);
	posX=msg.x;
	posY=msg.y;
	posZ=msg.z;
	

	meas.at<float>(0) = posX;
	meas.at<float>(1) = posY;
	meas.at<float>(2) = posZ;
	if(found==0)
	{
	 	kf.errorCovPre.at<float>(0,0) = 1; // px
		kf.errorCovPre.at<float>(1,1) = 1; // px
		kf.errorCovPre.at<float>(2,2) = 1;
		kf.errorCovPre.at<float>(3,3) = 1;
		kf.errorCovPre.at<float>(4,4) = 1; // px
		kf.errorCovPre.at<float>(5,5) = 1; // px


		
		state.at<float>(0) = meas.at<float>(0);
		state.at<float>(1) = meas.at<float>(1);
		state.at<float>(2) = meas.at<float>(2);
		state.at<float>(2) = 0.0;
		state.at<float>(3) = 0.0;
		
		// <<<< Initialization
		found = 1;
	}
	
	state=kf.predict();
	
	
	cv::Mat estimated=kf.correct(meas);
	printf("[%f,%f,%f] \n",estimated.at<float>(0),estimated.at<float>(1),estimated.at<float>(2));
	pospt.x=estimated.at<float>(0);
	pospt.y=estimated.at<float>(1);
	pospt.z=estimated.at<float>(2);

	myfile << estimated.at<float>(0) <<"	"<< posX<<"	"<< estimated.at<float>(1) <<"	"<< posY<<"	"<< estimated.at<float>(2) <<"	"<< posZ<< "\n";
	pub_point.publish(pospt);
	cv::waitKey(30);
	ros::spin();
}
int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "kalman_obj");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);

	float dt=0.0647;//Dont know it yet
	//Transition Matrix A
	cv::setIdentity(kf.transitionMatrix);
	kf.transitionMatrix.at<float>(0,3)=dt;
	kf.transitionMatrix.at<float>(1,4)=dt;
	kf.transitionMatrix.at<float>(2,5)=dt;
	//Measurement Matrix H
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0,0) = 1.0f;
	kf.measurementMatrix.at<float>(1,1) = 1.0f;
	kf.measurementMatrix.at<float>(2,2) = 1.0f;
	//Noise Covariance Matrix Q
	kf.processNoiseCov.at<float>(0,0) = 1e-2;
	kf.processNoiseCov.at<float>(1,1) = 1e-2;
	kf.processNoiseCov.at<float>(2,2) = 1e-2;
	kf.processNoiseCov.at<float>(3,3) = 2.0f;
	kf.processNoiseCov.at<float>(4,4) = 2.0f;
	kf.processNoiseCov.at<float>(5,5) = 2.0f;
	
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
	//cv::setIdentity(kf.errorCovPost,cv::Scalar(.1));
	ros::Subscriber sub = nh.subscribe("color_position", 100, posCallback);
	ros::spin();
}
