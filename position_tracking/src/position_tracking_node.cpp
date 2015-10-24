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
#include <geometry_msgs/PointStamped.h>
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
#include <geometry_msgs/Point.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
#include "ardrone_autonomy/Navdata.h"
using namespace std;

//Values for Trackbar, Hue, Saturation Value
float vex,vey,vez,x,y,z,dt,mx,my,mex,mey,counte;
//Control Set Up
geometry_msgs::Twist twist_msg;
geometry_msgs::Twist twist_msg_neg;
geometry_msgs::Twist twist_msg_hover;
geometry_msgs::Twist twist_msg_up;
std_msgs::Empty emp_msg;

ros::Publisher pub_empty_land;
ros::Publisher pub_twist;
ros::Publisher pub_empty_takeoff;
ros::Publisher pub_empty_reset;

ros::Publisher pub_pos;
geometry_msgs::PointStamped puntoc;

//Times of flying
double start_time;
float takeoff_time;
float fly_time;
float land_time;
float kill_time;
float tpast;
double actual;
float init;
float tiempo;
//Kalman Filter Initialization
int stateSize = 6;
int measSize = 3;
int contrSize = 0;
unsigned int type = CV_32F;
cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z]
cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_z]
cv::Mat estimated(stateSize,1,type);
cv::Mat EstimatedPose;
cv::Mat State;
cv::Mat kontrol;
cv::Mat reference(3,1,type);

cv::Mat PID(cv::Mat estimate,cv::Mat referencia)
{
	float ex,ey,ez,dex,dey,dez,expa,eypa,ezpa;
	float kx=0.4;//0.6 sirven gut;
	float ky=0.4;//;
	float kz=0.5;//;
	float kvx=0.4;//0.6 sirven gut;
	float kvy=0.4;
	float kvz=0.2;
	int inicio=0;
	cv::Mat control(3,1,type);
	ex=referencia.at<float>(0)-estimate.at<float>(0);
	ey=referencia.at<float>(1)-estimate.at<float>(1);
	ez=referencia.at<float>(2)-estimate.at<float>(2);
	control.at<float>(0)=kx*ex+kvx*(-estimate.at<float>(3));
	control.at<float>(1)=ky*ey+kvy*(-estimate.at<float>(4));
	control.at<float>(2)=kz*ez+kvy*(-estimate.at<float>(5));
	return control;
	

}
cv::Mat Kalman(cv::Mat Measure,cv::KalmanFilter K)
{
							
	State=K.predict();	
	EstimatedPose=K.correct(Measure);
	return EstimatedPose;
}
void posCallback(const ardrone_autonomy::Navdata& msg)
{
	ofstream myfile("/home/edrone/VelyPosy8.txt",ios_base::app);
	
	ros::NodeHandle node;
	pub_pos=node.advertise<geometry_msgs::PointStamped>("sensor_position",1);
	ros::Rate loop_rate(50);
	vex=msg.vx/1000;//Old DRONE CHEEEEEEEEEEECK;
	vey=msg.vy/1000;
	z=msg.altd;
	x=x+vex*dt;
	y=y+vey*dt;
	
			//ROS_INFO("Taking off");
			
				
	actual=ros::Time::now().toSec()-start_time;
	dt=actual-tpast;
	tpast=actual;
	kf.transitionMatrix.at<float>(0,3)=dt;
	kf.transitionMatrix.at<float>(1,4)=dt;
	kf.transitionMatrix.at<float>(2,5)=dt;
	
	if (actual<takeoff_time)
		{ //takeoff
			//pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat
			mx=mx+vex;
			my=my+vey;			
		ROS_INFO("Taking off");
		init=ros::Time::now().toSec()-start_time;
		printf("%f \n",init);
		//myfile << vex<<"	"<< vey<<"	"<< x<<"	"<<y<<"	"<<actual<<"\n";	

		}//while takeoff
		if(actual>takeoff_time+fly_time)
		{
			mex=mx/counte;
			mey=my/counte;
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
			ROS_INFO("Landing");
			meas.at<float>(0)=vex;
			meas.at<float>(1)=vey;
			meas.at<float>(2)=z;
			estimated=Kalman(meas,kf);
			
			//myfile << estimated.at<float>(0)<<"	"<< estimated.at<float>(1)<<"	"<<estimated.at<float>(2)<<"	"<< estimated.at<float>(3)<<"	"<<estimated.at<float>(4)<<"	"<<estimated.at<float>(5)<<"	"<<actual<<"\n";
			if (actual> takeoff_time+fly_time+land_time+kill_time){
			ROS_INFO("Closing Node");
			//pub_empty_reset.publish(emp_msg); //kills the drone
			exit(0); }//kill node
								
		}//while land
				if( actual> takeoff_time && actual< takeoff_time+fly_time)
		{
			meas.at<float>(0)=vex;
			meas.at<float>(1)=vey;
			meas.at<float>(2)=z;
			//printf("%f \n",dt);
			estimated=Kalman(meas,kf);
			tiempo=(actual-init);
			printf("%f	%f	%f \n",estimated.at<float>(1),estimated.at<float>(4),tiempo);
			myfile << estimated.at<float>(1)<<"	"<<estimated.at<float>(4)<<"	"<<meas.at<float>(1)<<"	"<<twist_msg.linear.y<<"	"<<actual<<"\n";
			
			kontrol=PID(estimated,reference);	
			twist_msg.linear.x=kontrol.at<float>(0);
			twist_msg.linear.y=0.3*sin(2*tiempo)+0.2*sin(3*tiempo);
			twist_msg.linear.z=0.0;
			puntoc.point.x=estimated.at<float>(0);
			puntoc.point.y=estimated.at<float>(1);
			puntoc.point.z=estimated.at<float>(2);
			puntoc.header.stamp=ros::Time::now();		
			//pub_twist.publish(twist_msg);
			pub_pos.publish(puntoc);
			//printf("%f \n",twist_msg.linear.y);
			//ROS_INFO("Flying +ve");

		}

	loop_rate.sleep();
}
int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "data");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	ros::NodeHandle node;
	pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
	//Deciding Times	
	start_time =ros::Time::now().toSec();
	takeoff_time=5.0;
	fly_time=40.0;
	land_time=3.0;
	kill_time =2.0;
	tpast=0;
	//Command
	twist_msg.linear.x=0.0;
	twist_msg.linear.y=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.x=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.z=0.0;
	ROS_INFO("Starting ARdrone_test loop");
	dt=1/50.0;
	x=0;
	y=0;
	mx=0;
	my=0;
	mex=0;
	mey=0;
	counte=0;
	//Feature Kalman Filter
	//Transition Matrix A
	kf.transitionMatrix = cv::Mat::zeros(stateSize, stateSize, type);
	cv::setIdentity(kf.transitionMatrix);
	kf.transitionMatrix.at<float>(0,3)=dt;
	kf.transitionMatrix.at<float>(1,4)=dt;
	kf.transitionMatrix.at<float>(2,5)=dt;
	//control matrix
	//Measurement Matrix H
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0,3) = 1.0f;
	kf.measurementMatrix.at<float>(1,4) = 1.0f;
	kf.measurementMatrix.at<float>(2,2) = 1.0f;
	//Noise Covariance Matrix Q
	kf.processNoiseCov.at<float>(0,0) = 1e-4;
	kf.processNoiseCov.at<float>(1,1) = 1e-4;
	kf.processNoiseCov.at<float>(2,2) = 1e-4;
	kf.processNoiseCov.at<float>(3,3) = 1e-4;
	kf.processNoiseCov.at<float>(4,4) = 1e-4;
	kf.processNoiseCov.at<float>(5,5) = 1e-4;
	kf.measurementNoiseCov.at<float>(0,0) = 1e-5;
	kf.measurementNoiseCov.at<float>(1,1) = 1e-5;
	kf.measurementNoiseCov.at<float>(2,2) = 1e-4;
	estimated.at<float>(0)=0.0;
	estimated.at<float>(1)=0.0;
	estimated.at<float>(2)=0.0;
	estimated.at<float>(3)=0.0;
	estimated.at<float>(4)=0.0;
	estimated.at<float>(5)=0.0;

	reference.at<float>(0)=0.0;
	reference.at<float>(1)=0.0;
	reference.at<float>(2)=0.0;
	state.at<float>(0)=0.0;
	state.at<float>(1)=0.0;
	state.at<float>(2)=0.0;
	state.at<float>(3)=0.0;
	state.at<float>(4)=0.0;
	state.at<float>(5)=0.0;
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));
	ros::Subscriber sub = nh.subscribe("/ardrone/navdata", 1, posCallback);
	ros::spin();
}
