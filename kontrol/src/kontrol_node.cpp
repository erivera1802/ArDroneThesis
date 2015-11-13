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


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace message_filters;
//Values for Trackbar, Hue, Saturation Value
float CamX,CamY,CamZ;
float PosX,PosY,PosZ;
float vsX,vsY,vsZ;
float viX,viY,viZ;
float ey,ez,ex,dey,eypa,dex,dey2,expa,dez,ezpa,iex,iey,iez,uy,ux,uz;
float refX,refY,refZ;

//Kalman Filter
   int stateSize = 6;
   int measSize = 3;
   int contrSize = 0;
   unsigned int type = CV_32F;
	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
	cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z]
  	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_z]
cv::Mat estimated;
cv::Mat reference(3,1,type);
cv::Mat estado(6,1,type);
cv::Mat kontrol(3,1,type);


float estimatedx;
float estimatedy;
float estimatedz;
float estimatedvx;
float estimatedvy;
float estimatedvz;
float posxe;
float posye;
float posze;
float vxe;
float vze;
float vye;
float resta;

//Times
double start_time;
float takeoff_time;
float fly_time;
float land_time;
float kill_time;
float tpast;
double actual;
float init;
float tiempo;

//Ros 
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

cv::Mat PID(cv::Mat estimate,cv::Mat referencia)
{
	float ex,ey,ez,dex,dey,dez,expa,eypa,ezpa;
	float kx=0.6;//0.6 sirven gut;
	float ky=0.7;//0.9;
	float kz=0.3;//;
	float kvx=0.5;//0.6 sirven gut;
	float kvy=0.6;//0.7
	float kvz=0.4;
	float kiy=0.0;
	float kix=0.0;
	int inicio=0;
	cv::Mat control(3,1,type);
	ex=referencia.at<float>(0)-estimate.at<float>(0);
	ey=referencia.at<float>(1)-estimate.at<float>(1);
	ez=referencia.at<float>(2)-estimate.at<float>(2);
	iey=iey+ey;
	iex=iex+ex;
	control.at<float>(0)=kx*ex+kvx*(-estimate.at<float>(3))+kix*iex;
	control.at<float>(1)=ky*ey+kvy*(-estimate.at<float>(4))+kiy*iey;
	control.at<float>(2)=kz*ez+kvy*(-estimate.at<float>(5));
	eypa=ey;
	return control;
	

}

void callback(const geometry_msgs::PointStamped::ConstPtr& p1, const geometry_msgs::PointStamped::ConstPtr& p2,const geometry_msgs::PointStamped::ConstPtr& v1,const geometry_msgs::PointStamped::ConstPtr& v2)
{
	ofstream myfile("/home/edrone/PID7.txt",ios_base::app);
  ros::NodeHandle neu;
	ros::Rate loop_rate(50);
	geometry_msgs::PointStamped posim=*p1;
	CamY=posim.point.x;
	CamZ=posim.point.y;
	CamX=-posim.point.z;

	geometry_msgs::PointStamped possen=*p2;
	PosX=possen.point.x;
	PosY=possen.point.y;
	PosZ=possen.point.z;
	
	geometry_msgs::PointStamped velim=*v1;
	viY=velim.point.x;
	viZ=velim.point.y;
	viX=velim.point.z;

	geometry_msgs::PointStamped velsen=*v2;
	vsX=velsen.point.x;
	vsY=velsen.point.y;
	vsZ=velsen.point.z;

	estado.at<float>(0)=PosX;
	estado.at<float>(1)=PosY;
	estado.at<float>(2)=PosZ;
	estado.at<float>(3)=vsX;
	estado.at<float>(4)=vsY;
	estado.at<float>(5)=vsZ;
		

	//printf("%f	%f	%f	%f \n",CamY,PosY,viY,vsY);
	resta=PosX-CamX;
	actual=ros::Time::now().toSec()-start_time;
		if (actual<takeoff_time)
		{ //takeoff
			pub_empty_takeoff.publish(emp_msg); //launches the drone
			pub_twist.publish(twist_msg_hover); //drone is flat			
			ROS_INFO("Taking off");
			init=actual;	

		}//while takeoff
		if(actual>takeoff_time+fly_time)
		{
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
			ROS_INFO("Landing");
			if (actual> takeoff_time+fly_time+land_time+kill_time)
			{
			ROS_INFO("Closing Node");
			//pub_empty_reset.publish(emp_msg); //kills the drone
			exit(0); 
			}//kill node
								
		}//while land
		if( actual> takeoff_time && actual< takeoff_time+fly_time)
		{
			tiempo=actual-init;
			if(CamY!=-10 && CamX!=-10)
			{
				//reference.at<float>(0)=estado.at<float>(0)-CamX-1;
				reference.at<float>(1)=estado.at<float>(1)-CamY;
			}
			kontrol=PID(estado,reference);	
			twist_msg.linear.x=kontrol.at<float>(0);
			if(tiempo<2+takeoff_time)
			twist_msg.linear.y=0.5;
			else
			{twist_msg.linear.y=0;}
			//twist_msg.linear.y=kontrol.at<float>(1);
			twist_msg.linear.z=0.0;
			pub_twist.publish(twist_msg);
			printf("%f	%f \n",reference.at<float>(0),twist_msg.linear.x);
			ey=reference.at<float>(1)-estado.at<float>(1);
			//myfile << twist_msg.linear.x<<"	"<<estado.at<float>(0)<<"	"<<ex<<"	"<< twist_msg.linear.y<<"	"<<estado.at<float>(1)<<"	"<<ey<<"	"<<tiempo<<"\n";
			myfile << twist_msg.linear.x<<"	"<<estado.at<float>(0)<<"	"<<ex<<"	"<< twist_msg.linear.y<<"	"<<estado.at<float>(1)<<"	"<<ey<<"	"<<tiempo<<"\n";
			
		}
	//myfile << resta<<"\n";
	//pub_twist= neu.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

}
int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;
	//ros::Rate loop_rate(50);
	pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty_takeoff = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = nh.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	pub_empty_reset = nh.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */


	twist_msg.linear.y=0.0;
	twist_msg.linear.x=0.0;
	twist_msg.linear.z=0.0;
	twist_msg.angular.z=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.x=0.0;
	eypa=0.0;
	expa=0.0;
	ezpa=0.0;
	iex=0.0;
	iey=0.0;
	iez=0.0;

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
	kf.processNoiseCov.at<float>(0,0) = 1e-3;
	kf.processNoiseCov.at<float>(1,1) = 1e-3;
	kf.processNoiseCov.at<float>(2,2) = 1e-3;
	kf.processNoiseCov.at<float>(3,3) = 1e-3;
	kf.processNoiseCov.at<float>(4,4) = 1e-3;
	kf.processNoiseCov.at<float>(5,5) = 1e-3;
	kf.measurementNoiseCov.at<float>(0,0) = 5e-6;
	kf.measurementNoiseCov.at<float>(1,1) = 5e-6;
	kf.measurementNoiseCov.at<float>(2,2) = 1e-1;


	start_time =ros::Time::now().toSec();
	takeoff_time=5.0;
	fly_time=20;
	land_time=3.0;
	kill_time =2.0;

	reference.at<float>(0)=0.0;
	reference.at<float>(1)=0.0;
	reference.at<float>(2)=0.0;

	estado.at<float>(1)=0.0;
	estado.at<float>(2)=0.0;
	estado.at<float>(3)=0.0;
	estado.at<float>(4)=0.0;
	estado.at<float>(5)=0.0;

	eypa=0.0;
	iex=0;
	iey=0;
	iez=0;

	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));
	message_filters::Subscriber<geometry_msgs::PointStamped> pos1_sub(nh, "color_position", 1);
	message_filters::Subscriber<geometry_msgs::PointStamped> pos2_sub(nh, "sensor_position", 1);
	message_filters::Subscriber<geometry_msgs::PointStamped> vel1_sub(nh, "color_velocity", 1);
	message_filters::Subscriber<geometry_msgs::PointStamped> vel2_sub(nh, "sensor_velocity", 1);
	typedef sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped,geometry_msgs::PointStamped,geometry_msgs::PointStamped> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), pos1_sub, pos2_sub,vel1_sub,vel2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));



	ros::spin();
	return 0;

}

