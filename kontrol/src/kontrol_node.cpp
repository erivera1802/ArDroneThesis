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
float ey,ez,ex,dey,eypa,dex,dey2,expa,dez,ezpa,iex,iey,iez,uy,ux,uz;
geometry_msgs::Twist twist_msg;
geometry_msgs::Point point_msg;
std_msgs::Empty emp_msg;
ros::Publisher pub_twist;
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



void callback(const geometry_msgs::PointStamped::ConstPtr& p1, const geometry_msgs::PointStamped::ConstPtr& p2)
{
  ros::NodeHandle neu;
	ros::Rate loop_rate(50);
	geometry_msgs::PointStamped posim=*p1;
	CamX=posim.point.x;
	CamY=posim.point.y;
	CamZ=posim.point.z;

	geometry_msgs::PointStamped possen=*p2;
	PosX=possen.point.x;
	PosY=possen.point.y;
	PosZ=possen.point.z;
	/*posY=p1.y;
	posZ=p1.z;*/
	printf("%f	%f \n",CamZ,PosZ);
	//ofstream myfile("/home/edrone/PDK1.txt",ios_base::app);
	//pub_twist= neu.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	//printf("hola \n");
}
int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;
	//ros::Rate loop_rate(50);
	

	refX=0;
	refY=0;
	refZ=1.0;
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
	
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));
	  message_filters::Subscriber<geometry_msgs::PointStamped> pos1_sub(nh, "color_position", 1);

	message_filters::Subscriber<geometry_msgs::PointStamped> pos2_sub(nh, "sensor_position", 1);
	typedef sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), pos1_sub, pos2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));



	ros::spin();
	return 0;

}

