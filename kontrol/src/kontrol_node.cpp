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
   int stateSize = 3;
   int measSize = 1;
   int contrSize = 1;
   unsigned int type = CV_32F;
	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
	cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z]
  	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_z]
cv::Mat estimated(3,1,type);
cv::Mat reference(3,1,type);
cv::Mat estado(6,1,type);
cv::Mat Uy(contrSize,1,type);
cv::Mat EstimatedPose;
cv::Mat State;
cv::Mat measy(measSize, 1, type);
cv::Mat kontrol(3,1,type); //Control de los 3 ejes
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

float Uu;

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

cv::Mat Kalman(cv::Mat Measure,cv::KalmanFilter K,cv::Mat contr)
{
							
	State=K.predict(contr);	
	EstimatedPose=K.correct(Measure);
	return EstimatedPose;
}
cv::Mat PID(cv::Mat estimate,cv::Mat referencia)
{
	float ex,ey,ez,dex,dey,dez,expa,eypa,ezpa;
	float kx=0.4;//0.6 sirven gut;
	float ky=0.6;//0.9;
	float kz=0.3;//;
	float kvx=0.4;//0.6 sirven gut;
	float kvy=0.6;//0.7
	float kvz=0.4;
	float kiy=0.0;
	float kix=0.0;
	int inicio=0;
	cv::Mat control(3,1,type);
	ex=referencia.at<float>(0)-estimate.at<float>(0);
	ey=2-estimate.at<float>(1);
	ez=referencia.at<float>(2)-estimate.at<float>(2);
	iey=iey+ey;
	iex=iex+ex;
	control.at<float>(0)=kx*ex+kvx*(-estimate.at<float>(3))+kix*iex;
	control.at<float>(1)=ky*ey+kvy*(-estimate.at<float>(4))+kiy*iey;
	control.at<float>(2)=kz*ez+kvy*(-estimate.at<float>(5));
	eypa=ey;
	return control;
}
float Kplace(float Px,float Velx,float delay,float rf)
{
	/*float K1=0.8719;
	float K2=0.3190;
	float K3=1.7452;*/
	float K1=4.16;
	float K2=1.469;
	float K3=7.836;
	float N=4.15;
	float Uc=rf*N-K1*Px-K2*Velx-K3*delay;
	return Uc;
	
}

void callback(const geometry_msgs::PointStamped::ConstPtr& p1, const geometry_msgs::PointStamped::ConstPtr& p2,const geometry_msgs::PointStamped::ConstPtr& v1,const geometry_msgs::PointStamped::ConstPtr& v2)
{
	ofstream myfile("/home/edrone/KpCam.txt",ios_base::app);
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
	
	//Kalman for the delay
	
	measy.at<float>(0)=vsY;
	State=kf.predict(Uy);
	estimated=kf.correct(measy);	
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
			Uy=Kplace(estado.at<float>(1),estimated.at<float>(1),estimated.at<float>(2),reference.at<float>(1));	
			twist_msg.linear.y=Uy.at<float>(0);
			twist_msg.linear.x=kontrol.at<float>(0);
			/*twist_msg.linear.y=estimated.at<float>(0);
			twist_msg.linear.x=estimated.at<float>(1);*/
			if(abs(Uy.at<float>(0))>1)
			{Uy.at<float>(0)=Uy.at<float>(0)/abs(Uy.at<float>(0));}
			if(abs(PosX)>3 || abs(PosY)>3)
			{pub_empty_land.publish(emp_msg);
			ROS_INFO("Landing");}
			//ROS_INFO("Doin' something");
			printf("%f	%f	%f\n",estado.at<float>(1),reference.at<float>(1),Uy.at<float>(0));
			/*if(tiempo<5+takeoff_time && tiempo>3+takeoff_time)
			{
				twist_msg.linear.x=0.3;
				printf("Step! \n");
			}
			else
			{
				twist_msg.linear.x=0;
				printf("NoStep! \n");
				pub_twist.publish(twist_msg_hover);
			}*/
			//twist_msg.linear.y=kontrol.at<float>(1);
			twist_msg.linear.z=0.0;
			pub_twist.publish(twist_msg);
			//printf("%f	%f \n",reference.at<float>(0),twist_msg.linear.x);
			ey=reference.at<float>(1)-estado.at<float>(1);
			ex=reference.at<float>(0)-estado.at<float>(0);
			//myfile <<estado.at<float>(1)<<"	"<<estado.at<float>(4)<<"	"<<estimated.at<float>(0)<<"	"<<estimated.at<float>(1)<<"	"<<estimated.at<float>(2)<<"	"<<tiempo<<"\n";
			myfile <<estado.at<float>(1)<<"	"<<estado.at<float>(4)<<"	"<<reference.at<float>(1)<<"	"<<estimated.at<float>(0)<<"	"<<estimated.at<float>(1)<<"	"<<estimated.at<float>(2)<<"	"<<Uy.at<float>(0)<<"	"<<tiempo<<"\n";
			
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
	/*kf.transitionMatrix.at<float>(1,1)=0.9489;
	kf.transitionMatrix.at<float>(2,1)=0.0;
	kf.transitionMatrix.at<float>(1,2)=1.1965;
	kf.transitionMatrix.at<float>(2,2)=0.7988;
	kf.transitionMatrix.at<float>(0,0)=1;
	kf.transitionMatrix.at<float>(0,1)=0.03285;
	kf.transitionMatrix.at<float>(0,2)=0.02107;
	*/
	kf.transitionMatrix.at<float>(0,0)=1;
	kf.transitionMatrix.at<float>(0,1)=0.0725;
	kf.transitionMatrix.at<float>(0,2)=0.1143;
	kf.transitionMatrix.at<float>(1,0)=0;
	kf.transitionMatrix.at<float>(1,1)=0.8873;
	kf.transitionMatrix.at<float>(1,2)=2.6748;
	kf.transitionMatrix.at<float>(2,0)=0;
	kf.transitionMatrix.at<float>(2,1)=0;
	kf.transitionMatrix.at<float>(2,2)=0.5988;
	
	/*kf.transitionMatrix.at<float>(0,0)=1;
	kf.transitionMatrix.at<float>(1,0)=dt;
	kf.transitionMatrix.at<float>(0,1)=0;
	kf.transitionMatrix.at<float>(1,1)=1;*/

	//Control Matrix B
	kf.controlMatrix = cv::Mat::zeros(stateSize, contrSize, type);
	/*kf.controlMatrix.at<float>(0)=-0.005;	
	kf.controlMatrix.at<float>(1)=-0.3166;
	kf.controlMatrix.at<float>(2)=0.1206;*/
	kf.controlMatrix.at<float>(0)=-0.0281;	
	kf.controlMatrix.at<float>(1)=-0.5739;
	kf.controlMatrix.at<float>(2)=0.2407;
	/*kf.controlMatrix.at<float>(0,0)=0;
	kf.controlMatrix.at<float>(1,0)=0;*/
	//Measurement Matrix H
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0,0) = 0.0f;
	kf.measurementMatrix.at<float>(0,1) = 1.0f;
	kf.measurementMatrix.at<float>(0,2) = 0.0f;
	//Noise Covariance Matrix Q
	kf.processNoiseCov.at<float>(0,0) = 1e-6;
	kf.processNoiseCov.at<float>(1,1) = 1e-6;
	kf.processNoiseCov.at<float>(2,2) = 1e-6;

	kf.measurementNoiseCov.at<float>(0,0) = 5e-3;

	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));
	Uy.at<float>(0)=0.0;
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
	estimated.at<float>(0,0)=0.0;
	estimated.at<float>(1,0)=0.0;
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

