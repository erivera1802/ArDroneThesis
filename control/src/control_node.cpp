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
using namespace std;
//Values for Trackbar, Hue, Saturation Value

int posY,posZ;
float ey,ez,ex,dey,eypa,dex,expa,dez,ezpa,iex,iey,iez,uy,ux,uz,posX;

geometry_msgs::Twist twist_msg;
geometry_msgs::Point point_msg;
std_msgs::Empty emp_msg;
ros::Publisher pub_twist;

	void posCallback(const geometry_msgs::Point& msg)
	{
		ofstream myfile("/home/edrone/example.txt",ios_base::app);
		ros::NodeHandle neu;
		ros::Rate loop_rate(50);
		posY=msg.x;
		posZ=msg.y;
		posX=msg.z;
		//printf("[%d,%d,%f] \n",posY,posZ,posX);
		pub_twist= neu.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

		ey=(319-posY)/320.0;
		ez=(179-posZ)/180.0;
		ex=(40.0-posX)/40.0;
		dey=-(ey-eypa)/1.0;
		dez=-(ez-ezpa)/1.0;
		dex=-(ex-expa)/1.0;
		iey=iey+ey;
		iez=iez+ez;
		iex=iex+ex;
		uy=ey;
		ux=ex/4+dex;
		uz=ez+dez;
		if(abs(uy)>1.0)
		{
			uy=uy/abs(uy);				
		}
		if(abs(ux)>1.0)
		{
			ux=ux/abs(ux);				
		}
		if(abs(uz)>1.0)
		{
			uz=uz/abs(uz);				
		}
		twist_msg.angular.z=uy;
		twist_msg.linear.x=ux;
		ROS_INFO("Ball Position:[%f,%f]",ux,uy);							
		pub_twist.publish(twist_msg);
		myfile << uy << "\n";
		ros::spinOnce();
	}

int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	
	
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
			
	ros::Subscriber sub = nh.subscribe("color_position", 100, posCallback);
	ros::spin();
	

}




