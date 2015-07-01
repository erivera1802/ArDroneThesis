
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
#include <fstream>
using namespace cv;
using namespace std;
//Values for Trackbar, Hue, Saturation Value
int iLowH=163;
int iHighH=179;
int iLowS=151;
int iHighS=201;
int iLowV=135;
int iHighV=181;

int posY,posZ;
float ey,ez,ex,dey,eypa,dex,expa,dez,ezpa,iex,iey,iez,uy,ux,uz;

geometry_msgs::Twist twist_msg;
std_msgs::Empty emp_msg;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		ofstream myfile("/home/edrone/example.txt",ios_base::app);
		
		ros::Rate loop_rate(50);
		int count=0;
		double dM01;
		double dM10;
		double dArea;
		int big;
		Point pt;
		ros::NodeHandle nh;
		ros::NodeHandle neu;
		ros::Publisher pub;
		ros::Publisher pub_empty_land;
		ros::Publisher pub_twist;
		ros::Publisher chatter_pub=neu.advertise<std_msgs::String>("chatter", 1000);
		pub_empty_land = neu.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
		pub = neu.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
		//Communicating between ROS an OpenCV
		cv_bridge::CvImagePtr cv_ptr;

		cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); 
	
		cv::Mat img_thr;

		  try
		  {
			cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cvCreateTrackbar("LowH", "Control", &iLowH, 179); 
			cvCreateTrackbar("HighH", "Control", &iHighH, 179); 
			cvCreateTrackbar("LowS", "Control", &iLowS, 255); 
			cvCreateTrackbar("HighS", "Control", &iHighS, 255); 
			cvCreateTrackbar("LowV", "Control", &iLowV, 255); 
			cvCreateTrackbar("HighV", "Control", &iHighV, 255);
			//Thresholding
			cv::cvtColor(cv_ptr->image,img_thr,CV_BGR2HSV); 
			cv::inRange(img_thr, cv::Scalar(iLowH,iLowS,iLowV), cv::Scalar(iHighH,iHighS,iHighV), img_thr); 
			//Voilá
			Moments oMoments = moments(img_thr);

			 dM01 = oMoments.m01;
			 dM10 = oMoments.m10;
			 dArea = oMoments.m00;
			
			if (dArea > 10000)
			{
				//calculate the position of the ball
				 posY = dM10 / dArea;
				 posZ = dM01 / dArea;
				pt.x=posY;
				pt.y=posZ;
				big=dArea/40000;  
				circle(cv_ptr->image, pt, big, Scalar(0,0,255), 1, 8, 0);
				ey=(319-posY)/320.0;
				ez=(179-posZ)/180.0;
				ex=(40.0-big)/40.0;
				dey=-(ey-eypa)/1.0;
				dez=-(ez-ezpa)/1.0;
				dex=-(ex-expa)/1.0;
				iey=iey+ey;
				iez=iez+ez;
				iex=iex+ex;
				uy=ey+dey;
				ux=ex+dex;
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
				ROS_INFO("Ball Position:[%f,%f,%f,%f]",ex,dex,iex,ux);
				myfile << uy << "\n";
				twist_msg.angular.z=uy;
				twist_msg.linear.x=ux;							
				pub.publish(twist_msg);
				ros::spinOnce();

		
				  
			}
			else
			{
				posY=319;
				posZ=179;
				ROS_INFO("Landing!!!!");
				pt.x=posY;
				pt.y=posZ;
				ey=(319-posY)/319.0;
				ez=(179-posZ)/179.0;  
				circle(cv_ptr->image, pt, 50, Scalar(0,0,255), 1, 8, 0);
				pub_empty_land.publish(emp_msg);
			}
		
		
			cv::imshow("view",img_thr);
			cv::imshow("Control",cv_ptr->image);
			cv::waitKey(30);
			eypa=ey;
			expa=ex;
			ezpa=ez;
			ros::spin();
			
		  }
		  catch (cv_bridge::Exception& e)
		  {
		    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		  }
	
	
	}

int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	
	cv::namedWindow("view");
	cv::startWindowThread();

	twist_msg.linear.y=0.0;
	twist_msg.angular.z=0.0;
	twist_msg.angular.y=0.0;
	twist_msg.angular.x=0.0;
	eypa=0.0;
	expa=0.0;
	ezpa=0.0;
	iex=0.0;
	iey=0.0;
	iez=0.0;
			



	//Subscribing to ardrone camera node
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
}




