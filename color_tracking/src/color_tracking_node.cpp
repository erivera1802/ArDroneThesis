
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
//Values for Trackbar, Hue, Saturation Value
int iLowH=0;
int iHighH=179;
int iLowS=0;
int iHighS=255;
int iLowV=0;
int iHighV=255;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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
		cv::imshow("view",img_thr);
		cv::imshow("Control",cv_ptr->image);
		cv::waitKey(30);
	  }
	  catch (cv_bridge::Exception& e)
	  {
	    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	  }
	
}

int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cv::namedWindow("view");
	cv::startWindowThread();
	//Subscribing to ardrone camera node
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
}




