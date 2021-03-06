#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
/*
using namespace cv;
using namespace std;
static const std::string OPENCV_WINDOW = "Image window";
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream

	Mat img_hsv;
	//cv::Mat img_gauss;
	//cv::Mat img_thr;
	//cv::GaussianBlur(cv_ptr->image, img_gauss, cv::Size(9,9), 1.5, 1.5);
	cvtColor(cv_ptr->image,img_hsv,CV_BGR2GRAY);
	//cv::inRange(img_hsv, cv::Scalar(60, 30, 20), cv::Scalar(140, 170, 128), img_hsv);
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	//cv::imshow(OPENCV_WINDOW, img_hsv);
	imshow(OPENCV_WINDOW, img_hsv);
	int minHessian = 400;
	//SurfFeatureDetector detector( minHessian );

	Ptr<FeatureDetector> detector = new SurfFeatureDetector;
	std::vector<KeyPoint> keypoints_1;
	detector->detect(img_hsv, keypoints_1 );
	
	cv::Mat img_keypoints_1;
	drawKeypoints( img_hsv, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	imshow("Keypoints 1", img_keypoints_1 );

    	waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

*/
using namespace cv;
using namespace std;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//Communicating between ROS an OpenCV
	cv_bridge::CvImagePtr cv_ptr;

	cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); 
	
	cv::Mat img_thr;

	  try
	  {
		cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		//Thresholding
		cv::cvtColor(cv_ptr->image,img_thr,CV_BGR2GRAY); 
		//Voilá
		int minHessian = 400;
		//SurfFeatureDetector detector( minHessian );

		Ptr<FeatureDetector> detector = new SurfFeatureDetector;
		std::vector<KeyPoint> keypoints_1;
		detector->detect(img_thr, keypoints_1 );
	
		cv::Mat img_keypoints_1;
		drawKeypoints( img_thr, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		imshow("view", img_keypoints_1 );
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
