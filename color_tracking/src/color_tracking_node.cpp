/*
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/front/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
	


	
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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
	
	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0; 
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255; 
	cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	cv::namedWindow(OPENCV_WINDOW);	
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)



	cv::Mat img_hsv;
	cv::Mat img_gauss;
	cv::Mat img_thr;
	cv::GaussianBlur(cv_ptr->image, img_gauss, cv::Size(9,9), 1.5, 1.5);
	cv::cvtColor(img_gauss,img_hsv,CV_BGR2HSV);
	cv::inRange(img_hsv, cv::Scalar(100,0,0), cv::Scalar(180,255,255), img_thr); //Threshold the image
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	//cv::imshow(OPENCV_WINDOW, img_hsv);
	//cv::imshow(OPENCV_WINDOW, img_hsv);
		
	 	cv::imshow("Gaussian", cv_ptr->image);

    cv::waitKey(3);

    
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


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
int iLowH=0;
int iHighH=179;
int iLowS=0;
int iHighS=255;
int iLowV=0;
int iHighV=255;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;

	cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	
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
		cv::cvtColor(cv_ptr->image,img_thr,CV_BGR2HSV); 
		cv::inRange(img_thr, cv::Scalar(iLowH,iLowS,iLowV), cv::Scalar(iHighH,iHighS,iHighV), img_thr); //Threshold the image
		//cv::inRange(cv_ptr->image, cv::Scalar(0,0,0), cv::Scalar(180,256,256), img_thr); 
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
	  ros::init(argc, argv, "image_listener");
	  ros::NodeHandle nh;
	  cv::namedWindow("view");
	  cv::startWindowThread();
	  image_transport::ImageTransport it(nh);
	  image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
	  ros::spin();
	  cv::destroyWindow("view");
}




