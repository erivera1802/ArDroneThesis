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

    cv::namedWindow(OPENCV_WINDOW);
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

	//cv::Mat img_hsv;
	//cv::Mat img_gauss;
	//cv::Mat img_thr;
	//cv::GaussianBlur(cv_ptr->image, img_gauss, cv::Size(9,9), 1.5, 1.5);
	//cv::cvtColor(img_gauss,img_hsv,CV_BGR2HSV);
	//cv::inRange(img_hsv, cv::Scalar(60, 30, 20), cv::Scalar(140, 170, 128), img_hsv);
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	//cv::imshow(OPENCV_WINDOW, img_hsv);
	//cv::imshow(OPENCV_WINDOW, img_hsv);
		int minHessian = 400;
		cv::SurfFeatureDetector detector( minHessian );
		std::vector<KeyPoint> keypoints_2;
		cv::detector.detect( cv_ptr->image, keypoints_2 );
		cv::Mat img_keypoints_2;
		cv::drawKeypoints( imgOriginal, keypoints_2, img_keypoints_2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );
		
	 	imshow("Keypoints 2", img_keypoints_2 );

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
