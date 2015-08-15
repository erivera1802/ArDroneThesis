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
using namespace cv;
using namespace std;
float ey,ez,ex;
float refz;
float xpast=0,ypast=0,zpast=0;
//Values for Trackbar, Hue, Saturation Value
int iLowH=160;
int iHighH=179;
int iLowS=131;
int iHighS=211;
int iLowV=146;
int iHighV=248;
//float PosX,PosY,PosZ;
geometry_msgs::Point point_msg;
std_msgs::Empty emp_msg;
geometry_msgs::Point punto;

//blob detection
 SimpleBlobDetector::Params params;
    std::vector<Point3f> col_3D(4);   
std::vector<Point2f> color_keypoints(4);         
//Color Trackbar

//Camera Matrix:
cv::Mat Matrix(3,3,CV_64F);
//Translation and rotation matrices
cv::Mat tvec;
cv::Mat rvec;
cv::Mat tvec_col;
cv::Mat rvec_col;
cv::Mat estimated;
//Distortion Vector
vector<double> distortion(5);
	//Kalman Filter Initialization
   int stateSize = 6;
   int measSize = 3;
   int contrSize = 0;
   unsigned int type = CV_32F;
	cv::KalmanFilter kf(stateSize, measSize, contrSize, type);
	cv::Mat state(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z]
  	cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_z]

	int found=0;


	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		ros::Rate loop_rate(50);
		ofstream myfile("/home/edrone/pos.txt",ios_base::app);
		int count=0;
		
		Point pt;
		ros::NodeHandle neu;
		ros::Publisher pub_empty_land;
		ros::Publisher pub_point=neu.advertise<geometry_msgs::Point>("color_position",1);
		pub_empty_land = neu.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	
		//Communicating between ROS an OpenCV
		cv_bridge::CvImagePtr cv_ptr; 
		cv::Mat img_thr;
		    std::vector<Point3f> col_3D(4);   
std::vector<Point2f> color_keypoints(4);  
		  try
		  {


				std::string image = ros::package::getPath("stream") + "/src/box3.jpg";
				cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				cv::Mat img2;
				cv::Mat color;
				cv::Mat gauss;
				cv::Mat blobs;
				color=cv_ptr->image;
				cv::GaussianBlur( cv_ptr->image, gauss, Size( 3, 3 ), 0, 0 );
				cv::cvtColor(gauss,gauss,CV_BGR2HSV); 
				cvCreateTrackbar("LowH", "blob", &iLowH, 179); 
				cvCreateTrackbar("HighH", "blob", &iHighH, 179); 
				cvCreateTrackbar("LowS", "blob", &iLowS, 255); 
				cvCreateTrackbar("HighS", "blob", &iHighS, 255); 
				cvCreateTrackbar("LowV", "blob", &iLowV, 255); 
				cvCreateTrackbar("HighV", "blob", &iHighV, 255);
				cv::inRange(gauss, cv::Scalar(iLowH,iLowS,iLowV), cv::Scalar(iHighH,iHighS,iHighV), gauss); 

				//Color reference
				col_3D[0] = Point3f(-0.23,-0.245,0); 
				col_3D[1] = Point3f(0.23,-0.245,0);
				col_3D[2] = Point3f(-0.23,0.245,0); 
				col_3D[3] = Point3f(0.23,0.245,0);
				//Thresholding
				cv::cvtColor(cv_ptr->image,img2,CV_BGR2GRAY);
				Mat img1 = imread(image, CV_LOAD_IMAGE_GRAYSCALE );
				
				if( img1.empty() ) 
				{
			 		printf("Error occured, image not read correctly \n");
				}
				//Blob detection
				vector<KeyPoint> keyPointsBlob;
				SimpleBlobDetector blobDetector( params );
				
				blobDetector.detect( gauss, keyPointsBlob );
				if(keyPointsBlob.size()==4)
				{
					drawKeypoints(gauss,keyPointsBlob,blobs,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
					imshow("blob", blobs );
					for(int a=0;a<keyPointsBlob.size();a=a+1)
					{
						color_keypoints[a]=keyPointsBlob[a].pt;
					}
					solvePnP(Mat(col_3D),Mat(color_keypoints),Matrix,distortion,rvec_col,tvec_col,false,CV_ITERATIVE);
					//printf("%f,%f,%f, COLOR \n",tvec_col.at<double>(0,0),tvec_col.at<double>(1,0),tvec_col.at<double>(2,0));
					//printf("%f,%f \n",color_keypoints[2].x,color_keypoints[2].y);

				}
				else
				{printf("kein keypoints \n");}
				//imshow("blob",gauss);
				
				//Detector, and extractor initializing
				//Detector, and extractor initializing
				//IMPORTAAAAAAAAAANT, REMEMBER TO CHECK SURF AND SIFT, 'CAUSE SCALE INVARIANCE PROBLEM :-(
				//UUUUSMJSJIASLJADSJLDAJILDAJILDAJILDALJ?????¡!!!!!!!!!!!!!!!!!
				//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				OrbFeatureDetector detector;
			    	vector<KeyPoint> keypoints1, keypoints2,objk,scenek;
				detector.detect(img1, keypoints1);
				    // computing descriptors
				OrbDescriptorExtractor extractor;
				Mat descriptors1, descriptors2;
				extractor.compute(img1, keypoints1, descriptors1);
				//Matching
				BFMatcher matcher(NORM_HAMMING,false);
				vector< vector< DMatch > > matches;
				vector<DMatch> good_matches;
				

				//Vector Initializing		
				Point center;
				DMatch Forward,Backward;
				std::vector<Point2f> obj;
				std::vector<Point2f> scene;
				float posY=0.0;
				float posZ=0.0;
				
		
				detector.detect(img2, keypoints2);
			    	// computing descriptors
			    	extractor.compute(img2, keypoints2, descriptors2);
				

				matcher.knnMatch(descriptors2, descriptors1, matches, 2);
				//Good Matches Selection
				for (int i = 0; i < matches.size(); ++i)
				{
				    const float ratio = 0.7; // As in Lowe's paper; can be tuned
				    if (matches[i][0].distance < ratio * matches[i][1].distance)
				    {
					good_matches.push_back(matches[i][0]);
				    }
				}
				for( int i = 0; i < good_matches.size(); i++ )
				{
					  //-- Get the keypoints from the good matches
					scene.push_back( keypoints2[ good_matches[i].queryIdx ].pt );
					obj.push_back( keypoints1[ good_matches[i].trainIdx ].pt );
				
					scenek.push_back( keypoints2[ good_matches[i].queryIdx ] );
					objk.push_back( keypoints1[ good_matches[i].trainIdx ] );

				}
				geometry_msgs::Point pospt;
				
				if(good_matches.size()>3)
				{
					Mat H = findHomography( obj, scene, CV_RANSAC );
				
					//-- Get the corners from the image_1 ( the object to be "detected" )
					std::vector<Point2f> obj_corners(5);
					std::vector<Point3f> obj_3D(5);
					obj_corners[0] = cvPoint(0,0); 
					obj_corners[1] = cvPoint( img1.cols, 0 );
					obj_corners[2] = cvPoint( img1.cols, img1.rows ); 
					obj_corners[3] = cvPoint( 0, img1.rows );
					obj_corners[4] = cvPoint( img1.cols/2, img1.rows/2 );
					obj_3D[0] = Point3f(-0.26,0.26,0); 
					obj_3D[1] = Point3f( 0.26, 0.26,0 );
					obj_3D[2] = Point3f( 0.26, -0.26,0 ); 
					obj_3D[3] = Point3f( 0, 0.26 ,0);
					obj_3D[4] = Point3f( 0.0, 0.0 ,0);
					std::vector<Point2f> scene_corners(5);
				
					perspectiveTransform( obj_corners, scene_corners, H);

	  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
					  

			
						

						Mat img_matches;
						
						cv::Mat img_keypoints_2;

			  		  drawMatches(  img2, keypoints2,img1, keypoints1,good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		
		
					good_matches.clear();
					obj.clear();
					scene.clear();
					objk.clear();
					scenek.clear();

					line( img_matches, scene_corners[0], scene_corners[1] , Scalar(0, 255, 0), 4 );
					  line( img_matches, scene_corners[1] , scene_corners[2] , Scalar( 0, 255, 0), 4 );
					  line( img_matches, scene_corners[2] , scene_corners[3] , Scalar( 0, 255, 0), 4 );
					  line( img_matches, scene_corners[3] , scene_corners[0] , Scalar( 0, 255, 0), 4);
					circle( img_matches, scene_corners[4], 32.0, Scalar( 0, 0, 255 ), 4, 8 );
					
					//PNP stuff
					solvePnP(Mat(obj_3D),Mat(scene_corners),Matrix,distortion,rvec,tvec,false,CV_ITERATIVE);
					//printf("%f,%f,%f, FEATURE\n",tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0));
					//printf("[%f,%f,%f] \n",tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0));
					//Comparacion!
					if(keyPointsBlob.size()==4)
					{
						float xmedia=tvec.at<double>(0,0)-tvec_col.at<double>(0,0);
						float ymedia=tvec.at<double>(1,0)-tvec_col.at<double>(1,0);
						float zmedia=tvec.at<double>(2,0)-tvec_col.at<double>(2,0);
						printf("%f,%f,%f \n",xmedia,ymedia,zmedia);
					}
					//Watch out big values
					if(abs(tvec.at<double>(0,0))>1500)
					{tvec.at<double>(0,0)=tvec.at<double>(0,0)/abs(tvec.at<double>(0,0));}
					if(abs(tvec.at<double>(1,0))>1500)
					{tvec.at<double>(1,0)=tvec.at<double>(1,0)/abs(tvec.at<double>(1,0));}
					if(abs(tvec.at<double>(2,0))>1500)
					{tvec.at<double>(2,0)=tvec.at<double>(2,0)/abs(tvec.at<double>(2,0));}
					//Watch out big changes
					if(abs(tvec.at<double>(0,0)-xpast)>500)
					{tvec.at<double>(0,0)=xpast;}
					if(abs(tvec.at<double>(1,0)-ypast)>500)
					{tvec.at<double>(1,0)=ypast;}
					if(abs(tvec.at<double>(2,0)-zpast)>500)
					{tvec.at<double>(2,0)=zpast;}
					
					



					punto.x=tvec.at<double>(0,0);
					punto.y=tvec.at<double>(1,0);
					punto.z=tvec.at<double>(2,0);
					
					imshow("view", img_matches );
					xpast=tvec.at<double>(0,0);
					ypast=tvec.at<double>(1,0);
					zpast=tvec.at<double>(2,0);
				}
				else
				{
					punto.x=0;
					punto.y=0;
					punto.z=0.7;
					found=0;
					printf("holi");
				}

				meas.at<float>(0) = punto.x;
				meas.at<float>(1) = punto.y;
				meas.at<float>(2) = punto.z;

				
				if(found==0)
				{
				 	kf.errorCovPre.at<float>(0,0) = 1; // px
					kf.errorCovPre.at<float>(1,1) = 1; // px
					kf.errorCovPre.at<float>(2,2) = 1;
					kf.errorCovPre.at<float>(3,3) = 1;
					kf.errorCovPre.at<float>(4,4) = 1; // px
					kf.errorCovPre.at<float>(5,5) = 1; // px
		
					state.at<float>(0) = meas.at<float>(0);
					state.at<float>(1) = meas.at<float>(1);
					state.at<float>(2) = meas.at<float>(2);
					state.at<float>(3) = 0.0;
					state.at<float>(4) = 0.0;
					state.at<float>(5) = 0.0;
		
					// <<<< Initialization
					found = 1;
				}
	
				state=kf.predict();
	
	
				 estimated=kf.correct(meas);
				if(estimated.at<float>(0)!=estimated.at<float>(0))
				{
					found=0;
					estimated.at<float>(0)=-200;
					estimated.at<float>(1)=-170;
					estimated.at<float>(2)=500;
					estimated.at<float>(3)=0;
					estimated.at<float>(4)=0;
					estimated.at<float>(5)=0;
					printf("hola");
				}
				//printf("[%f,%f,%f] \n",estimated.at<float>(0),estimated.at<float>(1),estimated.at<float>(2));
				pospt.x=estimated.at<float>(0);
				pospt.y=estimated.at<float>(1);
				pospt.z=estimated.at<float>(2);
				myfile << pospt.x<<"   "<< punto.x<<"   "<< pospt.y<<"   "<< punto.y<<"   "<< pospt.z<<"   "<< punto.z<<"\n";
				pub_point.publish(pospt);
			cv::waitKey(15);
			ros::spinOnce();
			
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
	cv::namedWindow("blob");
	cv::startWindowThread();
	//Blob detection initialization
	params.minThreshold = 40;
        params.maxThreshold = 60;
        params.thresholdStep = 5;
	params.minCircularity=0.5;
        params.minArea = 100;
        params.minConvexity = 0.3;
        params.minInertiaRatio = 0.01;
	
        params.maxArea = 8000;
        params.maxConvexity = 10;
	  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByArea = true;
        params.filterByColor = false;
        params.filterByCircularity = false;
	params.blobColor = 220;	
	//Initializing camera matrix
	Matrix.at<double>(0,0)=560.21;
	Matrix.at<double>(0,1)=0.0;
	Matrix.at<double>(0,2)=336.8;
	Matrix.at<double>(1,0)=0.0;
	Matrix.at<double>(1,1)=558.91;
	Matrix.at<double>(1,2)=172.37;
	Matrix.at<double>(2,0)=0.0;
	Matrix.at<double>(2,1)=0.0;
	Matrix.at<double>(2,2)=1.0;
	//Initializing distortion vector
	distortion[0]=-0.521;
	distortion[1]=0.278;
	distortion[2]=0.0009;
	distortion[3]=0.0008;
	distortion[4]=0;


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
	kf.processNoiseCov.at<float>(0,0) = 1e-1;
	kf.processNoiseCov.at<float>(1,1) = 1e-1;
	kf.processNoiseCov.at<float>(2,2) = 1e-1;
	kf.processNoiseCov.at<float>(3,3) = 2.0f;
	kf.processNoiseCov.at<float>(4,4) = 2.0f;
	kf.processNoiseCov.at<float>(5,5) = 2.0f;
	
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

	//Subscribing to ardrone camera node
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/ardrone/image_raw", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
	cv::destroyWindow("blob");
}


