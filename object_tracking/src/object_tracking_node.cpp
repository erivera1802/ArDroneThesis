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
#include <geometry_msgs/PointStamped.h>
#include <fstream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/video.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "ardrone_autonomy/Navdata.h"
using namespace cv;
using namespace std;
using namespace message_filters;
float ey,ez,ex;
float refz;
float xpast=0,ypast=0,zpast=0,xpastc=0,ypastc=0,zpastc=0;
//Values for Trackbar, Hue, Saturation Value
int iLowH=144;
int iHighH=5;
int iLowS=123;
int iHighS=185;
int iLowV=124;
int iHighV=194;
//Message initialization
geometry_msgs::Point point_msg;
std_msgs::Empty emp_msg;
geometry_msgs::Point punto;
geometry_msgs::Point puntoc;
geometry_msgs::Twist twist_msg;

//blob detection
SimpleBlobDetector::Params params;
std::vector<Point3f> col_3D(4);   
std::vector<Point2f> color_keypoints(4); 
float colorx,colory,colorz,fx,fy,fz;        
//Camera Matrix:
cv::Mat Matrix(3,3,CV_64F);
//Translation and rotation matrices
cv::Mat tvec;
cv::Mat rvec;
cv::Mat tvec_col;
cv::Mat rvec_col;
cv::Mat estimated;
cv::Mat estimatedc;
cv::Mat corrected;
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

cv::KalmanFilter kfc(stateSize, measSize, 3, type);
cv::Mat statec(stateSize, 1, type);  // [x,y,z,v_x,v_y,v_z]
cv::Mat measc(measSize, 1, type);    // [z_x,z_y,z_z]
cv::Mat cn(3,1,type);
cv::Mat estimatedPast(stateSize,1,type);
cv::Mat estimatedPastc(stateSize,1,type);
cv::Mat Speed(3,1,type);
int foundc=0;
int foundf=0;
//Simple Test Z
float zar,zab,zprom;
float estimatedx;
float estimatedy;
float estimatedz;
float estimatedvx;
float estimatedvy;
float estimatedvz;
geometry_msgs::Point posptav;
geometry_msgs::PointStamped posptavs;
geometry_msgs::Point Speedav;
float estimatedcx;
float estimatedcy;
float estimatedcz;
float estimatedcvx;
float estimatedcvy;
float estimatedcvz;
int lost;
int foundtotal;
cv::Mat EstimatedPose;
cv::Mat State;
cv::Mat Control(3,1,type);
cv::Mat ControlSaturado(3,1,type);
cv::Mat Reference(3,1,type);
ros::Publisher pub_control;
ros::Publisher pub_point;
ros::Publisher pub_vel;
Mat sp(3,1,type);
Mat spe(3,1,type);
double start_time;
Mat Saturacion(Mat seal)
{
	cv::Mat Sat(3,1,type);
		
	if(abs(seal.at<float>(0))>1.0)
	{
		Sat.at<float>(0)=seal.at<float>(0)/seal.at<float>(0);
	}
	else
	{
		Sat.at<float>(0)=seal.at<float>(0);
	}
	
	if(abs(seal.at<float>(1))>1.0)
	{
		Sat.at<float>(1)=seal.at<float>(1)/seal.at<float>(1);
	}
	else
	{
		Sat.at<float>(1)=seal.at<float>(1);
	}

	if(abs(seal.at<float>(2))>1.0)
	{
		Sat.at<float>(2)=seal.at<float>(2)/seal.at<float>(2);
	}
	else
	{
		Sat.at<float>(2)=seal.at<float>(2);
	}
	return Sat;
}
Mat PID(geometry_msgs::Point Pose,geometry_msgs::Point vel,Mat reference)
{
	float ex,ey,ez,dex,dey,dez,expa,eypa,ezpa;
	float kx=0.4;
	float ky=0.4;
	float kz=0.4;
	float kvx=0.4;
	float kvy=0.4;
	float kvz=0.4;
	int inicio=0;
	Mat control(3,1,type);
	ex=reference.at<float>(0)-Pose.x;
	ey=reference.at<float>(1)-Pose.y;
	ez=-reference.at<float>(2)+Pose.z;
	control.at<float>(0)=kx*ex+kvx*(-vel.x);
	control.at<float>(1)=ky*ey+kvy*(-vel.y);
	control.at<float>(2)=kz*ez+kvy*(-vel.z);
	return control;
	

}
Mat PnPStuff(std::vector<Point3f> Obj3D,std::vector<Point2f> color_key,Mat Matric,vector<double> distortionv,Mat rve,Mat tve,float xp,float yp,float zp)
{
	solvePnP(Mat(Obj3D),Mat(color_key),Matric,distortionv,rve,tve,false,CV_ITERATIVE);
	if(abs(tve.at<double>(0,0))>1500)
	{tve.at<double>(0,0)=tve.at<double>(0,0)/abs(tve.at<double>(0,0));}
	if(abs(tve.at<double>(1,0))>1500)
	{tve.at<double>(1,0)=tve.at<double>(1,0)/abs(tve.at<double>(1,0));}
	if(abs(tve.at<double>(2,0))>1500)
	{tve.at<double>(2,0)=tve.at<double>(2,0)/abs(tve.at<double>(2,0));}
	//Watch out big changes
	if(abs(tve.at<double>(0,0)-xp)>500)
	{tve.at<double>(0,0)=xp;}
	if(abs(tve.at<double>(1,0)-yp)>500)
	{tve.at<double>(1,0)=yp;}
	if(abs(tve.at<double>(2,0)-zp)>1)
	{tve.at<double>(2,0)=zp;}
	return tve;
	
}
geometry_msgs::Point Selection(geometry_msgs::Point Fea,geometry_msgs::Point Col,int F,int C, KalmanFilter KF,KalmanFilter KC,int pov)
{
	geometry_msgs::Point Preturn;
	
	if(F && C)
	{
		Preturn.x=(Fea.x);
		Preturn.y=(Fea.y);
		Preturn.z=(Fea.z);	
	}
	else if (F==1 && C==0)
	{
		Preturn.x=Fea.x;
		Preturn.y=Fea.y;
		Preturn.z=Fea.z;
	
	}
	else if (F==0 && C==1)
	{
		Preturn.x=Col.x;
		Preturn.y=Col.y;
		Preturn.z=Col.z;
	}
	else if(F==0 && C==0)
	{
		Preturn.x=-0.314;
		Preturn.y=-0.314;
		Preturn.z=-0.314;

		
			
	}
	return Preturn;
}
Mat Kalman(Mat Measure,KalmanFilter K)
{
							
	State=K.predict();	
	EstimatedPose=K.correct(Measure);
	return EstimatedPose;
}
Mat NaNCheck(Mat check,Mat past)
{
	if(check.at<float>(0)!=check.at<float>(0))
		{
			
		check.at<float>(0)=past.at<float>(0);
		check.at<float>(1)=past.at<float>(1);
		check.at<float>(2)=past.at<float>(2);
		check.at<float>(3)=past.at<float>(3);
		check.at<float>(4)=past.at<float>(4);
		check.at<float>(5)=past.at<float>(5);
		printf("hola");
		}
		past.at<float>(0)=check.at<float>(0);
		past.at<float>(1)=check.at<float>(1);
		past.at<float>(2)=check.at<float>(2);
		past.at<float>(3)=check.at<float>(3);
		past.at<float>(4)=check.at<float>(4);
		past.at<float>(5)=check.at<float>(5);
	return check;
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		ros::Rate loop_rate(50);
		ofstream myfile("/home/edrone/PosicionxSinModelo",ios_base::app);
		int count=0;
		//Node initialization
		Point pt;
		ros::NodeHandle neu;
		ros::Publisher pub_empty_land;
		pub_point=neu.advertise<geometry_msgs::PointStamped>("color_position",1);
		pub_vel=neu.advertise<geometry_msgs::PointStamped>("color_velocity",1);
		pub_control=neu.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		pub_empty_land = neu.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
	
		//Communicating between ROS an OpenCV
		cv_bridge::CvImagePtr cv_ptr; 
		cv::Mat img_thr;
		std::vector<Point3f> col_3D(4);   
		std::vector<Point2f> color_keypoints(4);  

				//Color Tracking Initialization
				std::string image = ros::package::getPath("stream") + "/src/gbox.jpg";
				cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				cv::Mat img2;
				cv::Mat color;
				cv::Mat gauss;
				cv::Mat gauss1;
				cv::Mat gauss2;
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
				if(iLowH<iHighH)
				{
				cv::inRange(gauss, cv::Scalar(iLowH,iLowS,iLowV), cv::Scalar(iHighH,iHighS,iHighV), gauss);
				}
				else
				{
				cv::inRange(gauss, cv::Scalar(iLowH,iLowS,iLowV), cv::Scalar(179,iHighS,iHighV), gauss1);
				cv::inRange(gauss, cv::Scalar(0,iLowS,iLowV), cv::Scalar(iHighH,iHighS,iHighV), gauss2);
				gauss=gauss1+gauss2;
				} 
				imshow("Color", color);
				//Color reference
				/*col_3D[0] = Point3f(-0.115,-0.08,0); 
				col_3D[1] = Point3f(0.115,-0.08,0);
				col_3D[2] = Point3f(-0.24,0.245,0); 
				col_3D[3] = Point3f(0.24,0.245,0);*/
				col_3D[0] = Point3f(-0.051,-0.065,0); 
				col_3D[1] = Point3f(0.051,-0.065,0);
				col_3D[2] = Point3f(-0.096,0.098,0); 
				col_3D[3] = Point3f(0.096,0.098,0);
				//Thresholding
				cv::cvtColor(cv_ptr->image,img2,CV_BGR2GRAY);
				Mat img1 = imread(image, CV_LOAD_IMAGE_GRAYSCALE );
				geometry_msgs::Point posptc;
				if( img1.empty() ) 
				{
			 		printf("Error occured, image not read correctly \n");
				}
				//Blob detection
				vector<KeyPoint> keyPointsBlob;
				SimpleBlobDetector blobDetector( params );
				blobDetector.detect( gauss, keyPointsBlob );
				if(keyPointsBlob.size()==4)//Just work if find 4 blobs
				{
					drawKeypoints(gauss,keyPointsBlob,blobs,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
					imshow("blob", blobs );
					for(int a=0;a<keyPointsBlob.size();a=a+1)
					{
						color_keypoints[a]=keyPointsBlob[a].pt;
					}

					//PnP color Stuff
					tvec_col=PnPStuff(col_3D,color_keypoints,Matrix,distortion,rvec_col,tvec_col,xpastc,ypastc,zpastc);
					
					



					puntoc.x=tvec_col.at<double>(0,0);
					puntoc.y=tvec_col.at<double>(1,0);
					puntoc.z=tvec_col.at<double>(2,0);
					
					//imshow("view", img_matches );
					xpastc=tvec_col.at<double>(0,0);
					ypastc=tvec_col.at<double>(1,0);
					zpastc=tvec_col.at<double>(2,0);
					//Straight to Kalman
					measc.at<float>(0) = puntoc.x;
					measc.at<float>(1) = puntoc.y;
					measc.at<float>(2) = puntoc.z;

					foundc=1;
					

				}
				else
				{										
					puntoc.x=xpastc;
					puntoc.y=ypastc;
					puntoc.z=zpastc;
					measc.at<float>(0) = puntoc.x;
					measc.at<float>(1) = puntoc.y;
					measc.at<float>(2) = puntoc.z;
					foundc=0;
					//printf("holi \n");
					imshow("blob", gauss );
				}
				geometry_msgs::Point pospt;
				geometry_msgs::Point Speedf;
				geometry_msgs::PointStamped Speedc;
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
				    const float ratio = 0.6; // As in Lowe's paper; can be tuned
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
				
				
				if(good_matches.size()>9)//9
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

					line( img_matches, scene_corners[0], scene_corners[1] , Scalar(255, 255, 0), 4 );
					  line( img_matches, scene_corners[1] , scene_corners[2] , Scalar( 0, 255, 0), 4 );
					  line( img_matches, scene_corners[2] , scene_corners[3] , Scalar( 255, 255, 0), 4 );
					  line( img_matches, scene_corners[3] , scene_corners[0] , Scalar( 0, 255, 0), 4);
					circle( img_matches, scene_corners[4], 32.0, Scalar( 0, 0, 255 ), 4, 8 );
					//Test for simplier Z
					zar=scene_corners[0].x-scene_corners[1].x;
					zab=scene_corners[2].x-scene_corners[3].x;
					zprom=abs(zar+zab)/2.0;
					//PNP stuff
	
					tvec=PnPStuff(obj_3D,scene_corners,Matrix,distortion,rvec,tvec,xpast,ypast,zpast);
					



					punto.x=tvec.at<double>(0,0);
					punto.y=tvec.at<double>(1,0);
					punto.z=tvec.at<double>(2,0);

					fx=tvec.at<double>(0,0);
					fy=tvec.at<double>(1,0);
					fz=tvec.at<double>(2,0);
					//Straight to Kalman
					meas.at<float>(0) = punto.x;
					meas.at<float>(1) = punto.y;
					meas.at<float>(2) = punto.z;
					imshow("view", img_matches );
					xpast=tvec.at<double>(0,0);
					ypast=tvec.at<double>(1,0);
					zpast=tvec.at<double>(2,0);
					foundf=1;
					
				}
				else
				{
					punto.x=xpast;
					punto.y=ypast;
					punto.z=zpast;
					meas.at<float>(0) = punto.x;
					meas.at<float>(1) = punto.y;
					meas.at<float>(2) = punto.z;
					foundf=0;
					//printf("hole \n");
					
				}

				


				

				//Kalman Prediction and Correction	
				estimatedc=Kalman(measc,kfc);
				estimated=Kalman(meas,kf);
				
				//Check for Nan
				estimated=NaNCheck(estimated,estimatedPast);
				estimatedc=NaNCheck(estimatedc,estimatedPastc);
				

				//Save Kalman for ROS message				
				pospt.x=estimated.at<float>(0);
				pospt.y=estimated.at<float>(1);
				pospt.z=estimated.at<float>(2);
				Speedf.x=estimated.at<float>(3);
				Speedf.y=estimated.at<float>(4);
				Speedf.z=estimated.at<float>(5);

				posptc.x=estimatedc.at<float>(0);
				posptc.y=estimatedc.at<float>(1);
				posptc.z=estimatedc.at<float>(2);
				Speedc.point.x=estimated.at<float>(3);
				Speedc.point.y=estimated.at<float>(4);
				Speedc.point.z=estimated.at<float>(5);
				Speedc.header.stamp=ros::Time::now();

				/*if(foundc==1)
				{
				Speedc.point.x=estimatedc.at<float>(3);
				Speedc.point.y=estimatedc.at<float>(4);
				Speedc.point.z=estimatedc.at<float>(5);
				Speedc.header.stamp=ros::Time::now();
				


				posptavs.point.x=estimatedc.at<float>(0);
				posptavs.point.y=estimatedc.at<float>(1);
				posptavs.point.z=estimatedc.at<float>(2);
				posptavs.header.stamp=ros::Time::now();
				}
				else if(foundc==0)
				{
				Speedc.point.x=-0;
				Speedc.point.y=-0;
				Speedc.point.z=-0;
				Speedc.header.stamp=ros::Time::now();

				posptavs.point.x=-0;
				posptavs.point.y=-0;
				posptavs.point.z=-0;
				posptavs.header.stamp=ros::Time::now();	
				}
				if(foundf==1)
				{

				
				Speedc.point.x=estimated.at<float>(0);
				Speedc.point.y=estimated.at<float>(1);
				Speedc.point.z=estimated.at<float>(2);
				Speedc.header.stamp=ros::Time::now();
				}
				else if(foundf==0)
				{
				Speedc.point.x=-0;
				Speedc.point.y=-0;
				Speedc.point.z=-0;
				Speedc.header.stamp=ros::Time::now();

				}*/
				//printf("[%f	%f	%f] \n",posptavs.point.x,posptavs.point.y,posptavs.point.z);

				posptav=Selection(pospt,posptc,foundf,foundc,kf,kfc,1);
				posptavs.point.x=posptav.x;
				posptavs.point.y=posptav.y;
				posptavs.point.z=posptav.z;
				printf("%f	%f	%f \n",posptavs.point.x,posptavs.point.y,posptavs.point.z);
				posptavs.header.stamp=ros::Time::now();
			pub_point.publish(posptavs);
			pub_vel.publish(Speedc);
			//pub_control.publish(twist_msg);
			//loop_rate.sleep();
			

	
	
	}

int main(int argc, char **argv)
{
	//Initializing ROS
	ros::init(argc, argv, "tracker");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);	
	cv::namedWindow("view");
	cv::namedWindow("blob");
	cv::namedWindow("Color");
	cv::startWindowThread();
	//Blob detection initialization
	params.minThreshold = 40;
        params.maxThreshold = 60;
        params.thresholdStep = 5;
	params.minCircularity=0.5;
        params.minArea = 30;
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
	lost=0;
	foundtotal=0;
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


	float dt=0.052;//Dont know it yet

	//Feature Kalman Filter
	//Transition Matrix A
	kf.transitionMatrix = cv::Mat::zeros(stateSize, stateSize, type);
	cv::setIdentity(kf.transitionMatrix);
	/*kfc.transitionMatrix.at<float>(0,3)=40;
	kfc.transitionMatrix.at<float>(1,4)=25;
	kfc.transitionMatrix.at<float>(2,5)=20;
	kfc.transitionMatrix.at<float>(3,3)=-10;
	kfc.transitionMatrix.at<float>(4,4)=-10;
	kfc.transitionMatrix.at<float>(5,5)=-10;*/
	kf.transitionMatrix.at<float>(0,3)=dt;
	kf.transitionMatrix.at<float>(1,4)=dt;
	kf.transitionMatrix.at<float>(2,5)=dt;
	//control matrix
	kf.controlMatrix=cv::Mat::zeros(7,3,type);
	kf.controlMatrix.at<float>(3,0)=6;
	kf.controlMatrix.at<float>(4,1)=4.5;
	kf.controlMatrix.at<float>(5,2)=100;
	//Measurement Matrix H
	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0,0) = 1.0f;
	kf.measurementMatrix.at<float>(1,1) = 1.0f;
	kf.measurementMatrix.at<float>(2,2) = 1.0f;
	//Noise Covariance Matrix Q
	kf.processNoiseCov.at<float>(0,0) = 1e-5;
	kf.processNoiseCov.at<float>(1,1) = 1e-5;
	kf.processNoiseCov.at<float>(2,2) = 1e-5;
	kf.processNoiseCov.at<float>(3,3) = 1e-4;
	kf.processNoiseCov.at<float>(4,4) = 1e-4;
	kf.processNoiseCov.at<float>(5,5) = 1e-4;
	kf.measurementNoiseCov.at<float>(0,0) = 1e-4;
	kf.measurementNoiseCov.at<float>(1,1) = 1e-4;
	kf.measurementNoiseCov.at<float>(2,2) = 1e-5;
	
	cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));

	//Color Kalman Filter
	//Transition Matrix A
	kfc.transitionMatrix = cv::Mat::zeros(stateSize, stateSize, type);
	cv::setIdentity(kfc.transitionMatrix);
	/*kfc.transitionMatrix.at<float>(0,3)=40;
	kfc.transitionMatrix.at<float>(1,4)=25;
	kfc.transitionMatrix.at<float>(2,5)=20;
	kfc.transitionMatrix.at<float>(3,3)=-10;
	kfc.transitionMatrix.at<float>(4,4)=-10;
	kfc.transitionMatrix.at<float>(5,5)=-10;*/
	kfc.transitionMatrix.at<float>(0,3)=dt;
	kfc.transitionMatrix.at<float>(1,4)=dt;
	kfc.transitionMatrix.at<float>(2,5)=dt;
	//control matrix
	kfc.controlMatrix=cv::Mat::zeros(7,3,type);
	kfc.controlMatrix.at<float>(3,0)=6;
	kfc.controlMatrix.at<float>(4,1)=4.5;
	kfc.controlMatrix.at<float>(5,2)=100;
	//Measurement Matrix H
	kfc.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kfc.measurementMatrix.at<float>(0,0) = 1.0f;
	kfc.measurementMatrix.at<float>(1,1) = 1.0f;
	kfc.measurementMatrix.at<float>(2,2) = 1.0f;
	//Noise Covariance Matrix Q
	kfc.processNoiseCov.at<float>(0,0) = 1e-5;
	kfc.processNoiseCov.at<float>(1,1) = 1e-5;
	kfc.processNoiseCov.at<float>(2,2) = 1e-5;
	kfc.processNoiseCov.at<float>(3,3) = 1e-4;
	kfc.processNoiseCov.at<float>(4,4) = 1e-4;
	kfc.processNoiseCov.at<float>(5,5) = 1e-4;
	kfc.measurementNoiseCov.at<float>(0,0) = 1e-4;
	kfc.measurementNoiseCov.at<float>(1,1) = 1e-4;
	kfc.measurementNoiseCov.at<float>(2,2) = 1e-5;
	
	cv::setIdentity(kfc.errorCovPost, cv::Scalar::all(0.1));

	//Trouble with estimated
	estimatedPast.at<float>(0)=0.0;
	estimatedPast.at<float>(1)=0.0;
	estimatedPast.at<float>(2)=0.0;
	estimatedPast.at<float>(3)=0.0;
	estimatedPast.at<float>(4)=0.0;
	estimatedPast.at<float>(5)=0.0;
	estimatedPastc.at<float>(0)=0.0;
	estimatedPastc.at<float>(1)=0.0;
	estimatedPastc.at<float>(2)=0.0;
	estimatedPastc.at<float>(3)=0.0;
	estimatedPastc.at<float>(4)=0.0;
	estimatedPastc.at<float>(5)=0.0;
	//Reference
	//Reference
	Reference.at<float>(0)=0.0;
	Reference.at<float>(1)=0.0;
	Reference.at<float>(2)=1;
	//Remember to delete!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	start_time =(double)ros::Time::now().toSec();
	//Subscribing to ardrone camera node
	image_transport::ImageTransport it(nh);
	
	image_transport::Subscriber sub = it.subscribe("/ardrone/image_raw", 1, imageCallback);
	
	ros::spin();
	cv::destroyWindow("view");
	cv::destroyWindow("blob");
	cv::destroyWindow("Color");
}


