//ROS implementation
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>
#define EIGEN_NO_DEBUG

#define N 200
#include "depth_fuse_functions.cpp"

//GLOBALS-----------------------------------------------------------------------

Vector6d Pose;

cv::Mat depthImage(480,640,CV_32FC1);

bool validityMask[480][640];

const double Wmax=64;
const double resolution = 0.02;
const double Dmax=0.2; 
const double Dmin=-0.04;

float myGrid[N][N][N]; 
float weightArray[N][N][N];

//------------------------------------------------------------------------------
//Main Function-----------------------------------------------------------------
//------------------------------------------------------------------------------
int main( int argc, char* argv[] )
{

 for(int x = 0; x<N; x++)
	{  
		for(int y=0; y<N;y++)
		{ 
      	#pragma omp parallel for
			for(int z = 0; z<N; z++)
			{	
			  myGrid[x][y][z]=Dmax;
			  weightArray[x][y][z]=0.0f;
 } } }

  Pose << 0.0,0.0,0.0,0.0,0.0,-0.40;

  cv::namedWindow("Render");
	ros::init(argc, argv, "depth_fuse");
	
	ros::NodeHandle n;
  
  ROS_INFO("INITIALIZATION OK");
  while(ros::ok())
  {
	  ros::Subscriber sub1 = n.subscribe("/camera/depth/image", 1, FuseDepth);
	  ros::spin();
	} 
  return(0);
}
