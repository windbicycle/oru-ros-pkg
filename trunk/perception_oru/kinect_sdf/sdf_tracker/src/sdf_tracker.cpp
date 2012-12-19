#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/core/core.hpp>

#include <time.h>
#include "sdf_tracker.h"

SDFTracker::SDFTracker()
{
  
  nh_ = ros::NodeHandle("~");
  n_ = ros::NodeHandle();

  nh_.param("ImageWidth",image_width_, 640);
  nh_.param("ImageHeight",image_height_, 480);

  int downsample=1;

  switch(image_height_)
  {
  case 480: downsample = 1; break; //VGA
  case 240: downsample = 2; break; //QVGA
  case 120: downsample = 4; break; //QQVGA
  }
  
  nh_.param("OutputTriangles",makeTris_, false);
  nh_.param("InteractiveMode", interactive_mode_, true);
  nh_.param("depth_registered",depth_registered_, false);
  nh_.param("MaxWeight",Wmax_, 64.0);
  nh_.param("CellSize",resolution_, 0.01);
  nh_.param("GridSizeX",XSize_, 256);
  nh_.param("GridSizeY",YSize_, 256);
  nh_.param("GridSizeZ",ZSize_, 256);
  nh_.param("offsetX",XOffset_, 0.0);
  nh_.param("offsetY",YOffset_, 0.0);
  nh_.param("offsetZ",ZOffset_, -0.40);
  nh_.param("PositiveTruncationDistance",Dmax_, 0.1);
  nh_.param("NegativeTruncationDistance",Dmin_, -0.04);
  nh_.param("RobustStatisticCoefficient", robust_statistic_coefficient_ , 0.02);
  nh_.param("Regularization", regularization_ , 0.01);
  nh_.param("MinPoseChangeToFuseData", min_pose_change_ , 0.01);
  nh_.param("ConvergenceCondition", min_parameter_update_ , 0.0001);
  nh_.param("MaximumRaycastSteps", raycast_steps_ , 12);
  nh_.param("FocalLengthX", fx_, 520.0/downsample);
  nh_.param("FocalLengthY", fy_, 520.0/downsample);
  nh_.param("CenterPointX", cx_, 319.5/downsample);
  nh_.param("CenterPointY", cy_, 239.5/downsample);
  nh_.param<std::string>("c_name",camera_name_,"camera");

  depthImage_ = new cv::Mat(image_height_,image_width_,CV_32FC1); 
  depthImage_denoised_ = new cv::Mat( image_height_,image_width_,CV_32FC1);

  validityMask_ = new bool*[image_height_];
  for (int i = 0; i < image_height_; ++i)
  {
    validityMask_[i] = new bool[image_width_];
  }   

  myGrid_ = new float**[XSize_];
  weightArray_ = new float**[XSize_];

  for (int i = 0; i < XSize_; ++i)
  {
    myGrid_[i] = new float*[YSize_];
    weightArray_[i] = new float*[YSize_];
    for (int j = 0; j < YSize_; ++j)
    {
      myGrid_[i][j] = new float[ZSize_];
      weightArray_[i][j] = new float[ZSize_];
    }
  }
    
  for (int x = 0; x < XSize_; ++x)
  {
    for (int y = 0; y < YSize_; ++y)
    {
      for (int z = 0; z < ZSize_; ++z)
      {
        myGrid_[x][y][z]=Dmax_;
        weightArray_[x][y][z]=0.0f;
      }
    }
  }
  quit_ = false;
  first_frame_=true;
  Pose_ << 0.0,0.0,0.0,0.0,0.0,0.0;
  Transformation_=Eigen::MatrixXd::Identity(4,4);
  Transformation_(0,3)+=XOffset_;
  Transformation_(1,3)+=YOffset_;
  Transformation_(2,3)+=ZOffset_;
};

SDFTracker::~SDFTracker()
{
  for (int i = 0; i < XSize_; ++i)
  {
    for (int j = 0; j < YSize_; ++j)
    {
      if (myGrid_[i][j]!=NULL)
      delete[] myGrid_[i][j];

      if (weightArray_[i][j]!=NULL)
      delete[] weightArray_[i][j];
    }
     
    if (myGrid_[i]!=NULL)
    delete[] myGrid_[i];
    if (weightArray_[i]!=NULL)
    delete[] weightArray_[i];   
  }

  delete[] myGrid_;
  delete[] weightArray_;  
  
  for (int i = 0; i < image_height_; ++i)
  {
    if ( validityMask_[i]!=NULL)
    delete[] validityMask_[i];
  }
  delete[] validityMask_;
  
  if(depthImage_!=NULL)
  delete depthImage_;

  if(depthImage_denoised_!=NULL)
  delete depthImage_denoised_;
  
};

void
SDFTracker::subscribeTopic(const std::string topic)
{
  
  std::string subscribe_topic = topic;

  if(depth_registered_)
  {
    if(topic=="default") subscribe_topic = camera_name_+"/depth_registered/image";
    depth_subscriber_ = n_.subscribe(subscribe_topic, 1, &SDFTracker::FuseDepth, this);
  }
  else
  {
    if(topic=="default") subscribe_topic = camera_name_+"/depth/image";
    depth_subscriber_ = n_.subscribe(subscribe_topic, 1, &SDFTracker::FuseDepth, this);
  }
}

void
SDFTracker::advertiseTopic(const std::string topic)
{
  std::string advertise_topic = topic;

  if(depth_registered_)
  {
    if(topic=="default") advertise_topic = "/"+camera_name_+"/depth_registered/image_denoised";
    depth_publisher_ = n_.advertise<sensor_msgs::Image>(advertise_topic, 10); 
  }
  else
  {
    if(topic == "default") advertise_topic = "/"+camera_name_+"/depth/image_denoised";
    depth_publisher_ = n_.advertise<sensor_msgs::Image>( advertise_topic ,10); 
  }

  heartbeat_depth_ = nh_.createTimer(ros::Duration(1.0), &SDFTracker::publishDepthDenoisedImage, this);

}

Eigen::Vector3d 
SDFTracker::VertexInterp(double iso, Eigen::Vector4d &p1d, Eigen::Vector4d &p2d,double valp1, double valp2)
{
  double mu;
  Eigen::Vector3d p;
  Eigen::Vector3d p1 = Eigen::Vector3d(p1d(0) , p1d(1), p1d(2) );
  Eigen::Vector3d p2 = Eigen::Vector3d(p2d(0) , p2d(1), p2d(2) );

  if (fabs(iso-valp1) < 0.000001)
    return(p1);
  if (fabs(iso-valp2) < 0.000001)
    return(p2);
  if (fabs(valp1-valp2) < 0.000001)
    return(p1);
  mu = (iso - valp1) / (valp2 - valp1);
  p(0) = p1d(0) + mu * (p2d(0) - p1d(0));
  p(1) = p1d(1) + mu * (p2d(1) - p1d(1));
  p(2) = p1d(2) + mu * (p2d(2) - p1d(2));

  return(p);  
};

Eigen::Matrix4d 
SDFTracker::Twist(const Vector6d &xi)
{
  Eigen::Matrix4d M;
  
  M << 0.0  , -xi(2),  xi(1), xi(3),
       xi(2), 0.0   , -xi(0), xi(4),
      -xi(1), xi(0) , 0.0   , xi(5),
       0.0,   0.0   , 0.0   , 0.0  ;
  
  return M;
};

Eigen::Vector4d 
SDFTracker::To3D(int row, int column, double depth, double fx, double fy, double cx, double cy)
{

  Eigen::Vector4d ret(double(column-cx)*depth/(fx),
                      double(row-cy)*depth/(fy),
                      double(depth),
                      1.0f);
  return ret;
};

cv::Point2d 
SDFTracker::To2D(const Eigen::Vector4d &location, double fx, double fy, double cx, double cy)
{
  cv::Point2d pixel(0,0);  
  if(location(2) != 0)
  {
     pixel.x = (cx) + location(0)/location(2)*(fx);
     pixel.y = (cy) + location(1)/location(2)*(fy);
  }
  
  return pixel;  
};

bool 
SDFTracker::validGradient(const Eigen::Vector4d &location)
{
 /* 
 The function tests the current location and its adjacent
 voxels for valid values (written at least once) to 
 determine if derivatives at this location are 
 computable in all three directions.

 Since the function SDF(Eigen::Vector4d &location) is a 
 trilinear interpolation between neighbours, testing the
 validity of the gradient involves looking at all the 
 values that would contribute to the final  gradient. 
 If any of these have a weight equal to zero, the result
 is false.
                      X--------X
                    /        / |
                  X--------X   ----X
                  |        |   | / |
              X----        |   X-------X
            /     |        | /       / |
          X-------X--------X-------X   |
          |     /        / |       |   |
          |   X--------X   |       |   |
     J    |   |        |   |       | /
     ^    X----        |   X-------X
     |        |        | / |  |
      --->I   X--------X   |  X
    /             |        | /
   v              X--------X
  K                                                */

  double i,j,k;
  modf(location(0)/resolution_ + XSize_/2, &i);
  modf(location(1)/resolution_ + YSize_/2, &j);  
  modf(location(2)/resolution_ + ZSize_/2, &k);
  
  if(std::isnan(i) || std::isnan(j) || std::isnan(k)) return false;

  int I = int(i)-1; int J = int(j)-1;   int K = int(k)-1;  
  
  if(I>=XSize_-4 || J>=YSize_-3 || K>=ZSize_-3 || I<=1 || J<=1 || K<=1)return false;

  float* D10 = &myGrid_[I+1][J+0][K];
  float* D20 = &myGrid_[I+2][J+0][K];
 
  float* D01 = &myGrid_[I+0][J+1][K];
  float* D11 = &myGrid_[I+1][J+1][K];
  float* D21 = &myGrid_[I+2][J+1][K];
  float* D31 = &myGrid_[I+3][J+1][K];
  
  float* D02 = &myGrid_[I+0][J+2][K];
  float* D12 = &myGrid_[I+1][J+2][K];
  float* D22 = &myGrid_[I+2][J+2][K];
  float* D32 = &myGrid_[I+3][J+2][K];

  float* D13 = &myGrid_[I+1][J+3][K];
  float* D23 = &myGrid_[I+2][J+3][K];

  if( !D10[1]==Dmax_ || !D10[2]==Dmax_ || 
      !D20[1]==Dmax_ || !D20[2]==Dmax_ || 
      
      !D01[1]==Dmax_ || !D01[2]==Dmax_ ||
      !D11[0]==Dmax_ || !D11[1]==Dmax_ || !D11[2]==Dmax_ || !D11[3]==Dmax_ ||
      !D21[0]==Dmax_ || !D21[1]==Dmax_ || !D21[2]==Dmax_ || !D21[3]==Dmax_ ||
      !D31[1]==Dmax_ || !D31[2]==Dmax_ ||
      
      !D02[1]==Dmax_ || !D02[2]==Dmax_ ||
      !D12[0]==Dmax_ || !D12[1]==Dmax_ || !D12[2]==Dmax_ || !D12[3]==Dmax_ ||
      !D22[0]==Dmax_ || !D22[1]==Dmax_ || !D22[2]==Dmax_ || !D22[3]==Dmax_ ||
      !D32[1]==Dmax_ || !D32[2]==Dmax_ ||
      
      !D13[1]==Dmax_ || !D13[2]==Dmax_ ||
      !D23[1]==Dmax_ || !D23[2]==Dmax_ 
      ) return false;
  else return true;
};

double 
SDFTracker::SDFGradient(const Eigen::Vector4d &location, int stepSize, int dim )
{
  double delta=resolution_*stepSize;
  Eigen::Vector4d location_offset = Eigen::Vector4d::Zero();
  location_offset(dim) = delta;

  return ((SDF(location+location_offset)) - (SDF(location-location_offset)))/(2.0*delta);
};

void 
SDFTracker::marchingTetrahedrons(Eigen::Vector4d &Origin, int tetrahedron)
{
  /*
  Function that outputs polygons from the SDF. The function is called
  giving a 3D location  of the (zero, zero) vertex and an index. 
  The index indicates which of the six possible tetrahedrons that can 
  be formed within a cube should be checked. 

      04===============05
      |\\              |\\
      ||\\             | \\
      || \\            |  \\
      ||  07===============06
      ||  ||           |   ||
      ||  ||           |   ||
      00--||-----------01  ||
       \\ ||            \  ||
        \\||             \ ||
         \||              \||
          03===============02

  Polygonise a tetrahedron given its vertices within a cube
  This is an alternative algorithm to Marching Cubes.
  It results in a smoother surface but more triangular facets.

              + 0
             /|\
            / | \
           /  |  \
          /   |   \
         /    |    \
        /     |     \
       +-------------+ 1
      3 \     |     /
         \    |    /
          \   |   /
           \  |  /
            \ | /
             \|/
              + 2

  Typically, for each location in space one would call:

  marchingTetrahedrons(CellOrigin,1);
  marchingTetrahedrons(CellOrigin,2);
  marchingTetrahedrons(CellOrigin,3);
  marchingTetrahedrons(CellOrigin,4);
  marchingTetrahedrons(CellOrigin,5);
  marchingTetrahedrons(CellOrigin,6);              
  */


  float val0, val1, val2, val3;
  val0 = val1 = val2 = val3 = Dmax_;

  Eigen::Vector4d V0, V1, V2, V3;
    
  int i = int(Origin(0));
  int j = int(Origin(1));
  int k = int(Origin(2));

  switch(tetrahedron)
  {
    case 1:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*resolution_;
    val1 = myGrid_[i+1][j][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*resolution_;
    val2 = myGrid_[i][j][k];
    V2 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2),1.0)*resolution_;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*resolution_;      
    break;
    
    case 2:  
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*resolution_;
    val1 = myGrid_[i+1][j][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*resolution_;
    val2 = myGrid_[i+1][j+1][k];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*resolution_;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*resolution_;      
    break;
    
    case 3:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*resolution_;
    val1 = myGrid_[i][j+1][k+1];
    V1 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*resolution_;
    val2 = myGrid_[i+1][j+1][k];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*resolution_;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*resolution_;      
    break;
    
    case 4:      
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*resolution_;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*resolution_;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*resolution_;
    val3 = myGrid_[i+1][j][k];
    V3 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*resolution_;      
    break;
      
    case 5:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*resolution_;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*resolution_;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*resolution_;
    val3 = myGrid_[i][j+1][k+1];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*resolution_;      
    break;
    
    case 6:
    val0 = myGrid_[i+1][j+1][k+1];
    V0 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2)+1,1.0)*resolution_;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*resolution_;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*resolution_;
    val3 = myGrid_[i][j+1][k+1];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*resolution_;      
    break;
  }  
    
  if(val0>Dmax_-resolution_ || val1>Dmax_-resolution_ || val2>Dmax_-resolution_ || val3>Dmax_-resolution_ )
  
    return;

  int count = 0;
  if(val0 < 0)count++;
  if(val1 < 0)count++;
  if(val2 < 0)count++;
  if(val3 < 0)count++;

  {
    switch(count)
    {
      case 0:
      case 4:
      break;

      case 1:
      /*One voxel has material*/
      if(val0 < 0) /*03,02,01*/
      {       
        if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 || tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 < 0) /*01,13,12*/
      {     
        if(tetrahedron == 2 ||tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3||tetrahedron == 4|| tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val2 < 0) /*02,12,23*/
      {     
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 3||tetrahedron == 1 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val3 < 0) /*03,32,31*/
      {     
        if(tetrahedron == 2 ||tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      break;

      case 2:
      /*two voxels have material*/
      if(val0 < 0 && val3 < 0)   /*01,02,31;31,02,32*/
      { 
        if(tetrahedron == 2 ||tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V0,val2,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 4||tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V3,V1,val3,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V0,val2,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 < 0 && val2 < 0) /*13,32,02;02,01,13*/
      {     
        if(tetrahedron == 1 ||tetrahedron == 4 || tetrahedron == 6 || tetrahedron == 3)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2||tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V2,val3,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val2 < 0 && val3 < 0)/*03,02,13;13,02,12*/
      {     
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val0 < 0 && val1 < 0)/*03,02,13;13,02,12*/
      {     
        if(tetrahedron == 3 ||tetrahedron == 6 || tetrahedron == 1 || tetrahedron == 4)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2 || tetrahedron == 5 )
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 < 0 && val3 < 0)/*01,12,32;32,30,01*/
      {     
        if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 || tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V0,val3,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2 ||tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V3,V0,val3,val0).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val0 < 0 && val2 < 0)/*01,03,32;32,12,01*/
      {
        if(tetrahedron == 1  ||tetrahedron == 3 ||tetrahedron == 4||tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 5||tetrahedron == 2)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"f -1 -2 -3" << std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      break;
      
      case 3:
      /*three voxels have material*/
      if(val0 > 0)/*03,01,02*/
      {       
        if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6 )
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 2 || tetrahedron == 5)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val1 > 0)/*10,12,13*/
      {
        if(tetrahedron == 2 ||tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6 )
        {
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V2,val1,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V1,val0,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val2 > 0) /*20,23,21*/
      {
        if(tetrahedron == 2 || tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V1,val2,val1).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        } 
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V1,val2,val1).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V2,val0,val2).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      else if(val3 > 0)/*30,31,32*/
      {
        if(tetrahedron == 2 ||tetrahedron == 5  )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
        else if(tetrahedron == 3 ||tetrahedron == 1 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangle_stream_<<"v "<< VertexInterp(0,V2,V3,val2,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V1,V3,val1,val3).transpose()<<std::endl;
          triangle_stream_<<"v "<< VertexInterp(0,V0,V3,val0,val3).transpose()<<std::endl;/**/
          triangle_stream_<<"f -1 -2 -3" << std::endl;
        }
      }
      break;
    }
  }
};

void 
SDFTracker::FuseDepth(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(msg, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform depth image.");
    return;
  }

  *depthImage_ = bridge->image;

  for(int row=0; row<depthImage_->rows-0; ++row)
  { 
    const float* Drow = depthImage_->ptr<float>(row);
    #pragma omp parallel for 
    for(int col=0; col<depthImage_->cols-0; ++col)
    { 
      if(!std::isnan(Drow[col]) && Drow[col]>0.4)
      {
      validityMask_[row][col]=true;
      }else
      {
        validityMask_[row][col]=false;
      }
    }
  }
  
  bool hasfused;
  if(!first_frame_)
  {
    if(quit_ && makeTris_)
    {
      triangle_stream_.open("triangles.obj");
      for (int i = 1; i < XSize_-2; ++i)
      {
        for (int j = 1; j < YSize_-2; ++j)
        {
          for (int k = 1; k < ZSize_-2; ++k)
          {
            Eigen::Vector4d CellOrigin = Eigen::Vector4d(double(i),double(j),double(k),1.0);
            //if(!validGradient(CellOrigin*resolution_)) continue;
            /*1*/marchingTetrahedrons(CellOrigin,1);
            /*2*/marchingTetrahedrons(CellOrigin,2);
            /*3*/marchingTetrahedrons(CellOrigin,3);
            /*4*/marchingTetrahedrons(CellOrigin,4);
            /*5*/marchingTetrahedrons(CellOrigin,5);
            /*6*/marchingTetrahedrons(CellOrigin,6);
          }
        }
      }
    
    triangle_stream_.close();
    }
    if(quit_) {ros::shutdown();}
    
    hasfused = true;
    Pose_ = EstimatePose();
  } 
  else
  {
    hasfused = false;
    first_frame_ = false;
    if(depthImage_->rows!=image_height_) 
      ROS_WARN("Getting images of unexpected size. Expected image height of %i, instead got %i! Make sure the parameter are set correctly", 
                image_height_, depthImage_->rows);
  } 
  
  Transformation_ = Twist(Pose_).exp()*Transformation_;
  
  transformations_.push_back(Transformation_);
  timestamps_.push_back(ros::Time::now());
  
  cumulative_pose_ += Pose_;
  Pose_ = Pose_ * 0.0;

  if(cumulative_pose_.norm() < min_pose_change_ && hasfused ){Render(); return;}
  cumulative_pose_ *= 0.0;
  
  Eigen::Matrix4d camToWorld = Transformation_.inverse();
  Eigen::Vector4d camera = camToWorld * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  //Main 3D reconstruction loop
  
  for(int x = 0; x<XSize_; ++x)
  { 
  #pragma omp parallel for 
    for(int y = 0; y<YSize_;++y)
    { 
      float* previousD = &myGrid_[x][y][0];
      float* previousW = &weightArray_[x][y][0];      
      for(int z = 0; z<ZSize_; ++z)
      {           
        //define a ray and point it into the center of a node
        Eigen::Vector4d ray((x-XSize_/2)*resolution_, (y- YSize_/2)*resolution_ , (z- ZSize_/2)*resolution_, 1);        
        ray = camToWorld*ray;
        if(ray(2)-camera(2) < 0)continue;
        
        cv::Point2d uv;
        uv=To2D(ray,fx_,fy_,cx_,cy_ );
        
        int j=floor(uv.x);
        int i=floor(uv.y);      
        
        //if the projected coordinate is within image bounds
        if(i>0 && i<depthImage_->rows-1 && j>0 && j <depthImage_->cols-1 && validityMask_[i][j] &&    
            validityMask_[i-1][j] && validityMask_[i][j-1])
        {
          const float* Di = depthImage_->ptr<float>(i);
          double Eta; 
          const float W=1/((1+Di[j])*(1+Di[j]));
            
          Eta=(double(Di[j])-ray(2));       
            
          if(Eta >= Dmin_)// && Eta<Dmax)
          {
              
            double D = std::min(Eta,Dmax_);//*copysign(1.0,Eta);*perpendicular
                
            previousD[z] = (previousD[z] * previousW[z] + float(D) * W) /
                      (previousW[z] + W);

            previousW[z] = std::min(previousW[z] + W , float(Wmax_));
          }//within visible region 
        }//within bounds      
      }//z   
    }//y
  }//x
  Render();
  return;
};

double 
SDFTracker::SDF(const Eigen::Vector4d &location)
{
  double i,j,k;
  double x,y,z;
  if(std::isnan(location(0)+location(1)+location(2))) return Dmax_;
  
  x = modf(location(0)/resolution_ + XSize_/2, &i);
  y = modf(location(1)/resolution_ + YSize_/2, &j);  
  z = modf(location(2)/resolution_ + ZSize_/2, &k);
    
  if(i>=XSize_-1 || j>=YSize_-1 || k>=ZSize_-1 || i<0 || j<0 || k<0)return Dmax_;

  int I = int(i); int J = int(j);   int K = int(k);
  
  float* N1 = &myGrid_[I][J][K];
  float* N2 = &myGrid_[I][J+1][K];
  float* N3 = &myGrid_[I+1][J][K];
  float* N4 = &myGrid_[I+1][J+1][K];

  double a1,a2,b1,b2;
  a1 = double(N1[0]*(1-z)+N1[1]*z);
  a2 = double(N2[0]*(1-z)+N2[1]*z);
  b1 = double(N3[0]*(1-z)+N3[1]*z);
  b2 = double(N4[0]*(1-z)+N4[1]*z);
    
  return double((a1*(1-y)+a2*y)*(1-x) + (b1*(1-y)+b2*y)*x);
};

Vector6d 
SDFTracker::EstimatePose(void) 
{
  Vector6d xi;
  xi<<0.0,0.0,0.0,0.0,0.0,0.0; // + (Pose-previousPose)*0.1;
  Vector6d xi_prev = xi;

  const double c = robust_statistic_coefficient_*Dmax_;
  
  const int iterations[3]={12, 8, 2};
  const int stepSize[3] = {4, 2, 1};

  for(int lvl=0; lvl < 3; ++lvl)
  {
    for(int k=0; k<iterations[lvl]; ++k)
    {

      const Eigen::Matrix4d camToWorld = Twist(xi).exp()*Transformation_;
      
      double A00=0.0,A01=0.0,A02=0.0,A03=0.0,A04=0.0,A05=0.0;
      double A10=0.0,A11=0.0,A12=0.0,A13=0.0,A14=0.0,A15=0.0;
      double A20=0.0,A21=0.0,A22=0.0,A23=0.0,A24=0.0,A25=0.0;
      double A30=0.0,A31=0.0,A32=0.0,A33=0.0,A34=0.0,A35=0.0;
      double A40=0.0,A41=0.0,A42=0.0,A43=0.0,A44=0.0,A45=0.0;
      double A50=0.0,A51=0.0,A52=0.0,A53=0.0,A54=0.0,A55=0.0;
      
      double g0=0.0, g1=0.0, g2=0.0, g3=0.0, g4=0.0, g5=0.0;
      
      for(int row=0; row<depthImage_->rows-0; row+=stepSize[lvl])
      {          
        #pragma omp parallel for \
        default(shared) \
        reduction(+:g0,g1,g2,g3,g4,g5,A00,A01,A02,A03,A04,A05,A10,A11,A12,A13,A14,A15,A20,A21,A22,A23,A24,A25,A30,A31,A32,A33,A34,A35,A40,A41,A42,A43,A44,A45,A50,A51,A52,A53,A54,A55)
        for(int col=0; col<depthImage_->cols-0; col+=stepSize[lvl])
        {
          if(!validityMask_[row][col]) continue;
          double depth = double(depthImage_->ptr<float>(row)[col]); 
          Eigen::Vector4d currentPoint = camToWorld*To3D(row,col,depth,fx_,fy_,cx_,cy_);
          
          if(!validGradient(currentPoint)) continue;
          double D = (SDF(currentPoint));
          double Dabs = fabs(D);
          if(D == Dmax_ || D == Dmin_) continue;
          
          //partial derivative of SDF wrt position  
          Eigen::Matrix<double,1,3> dSDF_dx(SDFGradient(currentPoint,1,0),
                                            SDFGradient(currentPoint,1,1),
                                            SDFGradient(currentPoint,1,2) 
                                            );
          //partial derivative of position wrt optimizaiton parameters
          Eigen::Matrix<double,3,6> dx_dxi; 
          dx_dxi << 0, currentPoint(2), -currentPoint(1), 1, 0, 0,
                    -currentPoint(2), 0, currentPoint(0), 0, 1, 0,
                    currentPoint(1), -currentPoint(0), 0, 0, 0, 1;

          //jacobian = derivative of SDF wrt xi (chain rule)
          Eigen::Matrix<double,1,6> J = dSDF_dx*dx_dxi;
          
          //double tukey = (1-(Dabs/c)*(Dabs/c))*(1-(Dabs/c)*(Dabs/c));
          double huber = Dabs < c ? 1.0 : c/Dabs;
          
          //Gauss - Newton approximation to hessian
          Eigen::Matrix<double,6,6> T1 = huber * J.transpose() * J;
          Eigen::Matrix<double,1,6> T2 = huber * J.transpose() * D;
          
          g0 = g0 + T2(0); g1 = g1 + T2(1); g2 = g2 + T2(2);
          g3 = g3 + T2(3); g4 = g4 + T2(4); g5 = g5 + T2(5);
          
          A00+=T1(0,0);A01+=T1(0,1);A02+=T1(0,2);A03+=T1(0,3);A04+=T1(0,4);A05+=T1(0,5);
          A10+=T1(1,0);A11+=T1(1,1);A12+=T1(1,2);A13+=T1(1,3);A14+=T1(1,4);A15+=T1(1,5);
          A20+=T1(2,0);A21+=T1(2,1);A22+=T1(2,2);A23+=T1(2,3);A24+=T1(2,4);A25+=T1(2,5);
          A30+=T1(3,0);A31+=T1(3,1);A32+=T1(3,2);A33+=T1(3,3);A34+=T1(3,4);A35+=T1(3,5);
          A40+=T1(4,0);A41+=T1(4,1);A42+=T1(4,2);A43+=T1(4,3);A44+=T1(4,4);A45+=T1(4,5);
          A50+=T1(5,0);A51+=T1(5,1);A52+=T1(5,2);A53+=T1(5,3);A54+=T1(5,4);A55+=T1(5,5);
        }//col
      }//row
      
      Eigen::Matrix<double,6,6> A;
      A<< A00,A01,A02,A03,A04,A05,
          A10,A11,A12,A13,A14,A15,
          A20,A21,A22,A23,A24,A25,
          A30,A31,A32,A33,A34,A35,
          A40,A41,A42,A43,A44,A45,
          A50,A51,A52,A53,A54,A55;
     double scaling = 1/A.maxCoeff();
      
      Vector6d g;
      g<< g0, g1, g2, g3, g4, g5;
      
      g = g * scaling;
      A = A * scaling;
      
      A = A + (regularization_)*Eigen::MatrixXd::Identity(6,6);
      xi = xi - A.ldlt().solve(g);
      Vector6d Change = xi-xi_prev;  
      double Cnorm = Change.norm();
      xi_prev = xi;
      if(Cnorm < min_parameter_update_) break;
    }//k
  }//level
  if(std::isnan(xi.sum())) xi << 0.0,0.0,0.0,0.0,0.0,0.0;
  return xi;
};//function

void 
SDFTracker::Render(void)
{
  double minStep = resolution_;
  cv::Mat depthImage_out(image_height_,image_width_,CV_32FC1);
  cv::Mat preview(image_height_,image_width_,CV_8UC3);
  

  const Eigen::Matrix4d expmap = Transformation_;
  const Eigen::Vector4d camera = expmap * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  const Eigen::Vector4d viewAxis = expmap * Eigen::Vector4d(0.0,0.0,1.0,0.0);
  
  //Rendering loop
  #pragma omp parallel for 
  for(int u = 0; u < image_height_; ++u)
  {
    for(int v = 0; v < image_width_; ++v)
    {
      
      Eigen::Vector4d p = expmap*To3D(u,v,1.0,fx_,fy_,cx_,cy_) - camera;
      p.normalize();
            
      double scaling = validityMask_[u][v] ? double(depthImage_->ptr<float>(u)[v])*0.8 : Dmax_;
      
      double scaling_prev=0;
      int steps=0;
      double D = resolution_;

      while(steps<raycast_steps_ && D>=minStep)
      { 
        double D_prev = D;
        D = SDF(camera + p*scaling);
     
        if(D < minStep && D_prev > 0)
        {
          scaling = scaling_prev - (scaling-scaling_prev) * D_prev /
                                   ( D - D_prev);
          if(scaling > 5.0) break;
          
          if(interactive_mode_)
          {
            Eigen::Vector4d normal_vector = Eigen::Vector4d::Zero();
            
            for(int ii=0; ii<3; ++ii)
            {
              normal_vector(ii) = fabs(SDFGradient(camera + p*scaling,1,ii));            
            }   
            normal_vector.normalize();

            preview.at<cv::Vec3b>(u,v)[0]=normal_vector(0)*255;
            preview.at<cv::Vec3b>(u,v)[1]=normal_vector(1)*255;
            preview.at<cv::Vec3b>(u,v)[2]=normal_vector(2)*255;  
          }
          
          depthImage_out.at<float>(u,v)=scaling*(viewAxis.dot(p));
        
        break;
        }
        scaling_prev = scaling;
        scaling = scaling + D;  
        ++steps;        
        
        //Input values are better than nothing.
        depthImage_out.at<float>(u,v)=depthImage_->ptr<float>(u)[v];  
   //     depthDenoised_mutex_.unlock();

        if(interactive_mode_)
        {
          preview.at<cv::Vec3b>(u,v)[0]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[1]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
        }
      }//ray
    }//col
  }//row
  depthDenoised_mutex_.lock();
  depthImage_out.copyTo(*depthImage_denoised_);
  depthDenoised_mutex_.unlock();    
  if(interactive_mode_)
  {
    cv::imshow("Render", preview);//depthImage_denoised);
    char q = cv::waitKey(3);
    if(q == 'q' || q  == 27 || q  == 71 ) { quit_ = true; }//int(key)
  }
  return;
};

void SDFTracker::publishDepthDenoisedImage(const ros::TimerEvent& event) 
{    
  if(depth_publisher_.getNumSubscribers()>0)
  {
    cv_bridge::CvImagePtr image_out (new cv_bridge::CvImage());
    image_out->header.stamp = ros::Time::now(); 
    image_out->encoding = "32FC1";      
    depthDenoised_mutex_.lock();
    depthImage_denoised_->copyTo(image_out->image);
    depthDenoised_mutex_.unlock();          
    depth_publisher_.publish(image_out->toImageMsg());     
  } 

  return;
};
