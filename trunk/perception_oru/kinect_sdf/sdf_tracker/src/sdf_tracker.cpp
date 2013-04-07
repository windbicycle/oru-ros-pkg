#include <cmath>
#include <iostream>
#include <limits>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkXMLImageDataReader.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

#include <opencv2/core/core.hpp>

#include <time.h>
#include "sdf_tracker.h"

SDF_Parameters::SDF_Parameters()
{  
  image_width = 640;
  image_height = 480;
  interactive_mode = true;
  Wmax = 64.0;
  resolution = 0.01;
  XSize = 256;
  YSize = 256;
  ZSize = 256;
  Dmax = 0.1;
  Dmin = -0.04;
  pose_offset = Eigen::MatrixXd::Identity(4,4);
  robust_statistic_coefficient = 0.02;
  regularization = 0.01;
  min_pose_change = 0.01;
  min_parameter_update = 0.0001;
  raycast_steps = 12;
  fx = 520.0;
  fy = 520.0;
  cx = 319.5;
  cy = 239.5;
}

SDF_Parameters::~SDF_Parameters()
{}

SDFTracker::SDFTracker() {
    SDF_Parameters myparams = SDF_Parameters();
    this->init(myparams);
}

SDFTracker::SDFTracker(SDF_Parameters &parameters)
{
  this->init(parameters);
}

SDFTracker::~SDFTracker()
{
  
  this->deleteGrids();

  for (int i = 0; i < parameters_.image_height; ++i)
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

void SDFTracker::init(SDF_Parameters &parameters)
{
  parameters_ = parameters;

  int downsample=1;
  switch(parameters_.image_height)
  {
  case 480: downsample = 1; break; //VGA
  case 240: downsample = 2; break; //QVGA
  case 120: downsample = 4; break; //QQVGA
  }
  parameters_.fx /= downsample;
  parameters_.fy /= downsample;
  parameters_.cx /= downsample;
  parameters_.cy /= downsample;

  depthImage_ = new cv::Mat(parameters_.image_height,parameters_.image_width,CV_32FC1); 
  depthImage_denoised_ = new cv::Mat( parameters_.image_height,parameters_.image_width,CV_32FC1);

  validityMask_ = new bool*[parameters_.image_height];
  for (int i = 0; i < parameters_.image_height; ++i)
  {
    validityMask_[i] = new bool[parameters_.image_width];
  }   

  myGrid_ = new float**[parameters_.XSize];
  weightArray_ = new float**[parameters_.XSize];

  for (int i = 0; i < parameters_.XSize; ++i)
  {
    myGrid_[i] = new float*[parameters_.YSize];
    weightArray_[i] = new float*[parameters_.YSize];

    for (int j = 0; j < parameters_.YSize; ++j)
    {
      myGrid_[i][j] = new float[parameters_.ZSize];
      weightArray_[i][j] = new float[parameters_.ZSize];
    }
  }
    
  for (int x = 0; x < parameters_.XSize; ++x)
  {
    for (int y = 0; y < parameters_.YSize; ++y)
    {
      for (int z = 0; z < parameters_.ZSize; ++z)
      {
        myGrid_[x][y][z]=parameters_.Dmax;
        weightArray_[x][y][z]=0.0f;
      }
    }
  }
  quit_ = false;
  first_frame_ = true;
  Pose_ << 0.0,0.0,0.0,0.0,0.0,0.0;
  cumulative_pose_ << 0.0,0.0,0.0,0.0,0.0,0.0;
  Transformation_=parameters_.pose_offset*Eigen::MatrixXd::Identity(4,4);

  if(parameters_.interactive_mode)
  {
    cv::namedWindow( "Render", 0 );
  }
};

void SDFTracker::deleteGrids(void)
{
  for (int i = 0; i < parameters_.XSize; ++i)
  {
    for (int j = 0; j < parameters_.YSize; ++j)
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

}

void SDFTracker::makeTriangles(void)
{
  for (int i = 1; i < parameters_.XSize-2; ++i)
  {
    for (int j = 1; j < parameters_.YSize-2; ++j)
    {
      for (int k = 1; k < parameters_.ZSize-2; ++k)
      {
        Eigen::Vector4d CellOrigin = Eigen::Vector4d(double(i),double(j),double(k),1.0);
        //if(!validGradient(CellOrigin*parameters_.resolution)) continue;
        /*1*/marchingTetrahedrons(CellOrigin,1);
        /*2*/marchingTetrahedrons(CellOrigin,2);
        /*3*/marchingTetrahedrons(CellOrigin,3);
        /*4*/marchingTetrahedrons(CellOrigin,4);
        /*5*/marchingTetrahedrons(CellOrigin,5);
        /*6*/marchingTetrahedrons(CellOrigin,6);
      }
    }
  }
}

void SDFTracker::saveTriangles(const std::string filename)
{
  std::ofstream triangle_stream;
  triangle_stream.open(filename.c_str());
  
  for (int i = 0; i < triangles_.size()-3; i+=3)
  {
    triangle_stream << "v " << triangles_[i](0) << " " << triangles_[i](1) << " " << triangles_[i](2) <<std::endl;
    triangle_stream << "v " << triangles_[i+1](0) << " " << triangles_[i+1](1) << " " << triangles_[i+1](2) <<std::endl;
    triangle_stream << "v " << triangles_[i+2](0) << " " << triangles_[i+2](1) << " " << triangles_[i+2](2) <<std::endl; 
    triangle_stream << "f -1 -2 -3" << std::endl; 
  }
  triangle_stream.close();
}

Eigen::Vector4d 
SDFTracker::VertexInterp(double iso, Eigen::Vector4d &p1d, Eigen::Vector4d &p2d,double valp1, double valp2)
{
  double mu;
  Eigen::Vector4d p;

  if (fabs(iso-valp1) < 0.000001)
    {
      p <<  p1d(0) - parameters_.XSize/2*parameters_.resolution,
            p1d(1) - parameters_.YSize/2*parameters_.resolution,
            p1d(2) - parameters_.ZSize/2*parameters_.resolution; 
            p1d(3); 
      return(p);
    }
  if (fabs(iso-valp2) < 0.000001)
    {
      p <<  p2d(0) - parameters_.XSize/2*parameters_.resolution,
            p2d(1) - parameters_.YSize/2*parameters_.resolution,
            p2d(2) - parameters_.ZSize/2*parameters_.resolution; 
            p2d(3); 
      return(p);
    }
  if (fabs(valp1-valp2) < 0.000001)
    {
      p <<  p1d(0) - parameters_.XSize/2*parameters_.resolution,
            p1d(1) - parameters_.YSize/2*parameters_.resolution,
            p1d(2) - parameters_.ZSize/2*parameters_.resolution; 
            p1d(3); 
      return(p);
    }

  mu = (iso - valp1) / (valp2 - valp1);
  p(0) = p1d(0) + mu * (p2d(0) - p1d(0)) - parameters_.XSize/2*parameters_.resolution;
  p(1) = p1d(1) + mu * (p2d(1) - p1d(1)) - parameters_.YSize/2*parameters_.resolution;
  p(2) = p1d(2) + mu * (p2d(2) - p1d(2)) - parameters_.ZSize/2*parameters_.resolution;
  p(3) = 1.0;
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

  float eps = 10e-9; 

  double i,j,k;
  modf(location(0)/parameters_.resolution + parameters_.XSize/2, &i);
  modf(location(1)/parameters_.resolution + parameters_.YSize/2, &j);  
  modf(location(2)/parameters_.resolution + parameters_.ZSize/2, &k);
  
  if(std::isnan(i) || std::isnan(j) || std::isnan(k)) return false;

  int I = int(i)-1; int J = int(j)-1;   int K = int(k)-1;  
  
  if(I>=parameters_.XSize-4 || J>=parameters_.YSize-3 || K>=parameters_.ZSize-3 || I<=1 || J<=1 || K<=1)return false;

  float* W10 = &weightArray_[I+1][J+0][K];
  float* W20 = &weightArray_[I+2][J+0][K];
 
  float* W01 = &weightArray_[I+0][J+1][K];
  float* W11 = &weightArray_[I+1][J+1][K];
  float* W21 = &weightArray_[I+2][J+1][K];
  float* W31 = &weightArray_[I+3][J+1][K];
  
  float* W02 = &weightArray_[I+0][J+2][K];
  float* W12 = &weightArray_[I+1][J+2][K];
  float* W22 = &weightArray_[I+2][J+2][K];
  float* W32 = &weightArray_[I+3][J+2][K];

  float* W13 = &weightArray_[I+1][J+3][K];
  float* W23 = &weightArray_[I+2][J+3][K];

  if( W10[1] < eps || W10[2] < eps || 
      W20[1] < eps || W20[2] < eps || 
      
      W01[1] < eps || W01[2] < eps ||
      W11[0] < eps || W11[1] < eps || W11[2] < eps || W11[3] < eps ||
      W21[0] < eps || W21[1] < eps || W21[2] < eps || W21[3] < eps ||
      W31[1] < eps || W31[2] < eps ||
      
      W02[1] < eps || W02[2] < eps ||
      W12[0] < eps || W12[1] < eps || W12[2] < eps || W12[3] < eps ||
      W22[0] < eps || W22[1] < eps || W22[2] < eps || W22[3] < eps ||
      W32[1] < eps || W32[2] < eps ||
      
      W13[1] < eps || W13[2] < eps ||
      W23[1] < eps || W23[2] < eps 
      ) return false;
  else return true;
}


double 
SDFTracker::SDFGradient(const Eigen::Vector4d &location, int stepSize, int dim )
{
  double delta=parameters_.resolution*stepSize;
  Eigen::Vector4d location_offset = Eigen::Vector4d(0,0,0,1);
  location_offset(dim) = delta;

  return ((SDF(location+location_offset)) - (SDF(location-location_offset)))/(2.0*delta);
};

void 
SDFTracker::marchingTetrahedrons(Eigen::Vector4d &Origin, int tetrahedron)
{
  /*
  The following part is adapted from code found at:
  http://paulbourke.net/geometry/polygonise/

  (Paul Bourke / David Thoth)


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
       +------|------+ 1
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
  val0 = val1 = val2 = val3 = parameters_.Dmax;

  Eigen::Vector4d V0, V1, V2, V3;
    
  int i = int(Origin(0));
  int j = int(Origin(1));
  int k = int(Origin(2));

  switch(tetrahedron)
  {
    case 1:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i][j][k];
    V2 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2),1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*parameters_.resolution;      
    break;
    
    case 2:  
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j+1][k];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*parameters_.resolution;      
    break;
    
    case 3:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i][j+1][k+1];
    V1 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j+1][k];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2),1.0)*parameters_.resolution;      
    break;
    
    case 4:      
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val3 = myGrid_[i+1][j][k];
    V3 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2),1.0)*parameters_.resolution;      
    break;
      
    case 5:
    val0 = myGrid_[i][j][k+1];
    V0 = Eigen::Vector4d(Origin(0),Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k+1];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;      
    break;
    
    case 6:
    val0 = myGrid_[i+1][j+1][k+1];
    V0 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;
    val1 = myGrid_[i+1][j+1][k];
    V1 = Eigen::Vector4d(Origin(0)+1,Origin(1)+1,Origin(2),1.0)*parameters_.resolution;
    val2 = myGrid_[i+1][j][k+1];
    V2 = Eigen::Vector4d(Origin(0)+1,Origin(1),Origin(2)+1,1.0)*parameters_.resolution;
    val3 = myGrid_[i][j+1][k+1];
    V3 = Eigen::Vector4d(Origin(0),Origin(1)+1,Origin(2)+1,1.0)*parameters_.resolution;      
    break;
  }  
    
  if(val0>parameters_.Dmax-parameters_.resolution || val1>parameters_.Dmax-parameters_.resolution || val2>parameters_.Dmax-parameters_.resolution || val3>parameters_.Dmax-parameters_.resolution )
  
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
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));/**/
        }
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));/**/
        }
      }
      else if(val1 < 0) /*01,13,12*/
      {     
        if(tetrahedron == 2 ||tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));/**/
        }
        else if(tetrahedron == 1 ||tetrahedron == 3||tetrahedron == 4|| tetrahedron == 6)
        {
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));/**/
        }
      }
      else if(val2 < 0) /*02,12,23*/
      {     
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));/**/
        }
        else if(tetrahedron == 3||tetrahedron == 1 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));/**/
        }
      }
      else if(val3 < 0) /*03,32,31*/
      {     
        if(tetrahedron == 2 ||tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V3,V2,val3,val2));
          triangles_.push_back(VertexInterp(0,V3,V1,val3,val1));/**/
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangles_.push_back(VertexInterp(0,V3,V1,val3,val1));
          triangles_.push_back(VertexInterp(0,V3,V2,val3,val2));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));/**/
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
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V3,V1,val3,val1));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V2,V0,val2,val0));
          triangles_.push_back(VertexInterp(0,V3,V2,val3,val2));/**/
        }
        else if(tetrahedron == 4||tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 6)
        {
          triangles_.push_back(VertexInterp(0,V3,V1,val3,val1));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V3,V2,val3,val2));
          triangles_.push_back(VertexInterp(0,V2,V0,val2,val0));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));/**/
        }
      }
      else if(val1 < 0 && val2 < 0) /*13,32,02;02,01,13*/
      {     
        if(tetrahedron == 1 ||tetrahedron == 4 || tetrahedron == 6 || tetrahedron == 3)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V3,V2,val3,val2));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));/**/
        }
        else if(tetrahedron == 2||tetrahedron == 5)
        {
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V3,V2,val3,val2));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));/**/
        }
      }
      else if(val2 < 0 && val3 < 0)/*03,02,13;13,02,12*/
      {     
        if(tetrahedron == 2 || tetrahedron == 5)
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));/**/
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));/**/
        }
      }
      else if(val0 < 0 && val1 < 0)/*03,02,13;13,02,12*/
      {     
        if(tetrahedron == 3 ||tetrahedron == 6 || tetrahedron == 1 || tetrahedron == 4)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));/**/
        }
        else if(tetrahedron == 2 || tetrahedron == 5 )
        {
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));/**/
        }
      }
      else if(val1 < 0 && val3 < 0)/*01,12,32;32,30,01*/
      {     
        if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 || tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V3,V0,val3,val0));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));/**/
        }
        else if(tetrahedron == 2 ||tetrahedron == 5)
        {
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V3,V0,val3,val0));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));/**/
        }
      }
      else if(val0 < 0 && val2 < 0)/*01,03,32;32,12,01*/
      {
        if(tetrahedron == 1  ||tetrahedron == 3 ||tetrahedron == 4||tetrahedron == 6)
        {
          /*correct tetrahedrons for this winding: 1,3,4,6*/
          /*wrong tetrahedrons for this winding: 2,5*/
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));/**/
        }
        else if(tetrahedron == 5||tetrahedron == 2)
        {
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));/**/
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
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));/**/
        }
        else if(tetrahedron == 2 || tetrahedron == 5)
        {
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));/**/
        }
      }
      else if(val1 > 0)/*10,12,13*/
      {
        if(tetrahedron == 2 ||tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));/**/
        }
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6 )
        {
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V1,V2,val1,val2));
          triangles_.push_back(VertexInterp(0,V0,V1,val0,val1));/**/
        }
      }
      else if(val2 > 0) /*20,23,21*/
      {
        if(tetrahedron == 2 || tetrahedron == 5 )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V2,V1,val2,val1));/**/
        } 
        else if(tetrahedron == 1 ||tetrahedron == 3 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangles_.push_back(VertexInterp(0,V2,V1,val2,val1));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V0,V2,val0,val2));/**/
        }
      }
      else if(val3 > 0)/*30,31,32*/
      {
        if(tetrahedron == 2 ||tetrahedron == 5  )
        {
          /*correct tetrahedrons for this winding: 2,5*/
          /*wrong tetrahedrons for this winding: 1,3,4,6*/
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));/**/
        }
        else if(tetrahedron == 3 ||tetrahedron == 1 ||tetrahedron == 4 ||tetrahedron == 6)
        {
          triangles_.push_back(VertexInterp(0,V2,V3,val2,val3));
          triangles_.push_back(VertexInterp(0,V1,V3,val1,val3));
          triangles_.push_back(VertexInterp(0,V0,V3,val0,val3));/**/
        }
      }
      break;
    }
  }
};

void
SDFTracker::setDepth(const cv::Mat &depth)
{
  depth_mutex_.lock();
  depth.copyTo(*depthImage_);
  depth_mutex_.unlock();

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
}

void 
SDFTracker::FuseDepth(void)
{
   
  Eigen::Matrix4d camToWorld = Transformation_.inverse();
  Eigen::Vector4d camera = camToWorld * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  
  //Main 3D reconstruction loop
  for(int x = 0; x<parameters_.XSize; ++x)
  { 
  #pragma omp parallel for \
  shared(x)
    for(int y = 0; y<parameters_.YSize;++y)
    { 
      float* previousD = &myGrid_[x][y][0];
      float* previousW = &weightArray_[x][y][0];      
      for(int z = 0; z<parameters_.ZSize; ++z)
      {           
        //define a ray and point it into the center of a node
        Eigen::Vector4d ray((x-parameters_.XSize/2)*parameters_.resolution, (y- parameters_.YSize/2)*parameters_.resolution , (z- parameters_.ZSize/2)*parameters_.resolution, 1);        
        ray = camToWorld*ray;
        if(ray(2)-camera(2) < 0) continue;
        
        cv::Point2d uv;
        uv=To2D(ray,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy );
        
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
            
          if(Eta >= parameters_.Dmin)
          {
              
            double D = std::min(Eta,parameters_.Dmax);
                
            previousD[z] = (previousD[z] * previousW[z] + float(D) * W) /
                      (previousW[z] + W);

            previousW[z] = std::min(previousW[z] + W , float(parameters_.Wmax));

          }//within visible region 
        }//within bounds      
      }//z   
    }//y
  }//x
  return;
};


void 
SDFTracker::FuseDepth(const cv::Mat& depth)
{
 
  depth_mutex_.lock();
  depth.copyTo(*depthImage_);
  depth_mutex_.unlock();

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
    hasfused = true;
    Pose_ = EstimatePose();
  } 
  else
  {
    hasfused = false;
    first_frame_ = false;
    if(depthImage_->rows!=parameters_.image_height) std::cout << "depth image rows do not match given image height parameter"<<std::endl;

  } 
  
  Transformation_ = Twist(Pose_).exp()*Transformation_;
  
  transformations_.push_back(Transformation_);
  cumulative_pose_ += Pose_;
  Pose_ = Pose_ * 0.0;

  if(cumulative_pose_.norm() < parameters_.min_pose_change && hasfused ){Render(); return;}
  cumulative_pose_ *= 0.0;
  
  Eigen::Matrix4d camToWorld = Transformation_.inverse();
  Eigen::Vector4d camera = camToWorld * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  //Main 3D reconstruction loop
  
  for(int x = 0; x<parameters_.XSize; ++x)
  { 
  #pragma omp parallel for \
  shared(x)
    for(int y = 0; y<parameters_.YSize;++y)
    { 
      float* previousD = &myGrid_[x][y][0];
      float* previousW = &weightArray_[x][y][0];      
      for(int z = 0; z<parameters_.ZSize; ++z)
      {           
        //define a ray and point it into the center of a node
        Eigen::Vector4d ray((x-parameters_.XSize/2)*parameters_.resolution, (y- parameters_.YSize/2)*parameters_.resolution , (z- parameters_.ZSize/2)*parameters_.resolution, 1);        
        ray = camToWorld*ray;
        if(ray(2)-camera(2) < 0) continue;
        
        cv::Point2d uv;
        uv=To2D(ray,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy );
        
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
            
          if(Eta >= parameters_.Dmin)
          {
              
            double D = std::min(Eta,parameters_.Dmax);
                
            previousD[z] = (previousD[z] * previousW[z] + float(D) * W) /
                      (previousW[z] + W);

            previousW[z] = std::min(previousW[z] + W , float(parameters_.Wmax));

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
  
  if(std::isnan(location(0)+location(1)+location(2))) return parameters_.Dmax;
  
  x = modf(location(0)/parameters_.resolution + parameters_.XSize/2, &i);
  y = modf(location(1)/parameters_.resolution + parameters_.YSize/2, &j);  
  z = modf(location(2)/parameters_.resolution + parameters_.ZSize/2, &k);
    
  if(i>=parameters_.XSize-1 || j>=parameters_.YSize-1 || k>=parameters_.ZSize-1 || i<0 || j<0 || k<0)return parameters_.Dmax;

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
  const double eps = 10e-9;
  const double c = parameters_.robust_statistic_coefficient*parameters_.Dmax;
  
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
          Eigen::Vector4d currentPoint = camToWorld*To3D(row,col,depth,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy);
          
          if(!validGradient(currentPoint)) continue;
          double D = (SDF(currentPoint));
          double Dabs = fabs(D);
          if(D > parameters_.Dmax - eps || D < parameters_.Dmin + eps) continue;
          
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
      
      g *= scaling;
      A *= scaling;
      
      A = A + (parameters_.regularization)*Eigen::MatrixXd::Identity(6,6);
      xi = xi - A.ldlt().solve(g);
      Vector6d Change = xi-xi_prev;  
      double Cnorm = Change.norm();
      xi_prev = xi;
      if(Cnorm < parameters_.min_parameter_update) break;
    }//k
  }//level
  if(std::isnan(xi.sum())) xi << 0.0,0.0,0.0,0.0,0.0,0.0;
  return xi;
};//function

void 
SDFTracker::Render(void)
{
  //double minStep = parameters_.resolution/4;
  cv::Mat depthImage_out(parameters_.image_height,parameters_.image_width,CV_32FC1);
  cv::Mat preview(parameters_.image_height,parameters_.image_width,CV_8UC3);
  
  const Eigen::Matrix4d expmap = Transformation_;
  const Eigen::Vector4d camera = expmap * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  const Eigen::Vector4d viewAxis = (expmap * Eigen::Vector4d(0.0,0.0,1.0,0.0)).normalized();
  const double max_ray_length = 5.0;
  
  //Rendering loop
 #pragma omp parallel for 
  for(int u = 0; u < parameters_.image_height; ++u)
  {
    for(int v = 0; v < parameters_.image_width; ++v)
    {
      bool hit = false;

      Eigen::Vector4d p = expmap*To3D(u,v,1.0,parameters_.fx,parameters_.fy,parameters_.cx,parameters_.cy) - camera;
      p.normalize();
            
      double scaling = validityMask_[u][v] ? double(depthImage_->ptr<float>(u)[v])*0.7 : parameters_.Dmax;
      
      double scaling_prev=0;
      int steps=0;
      double D = parameters_.resolution;
      while(steps<parameters_.raycast_steps && scaling < max_ray_length && !hit)
      { 

        double D_prev = D;
        D = SDF(camera + p*scaling);
     
        if(D < 0.0)
        {
          scaling = scaling_prev + (scaling-scaling_prev)*D_prev/(D_prev - D);

          hit = true;
          Eigen::Vector4d normal_vector = Eigen::Vector4d::Zero();
 
          if(parameters_.interactive_mode)
          {  
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
        scaling += std::max(parameters_.resolution,D);  
        ++steps;        
      }//ray
      if(!hit)     
      {
        //Input values are better than nothing.
        depthImage_out.at<float>(u,v)=depthImage_->ptr<float>(u)[v];  
  
        if(parameters_.interactive_mode)
        {
          preview.at<cv::Vec3b>(u,v)[0]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[1]=uchar(30);
          preview.at<cv::Vec3b>(u,v)[2]=uchar(30);
        }
      }//no hit
    }//col
  }//row

  depthDenoised_mutex_.lock();
  depthImage_out.copyTo(*depthImage_denoised_);
  depthDenoised_mutex_.unlock();    
 
  if(parameters_.interactive_mode)
  {
 
    cv::imshow("Render", preview);//depthImage_denoised);
    char q = cv::waitKey(3);
    if(q == 'q' || q  == 27 || q  == 71 ) { quit_ = true; }//int(key)
  }
  return;
};

void SDFTracker::getDenoisedImage(cv::Mat &img) 
{
    depthDenoised_mutex_.lock();
    depthImage_denoised_->copyTo(img);
    depthDenoised_mutex_.unlock();          
}

bool SDFTracker::quit(void)
{
  return quit_;
}


Eigen::Matrix4d 
SDFTracker::getCurrentTransformation(void)
{
  Eigen::Matrix4d T;
  transformation_mutex_.lock();
  T = Transformation_;
  transformation_mutex_.unlock();
  return T;
}

void
SDFTracker::setCurrentTransformation(const Eigen::Matrix4d &T)
{
  transformation_mutex_.lock();
  Transformation_= T;
  transformation_mutex_.unlock();
}

void SDFTracker::saveSDF(const std::string &filename)
{

// http://www.vtk.org/Wiki/VTK/Examples/Cxx/IO/WriteVTI

  //vtkImageData *sdf_volume = vtkImageData::New();
  
  vtkSmartPointer<vtkImageData> sdf_volume = vtkSmartPointer<vtkImageData>::New();

  sdf_volume->SetDimensions(parameters_.XSize,parameters_.YSize,parameters_.ZSize);
  sdf_volume->SetOrigin(  parameters_.resolution*parameters_.XSize/2,
                          parameters_.resolution*parameters_.YSize/2,
                          parameters_.resolution*parameters_.ZSize/2);
   
  float spc = parameters_.resolution;
  sdf_volume->SetSpacing(spc,spc,spc);
  
  vtkSmartPointer<vtkFloatArray> distance = vtkSmartPointer<vtkFloatArray>::New();
  vtkSmartPointer<vtkFloatArray> weight = vtkSmartPointer<vtkFloatArray>::New();
  
  int numCells = parameters_.ZSize * parameters_.YSize * parameters_.XSize;

  distance->SetNumberOfTuples(numCells);
  weight->SetNumberOfTuples(numCells);

  int i, j, k, offset_k, offset_j;
  for(k=0;k < parameters_.ZSize; ++k)
  {
    offset_k = k*parameters_.XSize*parameters_.YSize;
    for(j=0; j<parameters_.YSize; ++j)
    {
      offset_j = j*parameters_.XSize;
      for(i=0; i<parameters_.XSize; ++i)
      {
        
        int offset = i + offset_j + offset_k;
        distance->SetValue(offset, myGrid_[i][j][k]);
        weight->SetValue(offset, weightArray_[i][j][k]);

     }
   }
 }

  sdf_volume->GetPointData()->AddArray(distance);
  distance->SetName("Distance");

  sdf_volume->GetPointData()->AddArray(weight);
  weight->SetName("Weight");

  vtkSmartPointer<vtkXMLImageDataWriter> writer =
  vtkSmartPointer<vtkXMLImageDataWriter>::New();
  writer->SetFileName(filename.c_str());

  writer->SetInput(sdf_volume);
  writer->Write();
}

void SDFTracker::loadSDF(const std::string &filename)
{
 
  // //double valuerange[2];
  vtkXMLImageDataReader *reader  = vtkXMLImageDataReader::New();
  reader->SetFileName(filename.c_str());
  // //reader->GetOutput()->GetScalarRange(valuerange);
  reader->Update();
  reader->UpdateWholeExtent();
  reader->UpdateInformation();

  vtkSmartPointer<vtkImageData> sdf_volume = vtkSmartPointer<vtkImageData>::New();
  sdf_volume = reader->GetOutput();

  //will segfault if not specified
  int* sizes = sdf_volume->GetDimensions();
  parameters_.XSize = sizes[0];
  parameters_.YSize = sizes[1];
  parameters_.ZSize = sizes[2];

  double* cell_sizes = sdf_volume->GetSpacing();
  parameters_.resolution = float(cell_sizes[0]);  //TODO add support for different scalings along x,y,z. 

  this->deleteGrids();
  this->init(parameters_);

  vtkFloatArray *distance =vtkFloatArray::New();
  vtkFloatArray *weight =vtkFloatArray::New();
  distance = (vtkFloatArray *)reader->GetOutput()->GetPointData()->GetScalars("Distance");
  weight = (vtkFloatArray *)reader->GetOutput()->GetPointData()->GetScalars("Weight");

  int i, j, k, offset_k, offset_j;
  for(k=0;k < parameters_.ZSize; ++k)
  {
    offset_k = k*parameters_.XSize*parameters_.YSize;
    for(j=0; j<parameters_.YSize; ++j)
    {
      offset_j = j*parameters_.XSize;
      for(i=0; i<parameters_.XSize; ++i)
      {
        int offset = i + offset_j + offset_k;
        myGrid_[i][j][k] = distance->GetValue(offset);
        weightArray_[i][j][k] = weight->GetValue(offset);        
      }
    }
  }
}
