#include <boost/thread/mutex.hpp>

#include <fstream>
#include <iostream>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/StdVector>
#include <time.h>

#define EIGEN_USE_NEW_STDVECTOR

#ifndef SDF_TRACKER
#define SDF_TRACKER

class SDF_Parameters
{
public:
  bool makeTris;
  bool interactive_mode;
  int XSize;
  int YSize;
  int ZSize;
  int raycast_steps;
  int image_height;
  int image_width;
  double fx;
  double fy;
  double cx;
  double cy;
  double Wmax;
  double resolution;
  double Dmax; 
  double Dmin;
  Eigen::Matrix4d pose_offset;
  double robust_statistic_coefficient;
  double regularization;
  double min_parameter_update;
  double min_pose_change;

  SDF_Parameters();
  virtual ~SDF_Parameters();
};

typedef Eigen::Matrix<double,6,1> Vector6d; 

class SDFTracker
{
  protected:
  // variables
  std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > transformations_;
  std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > interest_points_;
  Eigen::Matrix4d Transformation_;
  Vector6d Pose_;
  Vector6d cumulative_pose_;
  cv::Mat *depthImage_;
  cv::Mat *depthImage_denoised_;

  boost::mutex transformation_mutex_;
  boost::mutex depth_mutex_;
  boost::mutex depthDenoised_mutex_;
  std::string camera_name_;
  
  bool** validityMask_;
  float*** myGrid_; 
  float*** weightArray_;    
  bool first_frame_;
  bool quit_;
  SDF_Parameters parameters_;
  std::ofstream triangle_stream_;
  // functions 
  Eigen::Vector3d VertexInterp(double iso, Eigen::Vector4d &p1d, Eigen::Vector4d &p2d,double valp1, double valp2);
  void marchingTetrahedrons(Eigen::Vector4d &Origin, int tetrahedron);
  virtual void init(SDF_Parameters &parameters);
  virtual void deleteGrids(void);

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual double SDF(const Eigen::Vector4d &location);
  virtual double SDFGradient(const Eigen::Vector4d &location, int dim, int stepSize);
  virtual void saveSDF(const std::string &filename = std::string("sdf_volume.vti"));
  virtual void loadSDF(const std::string &filename);   
  bool validGradient(const Eigen::Vector4d &location);
  virtual Vector6d EstimatePose(void); 
  virtual void FuseDepth(const cv::Mat &depth);
  virtual void Render(void);

  cv::Point2d To2D(const Eigen::Vector4d &location, double fx, double fy, double cx, double cy);
  Eigen::Matrix4d Twist(const Vector6d &xi);
  Eigen::Vector4d To3D(int row, int column, double depth, double fx, double fy, double cx, double cy);
  void saveTriangles(const std::string filename = std::string("triangles.obj"));
  
  void getDenoisedImage(cv::Mat &img); 
  Eigen::Matrix4d getCurrentTransformation(void);
  bool quit(void);

  SDFTracker();
  SDFTracker(SDF_Parameters &parameters);
  virtual ~SDFTracker();    
};

#endif