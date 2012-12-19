#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
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

typedef Eigen::Matrix<double,6,1> Vector6d; 


class SDFTracker
{
  protected:
  // variables
  std::vector<ros::Time> timestamps_;
  std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > transformations_;
  Eigen::Matrix4d Transformation_;
  Vector6d Pose_;
  Vector6d cumulative_pose_;
  cv::Mat *depthImage_;
  cv::Mat *depthImage_denoised_;

  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ros::Subscriber depth_subscriber_;
  ros::Publisher depth_publisher_;
  ros::Timer heartbeat_depth_;
  
  boost::mutex depthDenoised_mutex_;
  std::string camera_name_;
  bool depth_registered_;
  bool quit_;
  bool first_frame_;  
  bool makeTris_;
  bool** validityMask_;
  bool interactive_mode_;
  float*** myGrid_; 
  float*** weightArray_;    
  int XSize_;
  int YSize_;
  int ZSize_;
  int raycast_steps_;
  int image_height_;
  int image_width_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double Wmax_;
  double resolution_;
  double Dmax_; 
  double Dmin_;
  double XOffset_;
  double YOffset_;
  double ZOffset_;
  double robust_statistic_coefficient_;
  double regularization_;
  double min_parameter_update_;
  double min_pose_change_;

  std::ofstream triangle_stream_;
  // functions 
  Eigen::Vector3d VertexInterp(double iso, Eigen::Vector4d &p1d, Eigen::Vector4d &p2d,double valp1, double valp2);
  void marchingTetrahedrons(Eigen::Vector4d &Origin, int tetrahedron);
  void publishDepthDenoisedImage(const ros::TimerEvent& event);

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void subscribeTopic(const std::string topic = std::string("default"));    
  virtual void advertiseTopic(const std::string topic = std::string("default"));    
  virtual double SDF(const Eigen::Vector4d &location);
  virtual double SDFGradient(const Eigen::Vector4d &location, int dim, int stepSize);
  bool validGradient(const Eigen::Vector4d &location);
  virtual Vector6d EstimatePose(void); 
  cv::Point2d To2D(const Eigen::Vector4d &location, double fx, double fy, double cx, double cy);
  Eigen::Matrix4d Twist(const Vector6d &xi);
  Eigen::Vector4d To3D(int row, int column, double depth, double fx, double fy, double cx, double cy);
  virtual void FuseDepth(const sensor_msgs::Image::ConstPtr& msg);   
  virtual void Render(void);
  SDFTracker();
  ~SDFTracker();    
};

#endif 
/*SDF_REGISTRATION*/