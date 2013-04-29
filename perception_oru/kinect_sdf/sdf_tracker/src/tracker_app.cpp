#include <sdf_tracker.h>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{  
  //Parameters for an SDFtracker object 
  SDF_Parameters myParameters;

  //Pose Offset as a transformation matrix
  Eigen::Matrix4d initialTransformation = 
  Eigen::MatrixXd::Identity(4,4);

  //define translation offsets in x y z
  initialTransformation(0,3) = 0.0;  //x 
  initialTransformation(1,3) = 0.0;  //y
  initialTransformation(2,3) = -0.8; //z

  myParameters.pose_offset = initialTransformation;
  myParameters.interactive_mode = true;
  myParameters.resolution = 0.01;
  myParameters.XSize = 200;
  myParameters.YSize = 200;
  myParameters.ZSize = 200;

  //create the tracker object
  SDFTracker myTracker(myParameters);
  
  cv::VideoCapture capture( CV_CAP_OPENNI_ASUS );
  cv::Mat depthMap, depth;

  while(!myTracker.Quit())
  {
    capture >> depthMap;
    depthMap.convertTo(depth, CV_32F, 0.001);
    myTracker.FuseDepth(depth);  

  }  
    myTracker.SaveSDF();
    return 0;
}