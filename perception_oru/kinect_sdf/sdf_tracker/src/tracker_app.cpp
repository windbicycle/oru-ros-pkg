#include <sdf_tracker.h>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>


class SDFTracker_app : public openni::VideoStream::NewFrameListener
{
  public:
    SDFTracker_app(SDF_Parameters &parameters);
    ~SDFTracker_app();
    
    void onNewFrame(openni::VideoStream& stream);
  
  private:
    int skip_frame_;
    SDFTracker* myTracker_;
    openni::Device* rgbd_camera_;
    openni::VideoStream* depth_stream_;
    openni::VideoStream* color_stream_;
    openni::VideoFrameRef depth_FrameRef_;
    openni::VideoFrameRef color_FrameRef_;
};


int main(int argc, char* argv[])
{  
  //Parameters for an SDFtracker object 
  SDF_Parameters myParameters;

  myParameters.interactive_mode = true;
  myParameters.resolution = 0.025;
  myParameters.Dmax = 0.2;
  myParameters.Dmin = -0.2;
  myParameters.XSize = 400;
  myParameters.YSize = 150;
  myParameters.ZSize = 400;
  
  //QVGA for slow computers 
  myParameters.image_width = 320;
  myParameters.image_height = 240;
  

  //Pose Offset as a transformation matrix
  Eigen::Matrix4d initialTransformation = 
  Eigen::MatrixXd::Identity(4,4);

  //define translation offsets in x y z
  initialTransformation(0,3) = 0.0;  //x 
  initialTransformation(1,3) = 0.0;  //y
  //initialTransformation(2,3) = -0.5*myParameters.ZSize*myParameters.resolution;//z
  //myParameters.pose_offset = initialTransformation;

  //create the tracker object
  SDFTracker_app myTracker(myParameters);
    
  do{ sleep(1); }while(1);
    
    return 0;
}


//SDFTracker_app class implementation
SDFTracker_app::SDFTracker_app(SDF_Parameters &parameters)
{
  skip_frame_ = 0;
  openni::Status rc = openni::OpenNI::initialize();

  if (rc != openni::STATUS_OK)
  {
    printf("Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    exit(1);
  }

  std::cout << "OpenNI initialized" << std::endl;
  
  rgbd_camera_ = new openni::Device();
  rc = rgbd_camera_->open(openni::ANY_DEVICE);
  
  if (rc != openni::STATUS_OK)
  {
    printf("Couldn't open device\n%s\n", openni::OpenNI::getExtendedError());
    exit(2);
  }
 
  std::cout << "Opened device" << std::endl;

  depth_stream_ = new openni::VideoStream;
  if (rgbd_camera_->getSensorInfo(openni::SENSOR_DEPTH) != NULL)
  {
    rc = depth_stream_->create(*rgbd_camera_, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK)
    {
      printf("Couldn't create depth stream\n%s\n", openni::OpenNI::getExtendedError());
      exit(3);
    }
  }
  std::cout << "Created depth stream" << std::endl;

  openni::VideoMode vm;

  vm = depth_stream_->getVideoMode();
  vm.setResolution(parameters.image_width, parameters.image_height);
  depth_stream_->setVideoMode(vm);

  //std::cout << "Resolution set to VGA" << std::endl;

  depth_stream_->addNewFrameListener(this);

  std::cout << "Added listener" << std::endl;

  rc = depth_stream_->start();
  if (rc != openni::STATUS_OK)
  {
    printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
    exit(4);
  }

  std::cout << "Started depth stream" << std::endl;

  color_stream_ = new openni::VideoStream;
  if (rgbd_camera_->getSensorInfo(openni::SENSOR_COLOR) != NULL)
  {
    rc = color_stream_->create(*rgbd_camera_, openni::SENSOR_COLOR);
    if (rc != openni::STATUS_OK)
    {
      printf("Couldn't create color stream\n%s\n", openni::OpenNI::getExtendedError());
      exit(5);
    }
  }
  std::cout << "Created color stream" << std::endl;

  vm = color_stream_->getVideoMode();
  vm.setResolution(parameters.image_width,parameters.image_height);
  color_stream_->setVideoMode(vm);

  //std::cout << "Resolution set to VGA" << std::endl;

  rc = color_stream_->start();
  if (rc != openni::STATUS_OK)
  {
    printf("Couldn't start the color stream\n%s\n", openni::OpenNI::getExtendedError());
    exit(6);
  }
  std::cout << "Started color stream" << std::endl;

  if(rgbd_camera_->getImageRegistrationMode() != openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)
  rgbd_camera_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  
  std::cout << "Finished setting up camera " << std::endl;

  myTracker_ = new SDFTracker(parameters);
}

SDFTracker_app::~SDFTracker_app()
{
  depth_stream_->stop();
  depth_stream_->destroy();

  color_stream_->stop();
  color_stream_->destroy();
  rgbd_camera_->close();

  delete color_stream_;
  delete depth_stream_;
  delete rgbd_camera_;
  delete myTracker_;
  openni::OpenNI::shutdown();
}

void SDFTracker_app::onNewFrame(openni::VideoStream& stream)
{
  if(skip_frame_ < 5) 
  {
    ++skip_frame_; 
    return;
  }

  stream.readFrame(&depth_FrameRef_);

  // openni::RGB888Pixel* pColor = (openni::RGB888Pixel*)color_FrameRef_.getData();
  openni::DepthPixel* pDepth = (openni::DepthPixel*)depth_FrameRef_.getData();

  cv::Mat depthmap(depth_FrameRef_.getHeight(), depth_FrameRef_.getWidth(), CV_32FC1, cv::Scalar(0.0));
  
  for(int i=0; i<depth_FrameRef_.getHeight(); ++i)
  #pragma omp parallel
    for(int j=0; j<depth_FrameRef_.getWidth(); ++j) 
      depthmap.at<float>(i,depth_FrameRef_.getWidth()-(j+1)) = (pDepth[i*depth_FrameRef_.getWidth() + j] == 0) ? std::numeric_limits<float>::quiet_NaN() : float(pDepth[i*depth_FrameRef_.getWidth() + j])/1000;

  myTracker_->FuseDepth(depthmap);  

  if(myTracker_->Quit()) 
  {
    printf("Dumping volume and exiting.\n");
    myTracker_->SaveSDF();
    exit(0);
  }
}
