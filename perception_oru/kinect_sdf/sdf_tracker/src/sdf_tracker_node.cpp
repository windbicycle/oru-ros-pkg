#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#define EIGEN_NO_DEBUG

//#include <sdf_tracker.h>
#include <sdf_tracker.h>

class SDFTrackerNode
{
public:
  SDFTrackerNode(SDF_Parameters &parameters);
  virtual ~SDFTrackerNode() {
      if(myTracker_ != NULL) 
      {
	  delete myTracker_;
      }
  }
  void subscribeTopic(const std::string topic = std::string("default"));    
  void advertiseTopic(const std::string topic = std::string("default"));    
  void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
  int skip_frames_;
  SDFTracker* myTracker_;
  SDF_Parameters myParameters_;

  std::vector<ros::Time> timestamps_;
  ros::NodeHandle nh_;
  ros::NodeHandle n_;
  ros::Subscriber depth_subscriber_;
  ros::Subscriber color_subscriber_;
  ros::Publisher depth_publisher_;
  ros::Publisher color_publisher_;
  
  ros::Timer heartbeat_depth_;
  std::string camera_name_;

  bool depth_registered_;
  bool use_texture_;
  bool makeTris_;
  void publishDepthDenoisedImage(const ros::TimerEvent& event); 
};

SDFTrackerNode::SDFTrackerNode(SDF_Parameters &parameters)
{
  myParameters_ = parameters;
  nh_ = ros::NodeHandle("~");
  n_ = ros::NodeHandle();

  //node specific parameters
  nh_.param("depth_registered", depth_registered_, false);
  nh_.param<std::string>("c_name", camera_name_,"camera");
  nh_.param("OutputTriangles",makeTris_, false);
  nh_.param("UseTexture",use_texture_, false);

  //parameters used for the SDF tracker
  nh_.getParam("ImageWidth", myParameters_.image_width);
  nh_.getParam("ImageHeight", myParameters_.image_height); 
  nh_.getParam("InteractiveMode", myParameters_.interactive_mode);
  nh_.getParam("MaxWeight",myParameters_.Wmax);
  nh_.getParam("CellSize",myParameters_.resolution);
  nh_.getParam("GridSizeX",myParameters_.XSize);
  nh_.getParam("GridSizeY",myParameters_.YSize);
  nh_.getParam("GridSizeZ",myParameters_.ZSize);
  nh_.getParam("PositiveTruncationDistance",myParameters_.Dmax);
  nh_.getParam("NegativeTruncationDistance",myParameters_.Dmin);
  nh_.getParam("RobustStatisticCoefficient", myParameters_.robust_statistic_coefficient);
  nh_.getParam("Regularization", myParameters_.regularization);
  nh_.getParam("MinPoseChangeToFuseData", myParameters_.min_pose_change);
  nh_.getParam("ConvergenceCondition", myParameters_.min_parameter_update);
  nh_.getParam("MaximumRaycastSteps", myParameters_.raycast_steps);
  nh_.getParam("FocalLengthX", myParameters_.fx);
  nh_.getParam("FocalLengthY", myParameters_.fy);
  nh_.getParam("CenterPointX", myParameters_.cx);
  nh_.getParam("CenterPointY", myParameters_.cy);

  myTracker_ = new SDFTracker(myParameters_);

  skip_frames_ = 0;
}

void
SDFTrackerNode::subscribeTopic(const std::string topic)
{
  
  std::string subscribe_topic_depth = topic;
  std::string subscribe_topic_color = topic;

  if(depth_registered_)
  {
    if(topic=="default")
    {
      subscribe_topic_depth = camera_name_+"/depth_registered/image";
      subscribe_topic_color = camera_name_+"/rgb/image_color";
    }

    depth_subscriber_ = n_.subscribe(subscribe_topic_depth, 1, &SDFTrackerNode::depthCallback, this);
    if(use_texture_) color_subscriber_ = n_.subscribe(subscribe_topic_color, 1, &SDFTrackerNode::rgbCallback, this);
  }
  else
  {
    if(topic=="default")
    { 
      subscribe_topic_depth = camera_name_+"/depth/image";
      subscribe_topic_color = camera_name_+"/rgb/image_color";
    }
      depth_subscriber_ = n_.subscribe(subscribe_topic_depth, 1, &SDFTrackerNode::depthCallback, this);
      if(use_texture_) color_subscriber_ = n_.subscribe(subscribe_topic_color, 1, &SDFTrackerNode::rgbCallback, this);
  }
}

void
SDFTrackerNode::advertiseTopic(const std::string topic)
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

  heartbeat_depth_ = nh_.createTimer(ros::Duration(1.0), &SDFTrackerNode::publishDepthDenoisedImage, this);

}
void SDFTrackerNode::publishDepthDenoisedImage(const ros::TimerEvent& event) 
{    
  if(depth_publisher_.getNumSubscribers()>0)
  {
    cv_bridge::CvImagePtr image_out (new cv_bridge::CvImage());
    image_out->header.stamp = ros::Time::now(); 
    image_out->encoding = "32FC1";      
    myTracker_->getDenoisedImage(image_out->image);
    depth_publisher_.publish(image_out->toImageMsg());     
  } 
  return;
};

void SDFTrackerNode::rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(msg, "8UC3");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform rgb image.");
    return;
  
    /* do something colorful"*/
  }
}

void SDFTrackerNode::depthCallback(const sensor_msgs::Image::ConstPtr& msg)
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
  
  if(skip_frames_ < 3){++skip_frames_; return;}

  if(!myTracker_->quit_)
  {
    myTracker_->FuseDepth(bridge->image);
    timestamps_.push_back(ros::Time::now());
  }
  else 
  {
    if(makeTris_) myTracker_->saveTriangles();
    ros::shutdown();
  }
}



int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "sdf_tracker_node");
    
    SDF_Parameters param;
    //SDFTracker MyTracker;
    SDFTrackerNode MyTrackerNode(param);
    MyTrackerNode.subscribeTopic();
    MyTrackerNode.advertiseTopic();

    ros::spin();
    


    return 0;
}