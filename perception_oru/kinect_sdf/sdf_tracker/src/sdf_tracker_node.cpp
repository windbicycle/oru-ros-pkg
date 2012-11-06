#include <ros/ros.h>
#include <sdf_tracker.h>

int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "sdf_tracker_node");
  SDFTracker MyTracker;
  ros::spin();
  return 0;
}