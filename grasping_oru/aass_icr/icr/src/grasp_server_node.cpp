/**
 * @author Robert Krug
 * @date   Fri, Mar 9, 2012
 *
 */

#include <ros/ros.h>
#include "../include/grasp_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_server");

  double spin_frequency=100;
  std::string searched_param;
  ros::NodeHandle nh_private_("~");

  if(nh_private_.searchParam("spin_frequency",searched_param))
    nh_private_.getParam(searched_param, spin_frequency);

  ICR::GraspServer grasp_server;
  ROS_INFO("Grasp server ready");
  
  ros::Rate r(spin_frequency); 
  while(ros::ok())
    {
    grasp_server.spin();
    r.sleep();
    }
  return 0;
}
//----------------------------------------------------------------------------------------
