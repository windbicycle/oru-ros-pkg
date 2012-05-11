#include <ros/ros.h>
#include "../include/model_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////
//----------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "model_server");

  ICR::ModelServer model_server;
  ROS_INFO("Model server ready");
 
  while(ros::ok())
    model_server.spin();

  return 0;
}
//----------------------------------------------------------------------------------------
