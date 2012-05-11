/** \file icr_server_node.cpp 
 * \author Robert Krug 
 * \brief Just an implementation of icr_server_node. 
 * Read \ref IcrServer for more details.
 */
#include <ros/ros.h>
#include "../include/icr_server.h"


  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////


  //---------------------------------------------------------------------
  int main(int argc, char **argv)
  {
    ros::init(argc, argv, "icr_server");

    ICR::IcrServer icr_server;
    ROS_INFO("ICR server ready");
    while(ros::ok()) 
      {
	switch (icr_server.getComputationMode()) 
	  {
	  case MODE_CONTINUOUS : 
	    icr_server.computeSearchZones();
	    icr_server.computeIcr();
	    icr_server.publish();
	    break;

	  case MODE_STEP_WISE : 
	    icr_server.publish();
	    break;

	  default : 
	    ROS_ERROR("%d is an invalid computation mode - ICR computation not possible",icr_server.getComputationMode());
	  }

	ros::spinOnce();
      }

    return 0;
  }
  //---------------------------------------------------------------------

