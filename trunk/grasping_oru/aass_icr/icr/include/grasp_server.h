/**
 * @author Robert Krug
 * @date   Fri, Mar 9, 2012
 *
 */


#ifndef grasp_server_h___
#define grasp_server_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include "icr_msgs/SetObject.h"
#include <vector>
#include <string>
#include "phalange.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>

namespace ICR
{
/**
 *@brief Contains a list of Phalanges and publishes corresponding icr_msgs/ContactPoints messages 
 */
class GraspServer
{
 public:

  GraspServer();
  ~GraspServer();

/**
 *@brief Collects the current contact poses from the Phalanges and publishes icr_msgs/ContactPoints messages
 */
  void spin();

 private:

  ros::NodeHandle nh_, nh_private_;
  ros::ServiceServer set_obj_srv_;
  ros::Publisher grasp_pub_;  
  //ros::Publisher debug_pub_;//REMOVE  
  boost::mutex lock_;
  bool ref_set_;
   tf::TransformListener tf_list_;
  tf::StampedTransform palm_pose_;


/**
 *@brief Vector containing the phalanges of the hand - the order is important
 */
  std::vector<Phalange*> phalanges_;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  bool setObject(icr_msgs::SetObject::Request  &req, icr_msgs::SetObject::Response &res);
};
}//end namespace
#endif
