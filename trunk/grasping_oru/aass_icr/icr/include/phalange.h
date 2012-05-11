/**
 * @author Robert Krug
 * @date   Sat, Mar 10, 2012
 *
 *@brief Subscribes to gazebo_msgs/ContactsState outputted by the sensor_bumpers and computes the
 *average contact position.
 *
 */

#ifndef phalange_h___
#define phalange_h___

#include "ros/ros.h"
#include "icr_msgs/SetPose.h"
#include "icr_msgs/StampedContactPose.h"
#include <icr_msgs/ContactState.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <string>
#include "model_server.h"
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>


namespace ICR
{

/**
 *@brief This class represents a phalange of the hand. It subscribes to gazebo_msgs/ContactsState
 *outputted by the sensor_bumpers and computes the average contact position and normal direction for
 *a given target object. If the phalange is not in contact, the contact pose defaults to a
 *predefined pose. Also, the contact pose expressed in the target object frame is computed. Note,
 *that presently the published geometry_msgs/StampedPose message can create problems when
 *visualizing in Rviz. This is due to the fact that Rviz only buffers 5 geometry_msgs/StampedPose
 *msgs and proper tf transformations might not be available at the time the contact pose is
 *published. This can be fixed by increasing the buffer size in
 *rviz/src/rviz/default_plugin/pose_display.cpp and recompiling Rviz. Currently (in ROS Electric)
 *there is no way to increase Rviz's tf::MessageFilter queue at runtime. Also using a
 *topic_tools/throttle should fix the problem (did not test it though...)
 */
class Phalange
{
 public:

 
  Phalange(tf::Transform const & L_T_Cref,std::string const & phl_name,std::string const & phl_frame_id,std::string const & sensor_topic);
  ~Phalange();

  icr_msgs::StampedContactPose::ConstPtr getStampedContactPose();
  void setTargetObjFrameId(std::string const & obj_frame_id);  
  std::string getPhalangeName(); 
  std::string getPhalangeFrameId();
 
 private:

/**
 *@brief Using the default constructor is not allowed since the class needs to be initialized
 *properly. This could be changed by implementing corresponding setXYZ methods
 */
  Phalange();
  
  ros::NodeHandle nh_, nh_private_;
/**
 *@brief Publishes the average contact pose expressed in the phalange link frame if in contact, otherwise a predefined reference pose is published
 */
  ros::Publisher ct_pose_pub_;
/**
 *@brief Subscribes to a gazebo_msgs/ContactsState message 
 */
  ros::Subscriber contacts_subs_;
/**
 *@brief Allows to set the pose of the reference contact w.r.t the phalange link frame id (mainly intended for debugging)
 */
  ros::ServiceServer set_pose_srv_;
/**
 *@brief Pose of the reference contact expressed in the link frame id of the phalange
 */
  tf::Transform L_T_Cref_;
/**
 *@brief Current contact pose expressed in the target object frame. Used by the GraspServer class;
 */
  boost::shared_ptr<icr_msgs::StampedContactPose> C_T_O_;

  std::string phl_name_;
  std::string phl_frame_id_;

/**
 *@brief Defines the target object's frame_id 
 */
  std::string obj_frame_id_;

 boost::mutex lock_;
 tf::TransformListener tf_list_;

/**
 *@brief Message filter to ensure a transformation from the phalange link to the target object is available
 */
 tf::MessageFilter<icr_msgs::StampedContactPose>* tf_filter_;

  std::string tf_prefix_;

/**
 *@brief Used to generate the contact pose if Phalange::object_model_->geom_ and
 *Phalange::phalange_model_->geom_ are touching. The z-axis of the pose is given as argument. The x
 *& y axis are computed via projecting the x & y axis of the contact reference frame on the
 *nullspace of the new z axis (this is not very stable when axis of the link frame are (nearly)
 *aligned with the z axis of the contact pose since the norms of the projections of those axis will
 *be (nearly) 0)
 */
  tf::Quaternion projectPose(tf::Vector3 const & z);

  /////////////////
  //  CALLBACKS  //
  /////////////////

/**
 *@brief Callback listening to ContactsState msgs (which are always published, even if they are
 *empty). Computes the current contact pose.
 */
  void listenContacts(const icr_msgs::ContactState::ConstPtr& cts_st);
/**
 *@brief Sets the reference contact pose Phalange::L_T_Cref_ expressed in the phalange link frame
 */
  bool setPose(icr_msgs::SetPose::Request  &req, icr_msgs::SetPose::Response &res);
/**
 *@brief Callback registered with the filter; Transforms the current contact pose from the phalange
 *link frame to the object frame
 */
  void transformContactPose(const boost::shared_ptr<const icr_msgs::StampedContactPose>& C_T_L); 
};
}//end namespace
#endif
