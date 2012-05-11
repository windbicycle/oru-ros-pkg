/**
 * @author Robert Krug
 * @date  Tue Apr 10 2012
 *
 */

#include "../include/pose_broadcaster.h"
#include <tf/tf.h>
#include <gazebo_msgs/GetModelState.h>
#include <icr_msgs/GetObjectPose.h>

namespace ICR
{
  PoseBroadcaster::PoseBroadcaster() : nh_private_("~"),ref_frame_id_("/fixed"),camera_frame_id_("/camera_rgb_frame")
  {
    std::string param;
    std::string gazebo_prefix;
    std::string uc3m_objtrack_prefix;

    nh_private_.searchParam("pose_source", param);
    nh_private_.getParam(param, pose_source_);

    if(nh_private_.searchParam("reference_frame_id", param)) //try to read the reference frame from the parameter server
      nh_private_.getParam(param, ref_frame_id_);
    else
      ROS_WARN("No reference frame id found on the parameter server - using %s",ref_frame_id_.c_str());

  if(nh_private_.searchParam("camera_frame_id", param)) //try to read the camera frame from the parameter server
      nh_private_.getParam(param, camera_frame_id_);
    else
      ROS_WARN("No camera frame id found on the parameter server - using %s",camera_frame_id_.c_str());

    nh_private_.searchParam("gazebo_prefix", param);
    nh_private_.getParam(param, gazebo_prefix);

    nh_private_.searchParam("uc3m_objtrack_prefix", param);
    nh_private_.getParam(param, uc3m_objtrack_prefix);

    gazebo_get_ms_clt_ = nh_.serviceClient<gazebo_msgs::GetModelState>(gazebo_prefix + "/get_model_state");
    get_tracked_obj_ = nh_.serviceClient<icr_msgs::GetObjectPose>(uc3m_objtrack_prefix + "/get_object_pose");

    //If Gazebo is the intended pose source, wait for Gazebo services to be available 
    if(!strcmp(pose_source_.c_str(),"gazebo"))
      gazebo_get_ms_clt_.waitForExistence();  
    else if(!strcmp(pose_source_.c_str(),"uc3m_objtrack"))
       get_tracked_obj_.waitForExistence();  

  }
  //-----------------------------------------------------------------------------------
  void PoseBroadcaster::setObject(pcl::PointCloud<pcl::PointNormal> const & obj_cloud, std::string const & obj_name)
  {
    lock_.lock();
    obj_cloud_=obj_cloud;
    obj_name_=obj_name;
    lock_.unlock();
  }
  //-----------------------------------------------------------------------------------
  void PoseBroadcaster::broadcastPose()
  {
    // kca, 29 Apr 2012, pose is a member of PoseBroadcaster
    //    tf::Transform pose;
    lock_.lock();

    if(!strcmp(pose_source_.c_str(),"gazebo")) //Gazebo is defined as pose source
      {
	if(!getPoseGazebo())
	  {
	    lock_.unlock();
	    return;
	  }
      }
    //kca, 29 April 2012, : this is commented since now in uc3m_objtrack mode the pose of the object is obtained only once while loading the new model of an object.
    //
    else if(!strcmp(pose_source_.c_str(),"uc3m_objtrack")) //The UC3M objecttracker is defined as pose source
      {
	// DO NOTHING NOW, pose is calculated only once when loading object

    	// if(!getPoseUC3MObjtrack(pose))
    	//   {
    	//     lock_.unlock();
    	//     return;
    	//   }
      }
    else
      {
	ROS_ERROR("%s is an invalid pose source. Implemented are 'gazebo' and 'uc3m_objtrack'. Cannot broadcast object pose ... ",pose_source_.c_str());
	lock_.unlock();
	return;
      }
    
    //Broadcast the pose to tf
    tf_brc_.sendTransform(tf::StampedTransform(object_pose_, ros::Time::now(),ref_frame_id_, obj_cloud_.header.frame_id));

    lock_.unlock();
  }
  //-----------------------------------------------------------------------------------
  bool PoseBroadcaster::getPoseGazebo()
  {
    gazebo_msgs::GetModelState get_ms;
    get_ms.request.model_name=obj_name_;

    if(!gazebo_get_ms_clt_.call(get_ms))
      {
	ROS_ERROR("Could not get state of object %s from Gazebo. Cannot determine pose...",obj_name_.c_str());
	return false;
      }

    object_pose_.setOrigin(tf::Vector3(get_ms.response.pose.position.x,get_ms.response.pose.position.y,get_ms.response.pose.position.z));
    object_pose_.setRotation(tf::Quaternion(get_ms.response.pose.orientation.x,get_ms.response.pose.orientation.y,get_ms.response.pose.orientation.z,get_ms.response.pose.orientation.w));
    return true;
  }
  //-----------------------------------------------------------------------------------
  bool PoseBroadcaster::getPoseUC3MObjtrack()
  {
    //TODO: at the beginning, fit the obj_cloud_ to the blob and compute a static offset pose, then call the pose service from the UC3M objecttracker to get the current pose
    icr_msgs::GetObjectPose obj_pose;
    if (!get_tracked_obj_.call(obj_pose) ) {
      return false;
    } 
    geometry_msgs::PoseStamped detected_pose;
    //   tf::Transform transform; // transform that is broadcasted
    detected_pose.pose = obj_pose.response.object.pose;
    object_pose_.setOrigin(tf::Vector3(detected_pose.pose.position.x,
				       detected_pose.pose.position.y,
				       detected_pose.pose.position.z));
    object_pose_.setRotation(tf::Quaternion(detected_pose.pose.orientation.x,
					    detected_pose.pose.orientation.y,
					    detected_pose.pose.orientation.z,
					    detected_pose.pose.orientation.w ));


    tf::StampedTransform R_T_C;//camera pose expressed in the reference frame
    R_T_C.stamp_=obj_pose.response.object.header.stamp;
    try
      {
	tf_list_.waitForTransform(ref_frame_id_, camera_frame_id_, R_T_C.stamp_, ros::Duration(1.0));
	tf_list_.lookupTransform(ref_frame_id_, camera_frame_id_,R_T_C.stamp_,R_T_C);
      }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }

    object_pose_=R_T_C*object_pose_; //object pose R_T_O expressed in the reference frame

    return true;
  }
  //-----------------------------------------------------------------------------------


}//end namespace
