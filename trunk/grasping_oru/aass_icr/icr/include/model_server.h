/**
 * @author Robert Krug
 * @date   Wed Feb 22 2012
 *
 *
 *Implements a model server which spawns object models used fot the icr computation in gazebo and
 *pushes the according urdf files on the parameter server for RVIZ - visualization. Also the pose of
 *the object in the gazebo world frame is broadcasted via tf in order to track the object in RVIZ.
 *
 */


#ifndef model_server_h___
#define model_server_h___

#include "ros/ros.h"
#include "icr.h"
#include "pose_broadcaster.h"
/* #include "../srv_gen/cpp/include/icr/load_model.h" */
/* #include "../srv_gen/cpp/include/icr/SetObject.h" */
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <icr_msgs/LoadObject.h>
#include <icr_msgs/Object.h>
/* #include <tf/transform_broadcaster.h> */
#include <string>
#include <vector>
/* #include <iostream> */
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ICR
{

class ModelServer
{
 public:

  ModelServer();
  ~ModelServer(){};

  void spin();

 private:

  ros::NodeHandle nh_, nh_private_;
  boost::mutex lock_;
  /* tf::TransformBroadcaster tf_brc_; */
  
  std::string model_dir_;
  std::string obj_name_;
  std::string obj_frame_id_;
  std::string pose_source_;
  boost::shared_ptr<PoseBroadcaster> pose_brc_;
  icr_msgs::Object::Ptr obj_;
  double scale_;
  bool obj_loaded_;

  ros::ServiceServer load_obj_srv_;
  ros::ServiceClient gazebo_spawn_clt_;
  ros::ServiceClient gazebo_delete_clt_;
  ros::ServiceClient gazebo_pause_clt_;
  ros::ServiceClient gazebo_unpause_clt_;
  ros::ServiceClient gazebo_get_wp_clt_;
  std::vector<boost::shared_ptr<ros::ServiceClient> > set_obj_clts_;
 
  /* ros::Subscriber gazebo_modstat_sub_; */


bool loadURDF(std::string const & path,std::string & serialized_model);
bool gazeboSpawnModel(std::string const & serialized_model,geometry_msgs::Pose const & initial_pose);
 bool gazeboDeleteModel(std::string const & name);
 bool loadWavefrontObj(std::string const & path, pcl::PointCloud<pcl::PointNormal> & cloud, std::vector<std::vector<unsigned int> > & neighbors );

 

  /////////////////
  //  CALLBACKS  //
  /////////////////

 /** \brief Spawns the given urdf model as "icr_object" in gazebo, if such
   *  an object already exists it is deleted. Also, the urdf file is
   *  pushed onto the parameter server.
   */
  bool loadObject(icr_msgs::LoadObject::Request  &req, icr_msgs::LoadObject::Response &res);
  

};
}//end namespace

//--------------------------------------------------------------------------

#endif
