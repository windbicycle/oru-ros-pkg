/**
 * @author Robert Krug
 * @date  Tue Apr 10 2012
 *
 */


#ifndef pose_broadcaster_h___
#define pose_broadcaster_h___

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace ICR
{

/**
 * @brief Class aquiring object poses from a pose source and broadcasting the object's pose to tf 
 *
 */
  class PoseBroadcaster
  {
  public:

    //for some reason not defining this macro can crash the pose broadcaster on some systems,
    //although the class does not contain fixed sized Eigen members ...?
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

    PoseBroadcaster();
    ~PoseBroadcaster(){};

/**
 * @brief Sets the object for which the pose will be broadcasted
 *
 */
    void setObject(pcl::PointCloud<pcl::PointNormal> const & obj_cloud, std::string const & obj_name);
/**
 * @brief Broadcasts the object's pose.
 *
 */
    void broadcastPose();

    /** @brief True if pose broadcaster is configured to get the pose of an object from uc3m_objtracker
     */
    bool isPoseSourceUC3M() {return (strcmp(pose_source_.c_str(),"uc3m_objtrack") == 0);}

/**
 * @brief Not implemented yet! intended to get the pose from the UC3M objectracker by fitting
 * PoseBroadcaster::obj_cloud_ to the blob detected by the kinect
 * @arguments Takes a tf::Transform reference and writes the acquired pose to it
 */
    bool getPoseUC3MObjtrack();

  private:

    ros::NodeHandle nh_, nh_private_;
    boost::mutex lock_;
    tf::TransformBroadcaster tf_brc_;

    /** @brief The pose that is broadcasted in \ref broadcastPose 
     */
    tf::Transform object_pose_;

/**
 * @brief Point cloud describing the object geometry
 *
 */
    pcl::PointCloud<pcl::PointNormal> obj_cloud_;
    std::string obj_name_;
/**
 * @brief Describes the pose source. Can be either one of "gazebo" or "uc3m_objtrack"
 *
 */
    std::string pose_source_;
/**
 * @brief Frame in which the object's pose is expressed in
 *
 */
    std::string ref_frame_id_;
    std::string camera_frame_id_;
    ros::ServiceClient gazebo_get_ms_clt_;
    ros::ServiceClient get_tracked_obj_;

    tf::TransformListener tf_list_;
/**
 * @brief Gets the pose from a client call to Gazebo's /get_model_state service
 * @arguments Takes a tf::Transform reference and writes the acquired pose to it
 */
    bool getPoseGazebo();


    /////////////////
    //  CALLBACKS  //
    /////////////////

  };
}//end namespace

#endif
