#ifndef icr_server_h___
#define icr_server_h___

#include "ros/ros.h"
#include "icr.h"

//icr_msgs
#include "icr_msgs/SetObject.h"
#include "icr_msgs/Grasp.h"
#include "icr_msgs/SetSphericalQuality.h"
#include "icr_msgs/SetPhalangeParameters.h"
#include "icr_msgs/SetActivePhalanges.h"
#include <icr_msgs/ContactRegions.h>
#include <icr_msgs/ContactRegion.h>
#include <icr_msgs/SaveIcr.h>
#include <icr_msgs/GetContactRegions.h>

//PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>

#include "std_srvs/Empty.h"
#include <boost/thread/mutex.hpp>
#include <vector>
#include <string>
#include <tf/tf.h>

/** \class IcrServer icr_server.h 
 * \brief Server that computes Independent Contact Regions
 *
 * The icrServer can be parametrized in several ways:
 * \li A model of an object can be loaded \ref loadWfrontObj.  
 * \li The number of fingers can be specified \ref addFingers.
 * \li Each finger can be parametrized \ref setFingerParameters.
 * Finally, the icr can be computed using \ref computeIcr. 
 *
 * Assumption is that IcrServer, when properly parametrized, returns
 * indices of vertices that consist ICRs. Communication with IcrServer
 * is done through services.
 *
 * Example sequence of calling of services that leads to icr computation:
 * ....
 * 
 */

namespace ICR 
{
#define MODE_CONTINUOUS 0
#define MODE_STEP_WISE  1

class IcrServer
{
 public:

  IcrServer();
  ~IcrServer(){};

  int getComputationMode();
  void computeSearchZones();
  void computeIcr();
  void publish();

 private:

  ros::NodeHandle nh_,nh_private_;
  XmlRpc::XmlRpcValue phalange_config_;
  TargetObjectPtr               obj_;
  GraspPtr                      pt_grasp_;
  SearchZonesPtr                sz_;
  IndependentContactRegionsPtr  icr_;
  
  bool obj_set_;
  bool pt_grasp_initialized_;
  bool gws_computed_;
  bool sz_computed_;
  bool icr_computed_;

  std::vector<std::string> active_phalanges_;
  int computation_mode_;
  double qs_;
  std::string obj_frame_id_;
  std::string icr_database_dir_;
  boost::mutex lock_;
  icr_msgs::ContactRegions::Ptr icr_msg_;
  tf::Transform palm_pose_;

  ros::ServiceServer get_icr_srv_;
  ros::ServiceServer compute_icr_srv_;
  ros::ServiceServer compute_sz_srv_;
  ros::ServiceServer set_obj_srv_;
  ros::ServiceServer toggle_mode_srv_;
  ros::ServiceServer set_qs_srv_;
  ros::ServiceServer set_active_phl_srv_;
  ros::ServiceServer set_phl_param_srv_;
  ros::ServiceServer save_icr_srv_;
  ros::Subscriber ct_pts_sub_;
  ros::Publisher icr_cloud_pub_;
  ros::Publisher icr_pub_;

  uint findObjectPointId(Eigen::Vector3d* point_in) const; 
  void getActivePhalangeParameters(FParamList & phl_param);
  unsigned int getPhalangeId(std::string const & name);
  void getFingerParameters(std::string const & name,FingerParameters & f_param);
  bool cpFromGraspMsg(icr_msgs::Grasp const & c_pts,const std::string & name,Eigen::Vector3d & contact_position,bool & touching)const;
  void initPtGrasp();  
  bool cloudFromContactRegion(unsigned int region_id,pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, std::vector<unsigned int> & point_ids);
  
  /////////////////
  //  CALLBACKS  //
  /////////////////

 bool getIcr(icr_msgs::GetContactRegions::Request  &req, icr_msgs::GetContactRegions::Response &res);
 bool triggerSearchZonesCmp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool triggerIcrCmp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool toggleMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
 bool saveIcr(icr_msgs::SaveIcr::Request &req, icr_msgs::SaveIcr::Response &res);
 bool setObject(icr_msgs::SetObject::Request  &req, icr_msgs::SetObject::Response &res);
 bool setSphericalQuality(icr_msgs::SetSphericalQuality::Request  &req, icr_msgs::SetSphericalQuality::Response &res);
 bool setActivePhalanges(icr_msgs::SetActivePhalanges::Request  &req, icr_msgs::SetActivePhalanges::Response &res);
 bool setPhalangeParameters(icr_msgs::SetPhalangeParameters::Request  &req, icr_msgs::SetPhalangeParameters::Response &res);
 void graspCallback(icr_msgs::Grasp const & grasp); 
};
}//end namespace



#endif
