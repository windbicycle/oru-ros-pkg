//#include <ndt_fuser.h>
#include <ndt_fuser_hmt.h>
#include <ros/ros.h>

#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include <cstdio>
#include <Eigen/Eigen>
#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#ifndef SYNC_FRAMES
#define SYNC_FRAMES 20
#endif
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
  * \author Todor Stoyanov
  * 
  */
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> LaserOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseStamped> LaserPoseSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> PointsOdomSync;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> PointsPoseSync;

class NDTFuserNode {

    protected:
	// Our NodeHandle
	ros::NodeHandle nh_;

	message_filters::Subscriber<sensor_msgs::PointCloud2> *points2_sub_;
	message_filters::Subscriber<sensor_msgs::LaserScan> *laser_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;

	// Components for publishing
	tf::TransformBroadcaster tf_;
	ros::Publisher output_pub_;
	Eigen::Affine3d pose_, T, sensor_pose_;

	unsigned int nb_added_clouds_;
	double varz;
	
	boost::mutex m, message_m;
	lslgeneric::NDTFuserHMT<pcl::PointXYZ> *fuser;
	std::string points_topic, laser_topic, map_dir, map_name, odometry_topic;
	double size_x, size_y, size_z, resolution, sensor_range, min_laser_range_;
	bool visualize, match2D, matchLaser, beHMT, useOdometry;

	double pose_init_x,pose_init_y,pose_init_z,
	       pose_init_r,pose_init_p,pose_init_t;
	double sensor_pose_x,sensor_pose_y,sensor_pose_z,
	       sensor_pose_r,sensor_pose_p,sensor_pose_t;
	laser_geometry::LaserProjection projector_;

	message_filters::Synchronizer< LaserOdomSync > *sync_lo_;
	message_filters::Synchronizer< LaserPoseSync > *sync_lp_;
	
	message_filters::Synchronizer< PointsOdomSync > *sync_po_;
	message_filters::Synchronizer< PointsPoseSync > *sync_pp_;
	ros::ServiceServer save_map_;

	Eigen::Affine3d last_odom, this_odom;
    public:
	// Constructor
	NDTFuserNode(ros::NodeHandle param_nh) : nb_added_clouds_(0)
	{
	    ///topic to wait for point clouds, if available
	    param_nh.param<std::string>("points_topic",points_topic,"points");
	    ///topic to wait for laser scan messages, if available
	    param_nh.param<std::string>("laser_topic",laser_topic,"laser_scan");

	    ///if using the HMT fuser, NDT maps are saved in this directory. 
	    ///a word of warning: if you run multiple times with the same directory, 
	    ///the old maps are loaded automatically
	    param_nh.param<std::string>("map_directory",map_dir,"map");
	    param_nh.param<std::string>("map_name_prefix",map_name,"");
	    
	    ///initial pose of the vehicle with respect to the map
	    param_nh.param("pose_init_x",pose_init_x,0.);
	    param_nh.param("pose_init_y",pose_init_y,0.);
	    param_nh.param("pose_init_z",pose_init_z,0.);
	    param_nh.param("pose_init_r",pose_init_r,0.);
	    param_nh.param("pose_init_p",pose_init_p,0.);
	    param_nh.param("pose_init_t",pose_init_t,0.);
	    
	    ///pose of the sensor with respect to the vehicle odometry frame
	    param_nh.param("sensor_pose_x",sensor_pose_x,0.);
	    param_nh.param("sensor_pose_y",sensor_pose_y,0.);
	    param_nh.param("sensor_pose_z",sensor_pose_z,0.);
	    param_nh.param("sensor_pose_r",sensor_pose_r,0.);
	    param_nh.param("sensor_pose_p",sensor_pose_p,0.);
	    param_nh.param("sensor_pose_t",sensor_pose_t,0.);
	    
	    ///size of the map in x/y/z. if using HMT, this is the size of the central tile
	    param_nh.param("size_x_meters",size_x,10.);
	    param_nh.param("size_y_meters",size_y,10.);
	    param_nh.param("size_z_meters",size_z,10.);
	    
	    ///range to cutoff sensor measurements
	    param_nh.param("sensor_range",sensor_range,3.);
	    ///range to cutoff sensor measurements
	    param_nh.param("min_laser_range",min_laser_range_,0.1);
	    
	    //map resolution
	    param_nh.param("resolution",resolution,0.10);
	    param_nh.param("laser_variance_z",varz,resolution/4);

	    ///visualize in a local window
	    param_nh.param("visualize",visualize,true);
	    ///only mathc with 3dof
	    param_nh.param("match2D",match2D,false);
	    ///use HMT grid or simple grid.
	    param_nh.param("beHMT",beHMT,false);
	    ///use standard odometry messages for initial guess 
	    param_nh.param("useOdometry",useOdometry,false);
	    ///topic to wait for laser scan messages, if available
	    param_nh.param<std::string>("odometry_topic",odometry_topic,"odometry");

	    ///enable for LaserScan message input
	    param_nh.param("matchLaser",matchLaser,false);
	    
	    pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
		Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
	    
	    sensor_pose_ =  Eigen::Translation<double,3>(sensor_pose_x,sensor_pose_y,sensor_pose_z)*
		Eigen::AngleAxis<double>(sensor_pose_r,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(sensor_pose_p,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(sensor_pose_t,Eigen::Vector3d::UnitZ()) ;

	    if(matchLaser) match2D=true;
	    fuser = new lslgeneric::NDTFuserHMT<pcl::PointXYZ>(resolution,size_x,size_y,size_z,
		    sensor_range, visualize,match2D, false, false, 30, map_name, beHMT, map_dir, true);

	    fuser->setSensorPose(sensor_pose_);

	    if(!matchLaser) {
		points2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,points_topic,1);
		if(useOdometry) {
		    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
		    sync_po_ = new message_filters::Synchronizer< PointsOdomSync >(PointsOdomSync(SYNC_FRAMES), *points2_sub_, *odom_sub_);
		    sync_po_->registerCallback(boost::bind(&NDTFuserNode::points2OdomCallback, this, _1, _2));
		} else {
		    points2_sub_->registerCallback(boost::bind( &NDTFuserNode::points2Callback, this, _1));
		}
	    } else {
		laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_,laser_topic,2);
		if(useOdometry) {
		    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,odometry_topic,10);
		    sync_lo_ = new message_filters::Synchronizer< LaserOdomSync >(LaserOdomSync(SYNC_FRAMES), *laser_sub_, *odom_sub_);
		    sync_lo_->registerCallback(boost::bind(&NDTFuserNode::laserOdomCallback, this, _1, _2));

		} else {
		    laser_sub_->registerCallback(boost::bind( &NDTFuserNode::laserCallback, this, _1));
		}
	    }
	    save_map_ = param_nh.advertiseService("save_map", &NDTFuserNode::save_map_callback, this);

	}

	~NDTFuserNode()
	{
	    delete fuser;
	}

	void processFrame(pcl::PointCloud<pcl::PointXYZ> &cloud, 
			  Eigen::Affine3d Tmotion) {
	    
	    m.lock();
	    if (nb_added_clouds_  == 0)
	    {
		nb_added_clouds_++;
		fuser->initialize(pose_,cloud);
	    } else {
		nb_added_clouds_++;
		pose_ = fuser->update(Tmotion,cloud);
	    }
	    m.unlock();

	    tf::Transform transform;
#if ROS_VERSION_MINIMUM(1,9,0)
//groovy
	    tf::transformEigenToTF(pose_, transform);
#else
//fuerte
	    tf::TransformEigenToTF(pose_, transform);
#endif
	    tf_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "test"));

	}
	
	bool save_map_callback(std_srvs::Empty::Request  &req,
		std_srvs::Empty::Response &res ) {

	    bool ret = false;
	    ROS_INFO("Saving current map to map directory %s", map_dir.c_str());
	    m.lock();
	    ret = fuser->saveMap(); 
	    m.unlock();

	    return ret;
	}


	// Callback
	void points2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg_in)
	{
	    pcl::PointCloud<pcl::PointXYZ> cloud;
	    message_m.lock();
	    pcl::fromROSMsg (*msg_in, cloud);
	    message_m.unlock();
	    T.setIdentity();
	    this->processFrame(cloud,T);

	};
	
	// Callback
	void points2OdomCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_in,
		  const nav_msgs::Odometry::ConstPtr& odo_in)
	{
	    Eigen::Quaterniond qd;
	    Eigen::Affine3d Tm;
	    pcl::PointCloud<pcl::PointXYZ> cloud;
	    
	    message_m.lock();
	    qd.x() = odo_in->pose.pose.orientation.x;
	    qd.y() = odo_in->pose.pose.orientation.y;
	    qd.z() = odo_in->pose.pose.orientation.z;
	    qd.w() = odo_in->pose.pose.orientation.w;
	    
	    this_odom = Eigen::Translation3d (odo_in->pose.pose.position.x,
		    odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;
	    if (nb_added_clouds_  == 0)
	    {
		Tm.setIdentity();
	    } else {
		Tm = last_odom.inverse()*this_odom;
		//std::cout<<"delta from last update: "<<Tm.translation().transpose()<<" "<<Tm.rotation().eulerAngles(0,1,2)[2] << std::endl;
		//if(Tm.translation().norm()<0.2 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(5*M_PI/180.0)) {
		//    message_m.unlock();
		//    return;
		//}
	    }
	    last_odom = this_odom;

	    pcl::fromROSMsg (*msg_in, cloud);
	    message_m.unlock();
	    
	    this->processFrame(cloud,Tm);

	};
	
	// Callback
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
	{
	    // Add to a queue
	    sensor_msgs::PointCloud2 cloud;
	    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_unfiltered, pcl_cloud;
	    message_m.lock();
	    projector_.projectLaser(*msg_in, cloud);
	    message_m.unlock();
	    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

	    pcl::PointXYZ pt;
	    //add some variance on z
	    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
		pt = pcl_cloud_unfiltered.points[i];
		if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
		    pt.z += varz*((double)rand())/(double)INT_MAX;
		    pcl_cloud.points.push_back(pt);
		}
	    }
	    ROS_INFO("Got laser points");

	    T.setIdentity();
	    this->processFrame(pcl_cloud,T);

	};
	
	// Callback
	void laserOdomCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in,
		  const nav_msgs::Odometry::ConstPtr& odo_in)
	{
	    Eigen::Quaterniond qd;
	    sensor_msgs::PointCloud2 cloud;
	    pcl::PointCloud<pcl::PointXYZ> pcl_cloud, pcl_cloud_unfiltered;
	    Eigen::Affine3d Tm;

	    message_m.lock();
	    qd.x() = odo_in->pose.pose.orientation.x;
	    qd.y() = odo_in->pose.pose.orientation.y;
	    qd.z() = odo_in->pose.pose.orientation.z;
	    qd.w() = odo_in->pose.pose.orientation.w;
	    
	    this_odom = Eigen::Translation3d (odo_in->pose.pose.position.x,
		    odo_in->pose.pose.position.y,odo_in->pose.pose.position.z) * qd;

	    //std::cout<<"AT: "<<this_odom.translation().transpose()<<" "<<this_odom.rotation().eulerAngles(0,1,2)[2] << std::endl;
	    
	    if (nb_added_clouds_  == 0)
	    {
		Tm.setIdentity();
	    } else {
		Tm = last_odom.inverse()*this_odom;
		//std::cout<<"delta from last update: "<<Tm.translation().transpose()<<" "<<Tm.rotation().eulerAngles(0,1,2)[2] << std::endl;
		//if(Tm.translation().norm()<0.2 && fabs(Tm.rotation().eulerAngles(0,1,2)[2])<(5*M_PI/180.0)) {
		//    message_m.unlock();
		//    return;
		//}
	    }
	    last_odom = this_odom;

	    projector_.projectLaser(*msg_in, cloud);
	    message_m.unlock();
	    
	    pcl::fromROSMsg (cloud, pcl_cloud_unfiltered);

	    pcl::PointXYZ pt;
	    //add some variance on z
	    for(int i=0; i<pcl_cloud_unfiltered.points.size(); i++) {
		pt = pcl_cloud_unfiltered.points[i];
		if(sqrt(pt.x*pt.x+pt.y*pt.y) > min_laser_range_) {
		    pt.z += varz*((double)rand())/(double)INT_MAX;
		    pcl_cloud.points.push_back(pt);
		}
	    }
	    ROS_INFO("Got laser and odometry!");

	    this->processFrame(pcl_cloud,Tm);

	};
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_fuser_node");

    ros::NodeHandle param("~");
    NDTFuserNode t(param);
    ros::spin();

    return 0;
}

