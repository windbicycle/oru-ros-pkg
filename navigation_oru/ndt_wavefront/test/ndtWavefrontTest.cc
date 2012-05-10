#include <ndt_wavefront.h>
#include <spatial_index.h>
#include <robot_kinematics.h>
#include <pointcloud_utils.h>
#include <oc_tree.h>
#include <lazy_grid.h>

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

static int ctr = 0;

void ndtCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    //build the oct tree and save to disk, update counter
//    pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg,cloud);
   
    ROS_INFO ("Received %d data points with the following fields: %s", (int)(msg->width * msg->height),
	                pcl::getFieldsList (*msg).c_str ());


    lslgeneric::NDTMap<pcl::PointXYZ> nd(new lslgeneric::OctTree<pcl::PointXYZ>());
    nd.loadPointCloud(cloud);
   
    nd.computeNDTCells(); 
    ROS_INFO("Loaded point cloud");

    Eigen::Vector3d robotSize, inclinationConstraints;
    robotSize<<1,1,1;
    inclinationConstraints<<0.2,0.1,2*M_PI;
    lslgeneric::RobotKinematics kin(robotSize,inclinationConstraints);
    lslgeneric::NDTWavefrontPlanner<pcl::PointXYZ> planner(&nd,&kin);

    //hallway map
    lslgeneric::Pose3 start,goal;
    start.pos<<3,0,0;
    goal.pos<<12,-1,0.1;

    planner.planPath(start,goal);

    char fname[50];
    snprintf(fname,49,"ndt_map%05d.wrl",ctr);
    nd.writeToVRML(fname);

    snprintf(fname,49,"ndt_plan%05d.wrl",ctr);
    planner.writeToVRML(fname);

    ctr++;    


}

int
main (int argc, char** argv)
{

    ros::init(argc, argv, "ndt_builder");
    ros::NodeHandle n;
    ros::Subscriber chatter_sub = n.subscribe("points2_in", 10, ndtCallback);
    ros::spin();

    return (0);
}



