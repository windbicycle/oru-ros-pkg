#include <NDTMap.hh>
#include <OctTree.hh>
#include <AdaptiveOctTree.hh>
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


    lslgeneric::NDTMap nd(new lslgeneric::AdaptiveOctTree());
    nd.loadPointCloud(cloud);
   
    nd.computeNDTCells(); 
    ROS_INFO("Loaded point cloud");
    char fname[50];
    snprintf(fname,49,"ndt_map%05d.wrl",ctr);
    nd.writeToVRML(fname);
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



