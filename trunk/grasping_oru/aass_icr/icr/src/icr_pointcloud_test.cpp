#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "icr/compute_icr.h"
#include <string>

using std::string;

ros::Publisher pub;

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_icr_pcl_test");
  ros::NodeHandle nh;
  
  //let us assume that center points come from somewhere
  uint16_t myints[] =  {1838, 4526, 4362, 1083, 793};
  uint16_t myints2[] =  {138, 452, 436, 108, 793};
  uint16_t myints3[] =  {1000, 4000, 2000, 1083, 93};
  std::vector<uint16_t> 
    tmpPts(myints, myints + sizeof(myints) / sizeof(uint16_t) );

  //get icrs from icr_server_node
  compute_icr_srv.request.centerpoint_ids = tmpPts;
  std::vector<uint8_t> tmp_used(5,1);
  compute_icr_srv.request.used = tmp_used;

  if (clientComputeICR.call(compute_icr_srv)) {
    ROS_INFO("Response: %ld", (long int)compute_icr_srv.response.success);
  }

  //convert it to proper point clouds
  pub = nh.advertise<sensor_msgs::PointCloud> ("output", 1);
  sensor_msgs::PointCloud output_cloud;
  plc::PointCloud<plc::PointXYZ>::Ptr cloud(new plc::PointCloud<plc::PointXYZRGB>() ); 

  // unsigned char rgb = {0,0,0};
  // for(uint i=0 ; i < cloud->size(); ++i)
  //   {
  //     cloud->points[i].rgb = *((float*)rgb);
  //   }
  // main loop
  ros::Rate loop_rate(1);
  while (ros::ok())
    {
      // Create a ROS publisher for the output point cloud
      if (clientComputeICR.call(compute_icr_srv) 
	  && compute_icr_srv.response.success == true) 
	{
	  icr2pc(compute_icr_srv.response, cloud);
	  pub.publish(cloud);
	  ros::spinOnce();
	}
      loop_rate.sleep();
    }
}


bool icr2pc(icr::compute_icr::Response &res_in, plc::PointCloud<plc::PointXYZ>::Ptr cloud) {
  ROS_INFO("converting ICR to PointCloud message");
  //  pc_out.header 
  for (uint j=0; j<res.stx.size();++j) {    
    //    cout << "icr " << j << endl;
    for(uint i=res.stx[j]; i<res.stx[j]+res.len[j] ;++i) {
      //cout << res.all_icrs.at(i) << " ";
      plc::PointXYZ pt;
      pt.x = rand();
      pt.y = rand();
      pt.z = rand();
      
      cloud.points->push_back(pt);
    }
    cout << endl;
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;

  return true;
}
