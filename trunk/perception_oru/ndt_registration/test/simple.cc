#include <ndt_matcher_p2d.h>
#include <ndt_matcher_d2d.h>
#include <ndt_map.h>
#include <pointcloud_utils.h>

#include "pcl/point_cloud.h"
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <iostream>

using namespace std;

int
main (int argc, char** argv)
{
    double roll=0,pitch=0,yaw=0,xoffset=0,yoffset=0,zoffset=0;

    pcl::PointCloud<pcl::PointXYZ> cloud, cloud_offset;
    char fname[50];
    FILE *fout;
    double __res[] = {0.5, 1, 2, 4};
    std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));

    struct timeval tv_start,tv_end,tv_reg_start,tv_reg_end;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tout;
    Tout.setIdentity();
    if(argc == 3)
    {

        gettimeofday(&tv_start,NULL);
        //we do a single scan to scan registration
        cloud = lslgeneric::readVRML<pcl::PointXYZ>(argv[1]);
        cloud_offset = lslgeneric::readVRML<pcl::PointXYZ>(argv[2]);
        
        Tout =  Eigen::Translation<double,3>(xoffset,yoffset,zoffset)*
            Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;
        
	lslgeneric::NDTMatcherD2D<pcl::PointXYZ,pcl::PointXYZ> matcherD2D(false, false, resolutions);
        bool ret = matcherD2D.match(cloud,cloud_offset,Tout,true);
        
	std::cout<<"Transform: \n"<<Tout.matrix()<<std::endl;

        snprintf(fname,49,"c_offset.wrl");
        fout = fopen(fname,"w");
        fprintf(fout,"#VRML V2.0 utf8\n");
        lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud,Eigen::Vector3d(1,0,0));
        lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud_offset,Eigen::Vector3d(1,1,1));

        lslgeneric::transformPointCloudInPlace<pcl::PointXYZ>(Tout,cloud_offset);
        lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud_offset,Eigen::Vector3d(0,1,0));
        fclose(fout);
    }
}
