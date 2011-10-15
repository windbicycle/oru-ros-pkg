#include <NDTMap.hh>
#include <NDTHistogram.hh>
#include <OctTree.hh>
#include <AdaptiveOctTree.hh>
#include <PointCloudUtils.hh>

#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

#include <LazzyGrid.hh>

using namespace std;

int
main (int argc, char** argv)
{

    if(argc!=3) {
	cout<<"usage: histTest point_cloud1 point_cloud2\n";
	return(-1);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud2, cloud3;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    
    cloud = lslgeneric::readVRML(argv[1]);
    cloud2 = lslgeneric::readVRML(argv[2]);
  
    //lslgeneric::AdaptiveOctTree::MIN_CELL_SIZE = 0.01;
    lslgeneric::OctTree tr;
    lslgeneric::OctTree::BIG_CELL_SIZE = 4; 
    lslgeneric::OctTree::SMALL_CELL_SIZE = 1; 
    
    //lslgeneric::NDTMap nd(new lslgeneric::LazzyGrid(5));
    lslgeneric::NDTMap nd(&tr);
    nd.loadPointCloud(cloud);
    //lslgeneric::NDTMap nd2(new lslgeneric::LazzyGrid(5));
    lslgeneric::NDTMap nd2(&tr);
    nd2.loadPointCloud(cloud2);
   
    nd.computeNDTCells();
    nd2.computeNDTCells();
    
    lslgeneric::NDTHistogram nh(nd);
    lslgeneric::NDTHistogram nh2(nd2);
    cout<<"1 =========== \n";
    nh.printHistogram(true);
    cout<<"2 =========== \n";
    nh2.printHistogram(true);
    
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
	
    nh2.bestFitToHistogram(nh,T);
    cout<<" ==================== \n Transform R "<<T.rotation()<<"\nt "<<T.translation().transpose()<<endl; 
    
    cloud3 = lslgeneric::transformPointCloud(T,cloud2);
    //lslgeneric::NDTMap nd3(new lslgeneric::LazzyGrid(5));
    lslgeneric::NDTMap nd3(&tr);
    nd3.loadPointCloud(cloud3);
    nd3.computeNDTCells();
    
    lslgeneric::NDTHistogram nh3(nd3);
    cout<<"3 =========== \n";
    nh3.printHistogram(true);


    char fname[50];
    snprintf(fname,49,"/home/tsv/ndt_tmp/ndt_map.wrl");
    nd.writeToVRML(fname);

    snprintf(fname,49,"/home/tsv/ndt_tmp/histogramRegistered.wrl");
    FILE *f = fopen(fname,"w");
    fprintf(f,"#VRML V2.0 utf8\n");
    //green = target
    lslgeneric::writeToVRML(f,cloud,Eigen::Vector3d(0,1,0));
    //red = init
    lslgeneric::writeToVRML(f,cloud2,Eigen::Vector3d(1,0,0));
    //white = final
    lslgeneric::writeToVRML(f,cloud3,Eigen::Vector3d(1,1,1));
    fclose(f);

    return (0);
}



