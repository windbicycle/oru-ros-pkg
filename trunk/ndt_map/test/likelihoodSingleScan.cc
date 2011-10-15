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

static int ctr = 0;
int
main (int argc, char** argv)
{

    if(argc!=3) {
	cout<<"usage: ltest point_cloud1 point_cloud2\n";
	return(-1);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud, cloud2, cloud3;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    
    cloud = lslgeneric::readVRML(argv[1]);
    cloud2 = lslgeneric::readVRML(argv[2]);
  
    lslgeneric::OctTree::BIG_CELL_SIZE = 0.4; 
    lslgeneric::OctTree::SMALL_CELL_SIZE = 0.1; 
   // lslgeneric::AdaptiveOctTree::MIN_CELL_SIZE = 0.01;
    lslgeneric::OctTree tr;
    
    //lslgeneric::NDTMap nd(new lslgeneric::LazzyGrid(0.5));
    lslgeneric::NDTMap nd(&tr);
    nd.loadPointCloud(cloud);
    //lslgeneric::NDTMap nd2(new lslgeneric::LazzyGrid(0.5));
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
    //lslgeneric::NDTMap nd3(new lslgeneric::LazzyGrid(0.5));
    lslgeneric::NDTMap nd3(&tr);
    nd3.loadPointCloud(cloud3);
    nd3.computeNDTCells();
    
    lslgeneric::NDTHistogram nh3(nd3);
    cout<<"3 =========== \n";
    nh3.printHistogram(true);


    char fname[50];
    snprintf(fname,49,"/home/tsv/ndt_tmp/ndt_map.wrl");
    nd.writeToVRML(fname);
    snprintf(fname,49,"/home/tsv/ndt_tmp/original.wrl");
    lslgeneric::writeToVRML(fname,cloud);
    
    snprintf(fname,49,"/home/tsv/ndt_tmp/merged.wrl");
    
    FILE *f = fopen(fname,"w");
    fprintf(f,"#VRML V2.0 utf8\n");
    lslgeneric::writeToVRML(f,cloud,Eigen::Vector3d(1,1,1));
    lslgeneric::writeToVRML(f,cloud2,Eigen::Vector3d(1,0,0));
    lslgeneric::writeToVRML(f,cloud3,Eigen::Vector3d(0,1,0));
    fclose(f);

    double maxLikelihood = INT_MIN;
    double sumLikelihoods = 0;
    
    outCloud.points.resize(cloud.points.size());

    //loop through points and compute likelihoods LASER
    for(int i=0; i<cloud.points.size(); i++) {
	pcl::PointXYZ thisPt = cloud.points[i];
	//double likelihood = nd.getLikelihoodForPointWithInterpolation(thisPt);
	double likelihood = nd.getLikelihoodForPoint(thisPt);
	pcl::PointXYZI outPt;
	outPt.x = thisPt.x; 
	outPt.y = thisPt.y; 
	outPt.z = thisPt.z;
	outPt.intensity = likelihood;
	sumLikelihoods += likelihood;
	maxLikelihood = (likelihood > maxLikelihood) ? 
			    likelihood : maxLikelihood;
	outCloud.points[i] = outPt;
    }
    cout<<endl;
    cout<<"max likelihood "<<maxLikelihood<<endl;
    cout<<"sum likelihoods "<<sumLikelihoods<<endl;
    cout<<"average likelihood "<<sumLikelihoods/cloud.points.size()<<endl;
    //normalize for display
    //compute standart deviation
    for(int i=0; i<outCloud.points.size(); i++) {
	outCloud.points[i].intensity /= (maxLikelihood);
/*	outCloud.points[i].intensity = outCloud.points[i].intensity > 1 
	    ? 1 : outCloud.points[i].intensity;
*/
    }
    snprintf(fname,49,"/home/tsv/ndt_tmp/likelihood.wrl");
    lslgeneric::writeToVRML(fname,outCloud);
    return (0);
}



