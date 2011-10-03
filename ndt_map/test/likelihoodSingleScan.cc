#include <NDTMap.hh>
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

    if(argc!=2) {
	cout<<"usage: ltest point_cloud\n";
	return(-1);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZI> outCloud;
    
    cloud = lslgeneric::readVRML(argv[1]);
  
/*    lslgeneric::OctTree::BIG_CELL_SIZE = 0.4; 
    lslgeneric::OctTree::SMALL_CELL_SIZE = 0.1; 
    lslgeneric::AdaptiveOctTree::MIN_CELL_SIZE = 0.01;
    lslgeneric::AdaptiveOctTree tr;
    */
    lslgeneric::NDTMap nd(new lslgeneric::LazzyGrid(1));
    nd.loadPointCloud(cloud);
   
    nd.computeNDTCells(); 
    char fname[50];
    snprintf(fname,49,"/home/tsv/ndt_tmp/ndt_map.wrl");
    nd.writeToVRML(fname);
    snprintf(fname,49,"/home/tsv/ndt_tmp/original.wrl");
    lslgeneric::writeToVRML(fname,cloud);

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



