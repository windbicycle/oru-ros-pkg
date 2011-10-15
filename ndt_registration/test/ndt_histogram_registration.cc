#include <NDTMatcherF2F.hh>
#include <NDTMap.hh>
#include <NDTHistogram.hh>
#include <OctTree.hh>
#include <AdaptiveOctTree.hh>
#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include "pcl/registration/icp.h"
#include "pcl/filters/voxel_grid.h"

#include <cstdio>
#include <Eigen/Eigen>
#include <PointCloudUtils.hh>
#include <fstream>

using namespace std;

int
main (int argc, char** argv)
{
    
    std::ofstream logger ("/home/tsv/ndt_tmp/results_histogram.txt");
    cout.precision(15);

    pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2, cloud3, cloud4 ;
    double __res[] = {0.2, 0.4, 1, 2};
    std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));
    lslgeneric::NDTMatcherF2F matcherF2F(false, false, false, resolutions);

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tin, Tout; 

    struct timeval tv_start, tv_end;

    Tout.setIdentity();
    bool succ = false;
    if(argc == 3) {
	//we do a single scan to scan registration
	
	cloud1 = lslgeneric::readVRML(argv[1]);
	cloud2 = lslgeneric::readVRML(argv[2]);
	
	// lslgeneric::AdaptiveOctTree::MIN_CELL_SIZE = 0.01;
	lslgeneric::OctTree tr;
	lslgeneric::OctTree::BIG_CELL_SIZE = 4; 
	lslgeneric::OctTree::SMALL_CELL_SIZE = 0.2; 
    
	gettimeofday(&tv_start,NULL);
	
	lslgeneric::NDTMap fixed(&tr);
	fixed.loadPointCloud(cloud1);
	lslgeneric::NDTMap moving(&tr);
	moving.loadPointCloud(cloud2);
   
	fixed.computeNDTCells();
	moving.computeNDTCells();
    
	lslgeneric::NDTHistogram fixedH(fixed);
	lslgeneric::NDTHistogram movingH(moving);
	
	cout<<"1 =========== \n";
	fixedH.printHistogram(true);
	cout<<"2 =========== \n";
	movingH.printHistogram(true);
    
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
	
	movingH.bestFitToHistogram(fixedH,T);
	cout<<" ==================== \n Transform R "<<T.rotation()<<"\nt "<<T.translation().transpose()<<endl; 
    
	cloud3 = lslgeneric::transformPointCloud(T,cloud2);
	lslgeneric::NDTMap moved(&tr);
	moved.loadPointCloud(cloud3);
	moved.computeNDTCells();
    
	lslgeneric::NDTHistogram movedH(moved);
	cout<<"3 =========== \n";
	movedH.printHistogram(true);

	bool ret = matcherF2F.match(cloud1,cloud3,Tout);
	//bool ret = matcherF2F.match(fixed,moved,Tout);
	gettimeofday(&tv_end,NULL);

	cout<<" TIME: "<<
	    (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.<<endl;

	char fname[50];
	snprintf(fname,49,"/home/tsv/ndt_tmp/c_offset.wrl");
	FILE *fout = fopen(fname,"w");
	fprintf(fout,"#VRML V2.0 utf8\n");
	//green = target
	lslgeneric::writeToVRML(fout,cloud1,Eigen::Vector3d(0,1,0));
	//red = before histogram
	lslgeneric::writeToVRML(fout,cloud2,Eigen::Vector3d(1,0,0));
	//blue = after histogram
	lslgeneric::writeToVRML(fout,cloud3,Eigen::Vector3d(0,0,1));
	
	if(!ret) { 
	    logger<<">>>>>>> NO SOLUTION <<<<<<<<<\n";
	    cout<<"NO SOLUTION\n";
	} 
        {	
	cloud4 = lslgeneric::transformPointCloud(Tout,cloud3);
	Eigen::Vector3d out = Tout.rotation().eulerAngles(0,1,2);
	cout<<"rot: "<<out.transpose()<<endl;
	cout<<"translation "<<Tout.translation().transpose()<<endl;
	logger<<"rot: "<<out.transpose()<<endl;
	logger<<"translation "<<Tout.translation()<<endl;
	//white = after registration
	lslgeneric::writeToVRML(fout,cloud4,Eigen::Vector3d(1,1,1));
	}

	fclose(fout);

	succ = true;
    }
    return 0;
}



