#include <IntensityDetection.hh>

using namespace lslgeneric;
using namespace lsl_id;
using namespace std;

int
main (int argc, char** argv)
{

    if(argc!=3) {
	cout<<"Utility to compute point clusters that correspond to previously un-observed obstacles\n";
	cout<<"usage: inMap point_cloud_model point_cloud_updated\n";
	return(-1);
    }
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<pcl::PointXYZI> tCloud, testCloud;
    pcl::PointCloud<pcl::PointXYZI> thresholdedCloud;

    cloud = lslgeneric::readVRMLIntensity(argv[1]);
    tCloud = lslgeneric::readVRMLIntensity(argv[2]);
  

    //clean points 
    double robotInclination = 3*(M_PI/180);
    double MAX_OBST_HEIGHT = 1;
    double MAX_OBST_DIST = 25; 
    bool cutOffFar = true;
    pcl::PointXYZ scannerPosition;
    scannerPosition.x =4;
    scannerPosition.y =0;
    scannerPosition.z =0;
    Eigen::Vector3d robotSize, inclinationConstraints;
    robotSize<<8,5,3;
    inclinationConstraints<<15*M_PI/180,15*M_PI/180,2*M_PI;
    lslgeneric::RobotKinematics kin(robotSize,inclinationConstraints);

    pcl::PointXYZ pt = scannerPosition;
    //clean points from the scan that are inside the robot body
    for(int i=0; i<tCloud.points.size(); i++) {
	pcl::PointXYZI pt2 = tCloud.points[i];
	double dist = sqrt( (pt.x-pt2.x)*(pt.x-pt2.x) + (pt.y-pt2.y)*(pt.y-pt2.y) 
		+(pt.z-pt2.z)*(pt.z-pt2.z));
	if(dist > kin.robotRadius3d() && pt2.z < MAX_OBST_HEIGHT) {
	    if((cutOffFar && dist < MAX_OBST_DIST) || (!cutOffFar)) {
		testCloud.push_back(pt2);
	    }
	}
		
    }
    lslgeneric::writeToVRML("ndt_tmp/intense.wrl",tCloud);

/////////////////tests///////
    std::string fn = "ndt_tmp/clusters_intensity.wrl";
    thresholdIntensities(testCloud,thresholdedCloud);
    clusterIntensityOutput(thresholdedCloud,tCloud,fn);

    /*
    cout<<fn<<endl;
    extractDrivable(testCloud,drivable,obstacles);
    detectOutliersNDT(cloud,obstacles,outCloud);
    cleanOutliersSaltPepper(outCloud,outCloud2);
    clusterCellsOutput(outCloud2, testCloud, fn);*/
     
    return (0);
}
