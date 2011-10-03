#include <ObstacleDetection.hh>
#include <IntensityDetection.hh>

using namespace lslgeneric;
using namespace lsl_od;
using namespace std;

int
main (int argc, char** argv)
{

    if(argc!=3) {
	cout<<"Utility to compute point clusters that correspond to previously un-observed obstacles\n";
	cout<<"usage: inMap point_cloud_model point_cloud_updated\n";
	return(-1);
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> tCloud, testCloud, drivable,obstacles;
    pcl::PointCloud<pcl::PointXYZ> outCloud;
    pcl::PointCloud<pcl::PointXYZ> outCloud1;
    pcl::PointCloud<pcl::PointXYZ> outCloud2;
    
    cloud = lslgeneric::readVRML(argv[1]);
    tCloud = lslgeneric::readVRML(argv[2]);
  

    //clean points 
    double robotInclination = 3*(M_PI/180);
    double MAX_OBST_HEIGHT = -0.3;
    double MAX_OBST_DIST = 25; 
    bool cutOffFar = false;
    pcl::PointXYZ scannerPosition;
    scannerPosition.x =4;
    scannerPosition.y =0;
    scannerPosition.z =0;
    Eigen::Vector3d robotSize, inclinationConstraints;
    robotSize<<9,6,3;
    inclinationConstraints<<15*M_PI/180,15*M_PI/180,2*M_PI;
    lslgeneric::RobotKinematics kin(robotSize,inclinationConstraints);

    pcl::PointXYZ pt = scannerPosition;
    //clean points from the scan that are inside the robot body
    for(int i=0; i<tCloud.points.size(); i++) {
	pcl::PointXYZ pt2 = tCloud.points[i];
	double dist = sqrt( (pt.x-pt2.x)*(pt.x-pt2.x) + (pt.y-pt2.y)*(pt.y-pt2.y) 
		+(pt.z-pt2.z)*(pt.z-pt2.z));
	if(dist > kin.robotRadius3d() && pt2.z < MAX_OBST_HEIGHT && pt2.x > 5) {
	    if((cutOffFar && dist < MAX_OBST_DIST) || (!cutOffFar)) {
		testCloud.push_back(pt2);
	    }
	}
		
    }

/////////////////tests///////
    fn = "ndt_tmp/clusters_cell_drivable_ndt_sp.wrl";
    cout<<fn<<endl;
    extractDrivable(testCloud,drivable,obstacles);
    detectOutliersNDT(cloud,obstacles,outCloud);
    cleanOutliersSaltPepper(outCloud,outCloud2);
    clusterCellsOutput(outCloud2, testCloud, fn);
/*
    fn = "ndt_tmp/clusters_cell_ndt_sp.wrl";
    cout<<fn<<endl;
    detectOutliersNDT(cloud,testCloud,outCloud);
    cleanOutliersSaltPepper(outCloud,outCloud2);
    clusterCellsOutput(outCloud2, testCloud, fn);

    fn = "ndt_tmp/clusters_cell_drivable_ndt.wrl";
    cout<<fn<<endl;
    extractDrivable(testCloud,drivable,obstacles);
    detectOutliersNDT(cloud,obstacles,outCloud);
    clusterCellsOutput(outCloud, testCloud, fn);
    
    fn = "ndt_tmp/clusters_cell_drivable_ndt.wrl";
    cout<<fn<<endl;
    detectOutliersNDT(cloud,testCloud,outCloud);
    clusterCellsOutput(outCloud, testCloud, fn);
///////

    fn = "ndt_tmp/clusters_cell_drivable_point_sp.wrl";
    cout<<fn<<endl;
    extractDrivable(testCloud,drivable,obstacles);
    detectOutliersPointBased(cloud,obstacles,outCloud);
    cleanOutliersSaltPepper(outCloud,outCloud2);
    clusterCellsOutput(outCloud2, testCloud, fn);

    fn = "ndt_tmp/clusters_cell_point_sp.wrl";
    cout<<fn<<endl;
    detectOutliersPointBased(cloud,testCloud,outCloud);
    cleanOutliersSaltPepper(outCloud,outCloud2);
    clusterCellsOutput(outCloud2, testCloud, fn);

    fn = "ndt_tmp/clusters_cell_drivable_point.wrl";
    cout<<fn<<endl;
    extractDrivable(testCloud,drivable,obstacles);
    detectOutliersPointBased(cloud,obstacles,outCloud);
    clusterCellsOutput(outCloud, testCloud, fn);
    
    fn = "ndt_tmp/clusters_cell_drivable_point.wrl";
    cout<<fn<<endl;
    detectOutliersPointBased(cloud,testCloud,outCloud);
    clusterCellsOutput(outCloud, testCloud, fn);
*/
     
    return (0);
}



