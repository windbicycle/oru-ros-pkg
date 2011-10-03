#ifndef OBSTACLE_DETECTION_HH
#define OBSTACLE_DETECTION_HH

#include <NDTMap.hh>
#include <OctTree.hh>
#include <AdaptiveOctTree.hh>
#include <PointCloudUtils.hh>
#include <NDTWavefrontPlanner.hh>
#include <RobotKinematics.hh>
#include <LazzyGrid.hh>
#include <NDTCell.hh>

#include "pcl/point_cloud.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/kdtree/kdtree_flann.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include <cstdio>

namespace lsl_od {

FILE *ffout;
static std::string fn;

void clusterPoints(pcl::PointCloud<pcl::PointXYZ> pc,
	    std::vector<pcl::PointCloud<pcl::PointXYZ> > &clusters); 

//estimate volume of a point cluster
double clusterVolume(pcl::PointCloud<pcl::PointXYZ> pc); 

//partition into drivable and obstacle regions (from wavefront)
void extractDrivable(pcl::PointCloud<pcl::PointXYZ> pc,
	pcl::PointCloud<pcl::PointXYZ> &drivable,
	pcl::PointCloud<pcl::PointXYZ> &occupied); 

//extract outliers using 3DNDT likelihood
void detectOutliersNDT(pcl::PointCloud<pcl::PointXYZ> pcStatic,
	pcl::PointCloud<pcl::PointXYZ> pcNew,
	pcl::PointCloud<pcl::PointXYZ> &outliers); 

//extract outliers using simple point-to-point distance
void detectOutliersPointBased(pcl::PointCloud<pcl::PointXYZ> pcStatic,
	pcl::PointCloud<pcl::PointXYZ> pcNew,
	pcl::PointCloud<pcl::PointXYZ> &outliers); 

//cleans outliers based on cell class (remove the flat horizontal ones)
void cleanOutliersCellClass(pcl::PointCloud<pcl::PointXYZ> pc,
	pcl::PointCloud<pcl::PointXYZ> outliersNoisy,
	pcl::PointCloud<pcl::PointXYZ> &outliersClean);

//remove isolated outlier points
void cleanOutliersSaltPepper(pcl::PointCloud<pcl::PointXYZ> pc,
	pcl::PointCloud<pcl::PointXYZ> &outliers); 

void clusterCellsOutput(pcl::PointCloud<pcl::PointXYZ> outCloud, pcl::PointCloud<pcl::PointXYZ> testCloud,
			std::string filename); 

};
#endif
