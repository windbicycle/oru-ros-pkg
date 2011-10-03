#ifndef INTENSITY_DETECTION_HH
#define INTENSITY_DETECTION_HH

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

namespace lsl_id {

    void thresholdIntensities(pcl::PointCloud<pcl::PointXYZI> &testCloud,
			      pcl::PointCloud<pcl::PointXYZI> &thresholdedCloud);

    void clusterIntensityOutput(pcl::PointCloud<pcl::PointXYZI> &thresholdedCloud,
				pcl::PointCloud<pcl::PointXYZI> &testCloud,
				std::string fn);

};

#endif
