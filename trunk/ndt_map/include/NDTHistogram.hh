#ifndef NDT_HISTOGRAM_HH
#define NDT_HISTOGRAM_HH

#include <NDTMap.hh>

namespace lslgeneric {

class NDTHistogram {

    private:
	std::vector<int> histogramBinsFlat;
	std::vector<int> histogramBinsLine;
	std::vector<int> histogramBinsSphere;
	int N_LINE_BINS;
	int N_FLAT_BINS;
	int N_SPHERE_BINS;

	std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > directions;
	std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > averageDirections;
	void constructHistogram(NDTMap &map);
	void incrementLineBin();
	void incrementFlatBin(Eigen::Vector3d &normal);
	void incrementSphereBin();

	void computeDirections();
	void closedFormSolution(pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &trgt,
				Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);
    public:
	NDTHistogram (NDTMap &map);

	//get the transform that brings me close to target
	void bestFitToHistogram(NDTHistogram &target, Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);
	void printHistogram(bool bMatlab=false);

	pcl::PointCloud<pcl::PointXYZI> getDominantDirections(int nDirections);
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

};

#endif
