#ifndef NDT_HISTOGRAM_HH
#define NDT_HISTOGRAM_HH

#include <NDTMap.hh>
#include <vector>

namespace lslgeneric {

class NDTHistogram {

    private:
	std::vector<int> histogramBinsFlat;
	std::vector<int> histogramBinsLine;
	std::vector<int> histogramBinsSphere;
	
	int N_LINE_BINS;
	int N_FLAT_BINS;
	int N_SPHERE_BINS;
        double D1, D2;
	bool inited;

	std::vector< Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>, 
	    Eigen::aligned_allocator<Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> > > topThree;
	double topThreeS[3];

	std::vector<int> dist_histogramBinsFlat[3];
	std::vector<int> dist_histogramBinsLine[3];
	std::vector<int> dist_histogramBinsSphere[3];

	std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > directions;
	std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > averageDirections;
	void constructHistogram(NDTMap &map);
	void incrementLineBin(double d);
	void incrementFlatBin(Eigen::Vector3d &normal, double d);
	void incrementSphereBin(double d);

	void computeDirections();
	void closedFormSolution(pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &trgt,
				Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);
    public:
	NDTHistogram();
	NDTHistogram (NDTMap &map);
	NDTHistogram (const NDTHistogram& other);

	//get the transform that brings me close to target
	void bestFitToHistogram(NDTHistogram &target, Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T, bool bound_transform = true);
	void printHistogram(bool bMatlab=false);

	//call this to get the 1/2/3 best option, AFTER a call to bestFitToHistogram
	double getTransform(size_t FIT_NUMBER, Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
	    double ret = -1;
	    T.setIdentity();
	    if(FIT_NUMBER >=0 && FIT_NUMBER<3) {
		T = topThree[FIT_NUMBER];
		ret = topThreeS[FIT_NUMBER];
	    }
	    return ret;
	}

	pcl::PointCloud<pcl::PointXYZI> getDominantDirections(int nDirections);
	double getSimilarity(NDTHistogram &other);
	double getSimilarity(NDTHistogram &other, Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

};

#endif
