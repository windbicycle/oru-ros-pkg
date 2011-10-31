#include <NDTMatcher.hh>
#include <NDTMatcherF2F.hh>
#include <NDTHistogram.hh>

namespace lslgeneric {

class MapVertex {
    public:
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> pose;
	pcl::PointCloud<pcl::PointXYZ> scan;
	int id;
	NDTHistogram hist;
	double timeRegistration;
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class MapEdge {
    
    public:
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> relative_pose;
	Eigen::Matrix<double,6,6> covariance;
	int idFirst, idSecond;
    public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


class NDTMapBuilder {

    public:
	NDTMapBuilder(bool _doHistogram = false){
	    isP2F = false; isF2F=false; doHistogram = _doHistogram; 
	}
	bool setICP() {
	    isP2F = false; isF2F=false;
	    return true; 
	}
	bool setMatcherP2F(NDTMatcher *_matcherP2F   ) {
	    matcherP2F  = _matcherP2F;
	    isP2F = true; isF2F=false;
	    return true; 
	}

	bool setMatcherF2F(NDTMatcherF2F *_matcherF2F) {
	    matcherF2F  = _matcherF2F;
	    isP2F = false; isF2F=true;
	    return true; 
	}

	bool addScan(pcl::PointCloud<pcl::PointXYZ> scan, int id=-1);
	void saveG2OlogFile(const char* fname);
	void saveDatlogFile(const char* fname);
	void printNodePositions();
	void theMotherOfAllPointClouds(const char* fname);
    
	lslgeneric::OctTree tr;

    private:
	NDTMatcher *matcherP2F;
	NDTMatcherF2F *matcherF2F;
	bool isP2F, isF2F, doHistogram;
        std::vector<MapVertex, Eigen::aligned_allocator<MapVertex> > vertices; 	
        std::vector<MapEdge, Eigen::aligned_allocator<MapEdge> > edges; 	
};
};
