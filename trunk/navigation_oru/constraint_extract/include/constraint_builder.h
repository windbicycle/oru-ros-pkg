#ifndef CONSTRAINT_BUILDER_H
#define CONSTRAINT_BUILDER_H

#include <grid_map.h>
#include <vector>
#include <Eigen/Core>

class ConvexPolygon {
    private:
	Eigen::MatrixXd A; //A*x <= b
	Eigen::VectorXd b;
    public:
	double theta_min,theta_max,phi_min,phi_max,v_min,v_max,w_min,w_max;
	std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > endpoints;
	Eigen::Vector2d seed;
	
	void calculateMatrixForm();
	void writeToPolyFile(FILE* polyfile);
	void drawInImage(cv::Mat &image, GridMap* map);	
};

class ConstraintBuilder {

    private:
	std::vector<State2D> trajectory;
	GridMap *map;
	std::vector<ConvexPolygon, Eigen::aligned_allocator<ConvexPolygon> > polygons;
	Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> allowedStates;
	Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> stateIntervals;
	bool traceLine(size_t x1, size_t y1, size_t x2, size_t y2, size_t idx_min, size_t idx_max);
	bool isConvex(size_t x1, size_t y1, size_t x2, size_t y2, size_t px, size_t py, size_t cx, size_t cy);

    public:
	~ConstraintBuilder() { if(map!=NULL) delete map; }
	void loadConfigFile(const char *configFile);
	void processEnvironment();
	void savePolygonFile(const char*outputFile);
	void savePolygonPicture(const char*outputFile);
	
};

#endif
