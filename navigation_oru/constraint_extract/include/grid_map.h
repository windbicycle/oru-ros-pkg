#ifndef GRID_MAP_H
#define GRID_MAP_H

#include<cv.h>
#include<Eigen/Core>

#define N_THETA_INCREMENTS 10
#define N_PHI_INCREMENTS 1

class State2D {
    public:
	double poseX, poseY, poseT;
	double linearVelocity, angularVelocity;
};

class RobotModelWithPose {
    private:
	State2D pose;
	double robotLength, robotWidth;
	double radius;
	Eigen::Matrix<double,4,2> A;
	Eigen::Matrix<double,4,1> b;
	void formConstraints();
    public:
	void setRobotParams( double _robotLength, double _robotWidth );
	void setPose(double px, double py, double pt);
	void setPose(const State2D &state);
	double getRadius() const { return radius; }
	State2D getPose() const { return pose; }
	bool isOccupied(double positionX, double positionY) const;
};

class GridCell {
    private:
	bool occupancy;
    public:
	GridCell() { occupancy = false; }
	GridCell(bool occ) { occupancy = occ; }
	GridCell(const GridCell& other) { occupancy = other.occupancy; }
	bool isOccupied() { return occupancy; }
	void setOccupied(bool occ) { occupancy = occ; }
};

struct StateInterval {
	bool allowed_theta_phi[N_THETA_INCREMENTS][N_PHI_INCREMENTS];
};

class GridMap {

    private:
	GridCell **map;
	double resX, resY;
	//double sizeXmeters, sizeYmeters;
        double centerXmeters, centerYmeters;
	bool isAllocated;
	cv::Point centerPixel;
	double cellDiag;

	void allocateMap(size_t _sizeX, size_t _sizeY);
	void deallocMap();

    public:
	GridMap(double _centerX, double _centerY, 
		double _resX, double _resY);
	~GridMap() {
	    deallocMap();
	}
	size_t sizeX, sizeY;
	RobotModelWithPose robot;

	void loadFromDisk(const char* filename);
	void loadOpencvMap(const cv::Mat &image);
	//bool isOccupied(const RobotModelWithPose& robot);
	void getCvImage(cv::Mat &image);

	void world2pixel(const Eigen::Vector2d &world, cv::Point &pixel);
	void pixel2world(const cv::Point &pixel, Eigen::Vector2d &world);
        void precomputeAllowedPositions(Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedState, 
		Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> &stateIntervals) ;
};


#endif
