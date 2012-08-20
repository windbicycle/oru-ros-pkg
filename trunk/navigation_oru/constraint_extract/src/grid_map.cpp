#include <grid_map.h>
#include <highgui.h>

using namespace std;

void RobotModelWithPose::setRobotParams( double _robotLength, double _robotWidth ) {
    robotLength = _robotLength;
    robotWidth = _robotWidth;
    radius = sqrt(robotLength*robotLength + robotWidth*robotWidth)/2;
}

void RobotModelWithPose::setPose(double px, double py, double pt) {
    pose.poseX = px;
    pose.poseY = py;
    pose.poseT = pt;
    formConstraints();
}
	
void RobotModelWithPose::setPose(const State2D &state) {
    pose = state;
    formConstraints();
}
	
void RobotModelWithPose::formConstraints() {
    //forms A and b from the robot size and pose
    vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > endpoints;
    Eigen::Vector2d ep;
    Eigen::Vector2d center;
    center<<pose.poseX,pose.poseY;

    //get the coordinates of the endpoints of the robot body
    ep<<-robotLength/2,-robotWidth/2;
    ep = (ep + center)*cos(pose.poseT);
    endpoints.push_back(ep);

    ep<<-robotLength/2,robotWidth/2;
    ep = (ep + center)*cos(pose.poseT);
    endpoints.push_back(ep);

    ep<<robotLength/2,robotWidth/2;
    ep = (ep + center)*cos(pose.poseT);
    endpoints.push_back(ep);

    ep<<robotLength/2,-robotWidth/2;
    ep = (ep + center)*cos(pose.poseT);
    endpoints.push_back(ep);

    //use same code as forming a polyogn now
    Eigen::Vector2d normal;
    for(size_t i=1; i<endpoints.size(); i++) {
	//to define the correct line direction, we also need a point on the inside of the constraint - the center 
	normal(0) = endpoints[i-1](1) - endpoints[i](1);
	normal(1) = endpoints[i](0) - endpoints[i-1](0);
	if(normal.dot(center) > 0) { //we want the outward pointing normal, so n.dot(s) < 0
	    normal = -normal;
	}
	normal.normalize();
	b(i-1) = -endpoints[i].dot(normal); //endpoints[i];
	A(i-1,0) = normal(0);
	A(i-1,1) = normal(1);
    }
    normal(0) = endpoints.back()(1) - endpoints.front()(1);
    normal(1) = endpoints.front()(0) - endpoints.back()(0);
    if(normal.dot(center) > 0) { //we want the outward pointing normal, so n.dot(s) < 0
	normal = -normal;
    }
    normal.normalize();
    b(endpoints.size()-1) = -endpoints.front().dot(normal); //endpoints[i];
    A(endpoints.size()-1,0) = normal(0);
    A(endpoints.size()-1,1) = normal(1);
}

bool RobotModelWithPose::isOccupied(double positionX, double positionY) const {
    Eigen::Vector2d x;
    Eigen::Matrix<double,4,1> res;
    x <<positionX,positionY;
    res = A*x;
    
    return (res(0) < b(0)) && (res(1) < b(1)) &&(res(2) < b(2)) && (res(3) < b(3)); 
}

void GridMap::allocateMap(size_t _sizeX, size_t _sizeY) {
    if(isAllocated) {
	deallocMap();
    }
    sizeX = _sizeX;
    sizeY = _sizeY;
    map = new GridCell*[sizeX];
    cellDiag = sqrt(resX*resX+resY*resY);
    centerPixel = cv::Point(sizeX/2, sizeY/2);
    for (size_t i = 0; i < sizeX; i++) 
    {
	map[i] = new GridCell[sizeY];
	for (size_t j = 0; j < sizeY; j++) 
	{
	    map[i][j].setOccupied(false);
	}
    }
    isAllocated = true;
}

void GridMap::deallocMap() {
    if(!isAllocated) {
	return;
    }
    for (size_t i = 0; i < sizeX; i++) 
    {
	delete[] map[i];
	map[i] = NULL;
    }
    delete[] map;
    map = NULL;
    isAllocated = false;
}

GridMap::GridMap(double _centerX, double _centerY, 
	double _resX, double _resY) {

    centerXmeters = _centerX;
    centerYmeters = _centerY;
    resX = _resX;
    resY = _resY;
}

void GridMap::loadFromDisk(const char* filename) {
    cv::Mat image = cv::imread(filename,0);
    if(image.data == NULL) {
	cout<<"UNSUPPORTED OR MISSING IMAGE "<<filename<<endl;
	return;
    }
    this->loadOpencvMap (image);
}

void GridMap::loadOpencvMap(const cv::Mat &image) {
    //first set sizeX, sizeY
    sizeX = image.rows;
    sizeY = image.cols;
    //allocate grid map
    allocateMap(sizeX,sizeY);
    //go through image and set occupancy 
    for (size_t i = 0; i < sizeX; i++) 
    {
	for (size_t j = 0; j < sizeY; j++) 
	{
	    map[i][j].setOccupied(image.at<uchar>(i,j) < 125);
	}
    }

}
	
void GridMap::getCvImage(cv::Mat &image) {

    if (!isAllocated) return;
    image.create(sizeX,sizeY,CV_8UC1);
    cout<<"iisize: "<<sizeX<<" "<<sizeY<<" "<<image.rows<<" "<<image.cols<<endl;
    for (size_t i = 0; i < sizeX; i++) 
    {
	for (size_t j = 0; j < sizeY; j++) 
	{
	    image.at<uchar>(i,j) = map[i][j].isOccupied() ? 0 : 255;
	}
    }
}

void GridMap::world2pixel(const Eigen::Vector2d &world, cv::Point &pixel) {

    //center in pixels = size in pixels/2
    pixel.y = (world(0)-centerXmeters)/resX + centerPixel.x;
    pixel.x = (world(1)-centerYmeters)/resY + centerPixel.y;

}

void GridMap::pixel2world(const cv::Point &pixel, Eigen::Vector2d &world) {
    world<<(pixel.y-centerPixel.x)*resX + centerXmeters,
           (pixel.x-centerPixel.y)*resY + centerYmeters;
}

void GridMap::precomputeAllowedPositions(Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> &allowedStates, 
	Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> &stateIntervals) {

    allowedStates = Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> (sizeX,sizeY);
    stateIntervals = Eigen::Matrix<StateInterval,Eigen::Dynamic,Eigen::Dynamic> (sizeX,sizeY);
    allowedStates.setOnes();
    Eigen::Vector2d inMeters;
    cv::Point pixel;
    double xc, yc;
    double theta_increment = 2*M_PI / (double) N_THETA_INCREMENTS;
    State2D pose;
    double rad = robot.getRadius() + cellDiag;
    int rad_pixelsX = rad / resX;
    int rad_pixelsY = rad / resY;

    cout<<"size: "<<sizeX<<" "<<sizeY<<endl;
    for(size_t x=0; x<sizeX; x++) {
	for(size_t y=0; y<sizeY; y++) {
	    if(map[x][y].isOccupied()) {
		allowedStates(x,y) = false;
		continue;
	    }
	    //set a robot pose to here
	    pose.poseX = (x-centerPixel.x)*resX + centerXmeters;
	    pose.poseY = (y-centerPixel.y)*resY + centerYmeters;
	    pose.poseT = 0; //p*theta_increment; 
	    robot.setPose(pose);

	    //if all cells within robotsize+cellDiagonal are free -> free
	    bool allFree = true;
	    for(int i = x - rad_pixelsX; i < x+rad_pixelsX; i++) {
		for(int j = y - rad_pixelsY; j < y+rad_pixelsY; j++) {
		    if(j >= sizeY || i >= sizeX || i<0 || j<0) {
			allFree = false;
			break;
		    }
		    if(map[i][j].isOccupied()) {
			allFree = false;
			break;
		    }
		}
	    }
	    if(allFree) {
		memset(stateIntervals(x,y).allowed_theta_phi,true,sizeof(bool)*N_THETA_INCREMENTS*N_PHI_INCREMENTS);
		//cout<<"all free at "<<x<<" "<<y<<endl;
		continue;
	    } else {
		//cout<<"oops at "<<x<<" "<<y<<endl;
	    }

	    for(int p=0; p<N_THETA_INCREMENTS; p++) {
		pose.poseT = p*theta_increment; 
		robot.setPose(pose);
		for(int q=0; q<N_PHI_INCREMENTS; q++) {
		    allFree =true;
		    for(int i = x - rad_pixelsX; i < x+rad_pixelsX; i++) {
			for(int j = y - rad_pixelsY; j < y+rad_pixelsY; j++) {
			    if(j >= sizeY || i >= sizeX || i<0 || j<0) {
				xc = (i-centerPixel.x)*resX + centerXmeters;
				yc = (j-centerPixel.y)*resY + centerYmeters;
				if(robot.isOccupied(xc,yc)) {
				    stateIntervals(x,y).allowed_theta_phi[p][q] = false;
				    allFree = false; i = INT_MAX; j = INT_MAX; break; //escape the two inner loops
				}
				continue;
			    }
			    if(map[i][j].isOccupied()) {
				xc = (i-centerPixel.x)*resX + centerXmeters;
				yc = (j-centerPixel.y)*resY + centerYmeters;
				if(robot.isOccupied(xc,yc)) {
				    stateIntervals(x,y).allowed_theta_phi[p][q] = false;
				    allFree = false; i = INT_MAX; j = INT_MAX; break; //escape the two inner loops
				}
			    }
			}
		    }
		    if(allFree) {
			//cout<<" free "<<x<<" "<<y<<" t "<<pose.poseT<<endl;
		    } else {
			//cout<<" eeek! "<<x<<" "<<y<<" t "<<pose.poseT<<endl;
			stateIntervals(x,y).allowed_theta_phi[p][q] = true;
		    }
		}
	    }

	}
    }


}
