#include <Cell.hh>

using namespace std;
using namespace lslgeneric;

/** default constructor, a Cell centered at the origin 
  */
Cell::Cell() {
    center.x = 0;
    center.y = 0;
    center.z = 0;
}

/** creates a new cell with specified parameters
  */
Cell::Cell(pcl::PointXYZ  _center, double &_xsize, double &_ysize, double &_zsize) {

    center = _center;
    xsize = _xsize;
    ysize = _ysize;
    zsize = _zsize;

}

/** copy constructor
  */
Cell::Cell(const Cell& other) {

    center = other.center;
    xsize = other.xsize;
    ysize = other.ysize;
    zsize = other.zsize;

}
/** empty destructor
  */
Cell::~Cell() {
}

/** getter for the cell center
  */
pcl::PointXYZ Cell::getCenter() {
    return center;
}
/** getter for the cell size
  */
void Cell::getDimensions(double &xs, double &ys, double &zs) const {
    xs = xsize;
    ys = ysize;
    zs = zsize;
}

/** performs a check if a point is inside the cell
  \param pt point to be tested
  */
bool Cell::isInside(pcl::PointXYZ pt) {
    if(pt.x < center.x-xsize/2 || pt.x > center.x+xsize/2) {
	return false;
    }
    if(pt.y < center.y-ysize/2 || pt.y > center.y+ysize/2) {
	return false;
    }
    if(pt.z < center.z-zsize/2 || pt.z > center.z+zsize/2) {
	return false;
    }
    return true;
}

/** setter for the cell center
  */
void Cell::setCenter(pcl::PointXYZ cn) {
    center = cn;
}

/** setter for size
  */
void Cell::setDimensions(const double &xs, const double &ys, const double &zs) {
    xsize = xs;
    ysize = ys;
    zsize = zs;
}

/** returns a cell of type Cell
  */
Cell* Cell::clone() {
    Cell *ret = new Cell();
    return ret;
}
/** returns an exact copy of this cell
  */
Cell* Cell::copy() {
    Cell *ret = new Cell();
    ret->setDimensions(xsize,ysize,zsize);
    ret->setCenter(center);

    return ret;
}
	    
