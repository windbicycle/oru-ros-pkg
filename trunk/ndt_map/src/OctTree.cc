#include <OctTree.hh>
#include <Debug.hh>
#include "ros/ros.h"
#include <NDTCell.hh>

#include <climits>
using namespace std;
using namespace lslgeneric;

/** output to a vrml file, color is always red. 
  */
void OctCell::writeToVRML(FILE *fout, Eigen::Vector3f color) {

    if(fout==NULL) {
	ERR("writeToVRML failed, null FILE pointer\n");
	return;
    }

    fprintf(fout,"Shape {\n\tgeometry IndexedFaceSet {\n\t\tcoord \
	    Coordinate {\n\t\tpoint [\n");
    fprintf(fout,"%lf %lf %lf\n", center.x-xsize/2, center.y-ysize/2, center.z-zsize/2);
    fprintf(fout,"%lf %lf %lf\n", center.x+xsize/2, center.y-ysize/2, center.z-zsize/2);
    fprintf(fout,"%lf %lf %lf\n", center.x+xsize/2, center.y+ysize/2, center.z-zsize/2);
    fprintf(fout,"%lf %lf %lf\n", center.x-xsize/2, center.y+ysize/2, center.z-zsize/2);

    fprintf(fout,"%lf %lf %lf\n", center.x-xsize/2, center.y-ysize/2, center.z+zsize/2);
    fprintf(fout,"%lf %lf %lf\n", center.x+xsize/2, center.y-ysize/2, center.z+zsize/2);
    fprintf(fout,"%lf %lf %lf\n", center.x+xsize/2, center.y+ysize/2, center.z+zsize/2);
    fprintf(fout,"%lf %lf %lf\n", center.x-xsize/2, center.y+ysize/2, center.z+zsize/2);

    fprintf(fout,"]\n}\ncolor Color {\n\t color [\n");
    for(int i=0; i<8; i++) {
	fprintf(fout,"%lf %lf %lf\n", color(0),color(1),color(2) );
    }
    
    fprintf(fout,"]\n}\n coordIndex [\n");
    fprintf(fout,"0 1 2 3 -1\n\
	    4 5 6 7 -1\n\
	    0 1 5 4 -1\n\
	    1 2 6 5 -1\n\
	    2 3 7 6 -1\n\
	    3 0 4 7 -1\n\
	    ]\n}\n}");

}

/** empty
  */
OctCell::OctCell() : Cell() {

}

/** calls parent
  */
OctCell::OctCell(pcl::PointXYZ _center, double &xsize, double &ysize, double &zsize) :
    Cell(_center,xsize,ysize,zsize) {

}

/** copy constructor
  */
OctCell::OctCell(const OctCell& other):Cell() {
    
    center = other.center;
    xsize = other.xsize;
    ysize = other.ysize;
    zsize = other.zsize;

}

/** return a new cell of type OctCell
  */
Cell* OctCell::clone() {
    OctCell *ret = new OctCell();
    return ret;
}

/** return an exact copy of this cell
  */
Cell* OctCell::copy() {
    OctCell *ret = new OctCell();
    ret->setDimensions(xsize,ysize,zsize);
    ret->setCenter(center);
   
    for(unsigned int i=0; i<this->points.size(); i++) {
	pcl::PointXYZ pt = this->points[i];
	ret->points.push_back(pt);
    }	
    return ret;
}

	    

int OctTree::MAX_POINTS, OctTree::MAX_DEPTH;
double OctTree::BIG_CELL_SIZE, OctTree::SMALL_CELL_SIZE;

bool OctTree::parametersSet = false;
	    
void OctTree::setParameters(double _BIG_CELL_SIZE	,
                            double _SMALL_CELL_SIZE     ,	
			    int _MAX_POINTS		,
                            int _MAX_DEPTH		
			    ) {

    //defaults
    OctTree::MAX_POINTS		= _MAX_POINTS		;
    OctTree::MAX_DEPTH		= _MAX_DEPTH		;
    OctTree::BIG_CELL_SIZE	= _BIG_CELL_SIZE	;
    OctTree::SMALL_CELL_SIZE	= _SMALL_CELL_SIZE	;
    parametersSet = true;
}
/** returns the child index that should hold the point. 
    \note the point is not necessarily within the child's boundaries, 
    but is guaranteed to be in the same quadrant (octant?)
  */
size_t OctTree::getIndexForPoint(const pcl::PointXYZ pt) {
    /** index table: (convenient computations)
      *	id  x	y   z
      * 0   +	+   +
      * 1   +	+   -
      * 2   +	-   -
      * 3   +	-   +
      * 4   -	+   +
      * 5   -	+   -
      * 6   -	-   -
      * 7   -	-   +
      */
    
    size_t ret = 0;
    if(pt.x < myCell->getCenter().x) {
	ret += 4;
    }
    if(pt.y < myCell->getCenter().y) {
	ret += 2;
    }
    if(ret%4==0) {
	if(pt.z < myCell->getCenter().z) {
	    ret += 1;
	} 
    } else {
	if(pt.z > myCell->getCenter().z) {
	    ret += 1;
	} 
    }	
    return ret;
}

/** empty constructor
  */
OctTree::OctTree() {

    parent=NULL;
    leaf=true;
    depth=0;
    for(unsigned int it=0; it<8; it++) {
	children[it]=NULL;
    }
    myCell = NULL;
    leafsCached = false;
    if(!OctTree::parametersSet) {
	DBG(1,"using default config\n");
	OctTree::setParameters();
    }
}

/** parametrized constructor
  */
OctTree::OctTree(pcl::PointXYZ center, double xsize, double ysize, 
	double zsize, OctCell* type, OctTree *_parent, unsigned int _depth) {

    parent=_parent;
    leaf=true;
    depth=_depth;
    for(unsigned int it=0; it<8; it++) {
	children[it]=NULL;
    }

    OctCell* tmp = dynamic_cast<OctCell*>(type->clone());
    if(tmp==NULL) {
	ERR("dynamic cast of cell failed!!\n");
	return;
    }
    tmp->setCenter(center);
    tmp->setDimensions(xsize,ysize,zsize);
    tmp->points.clear();
    myCell = tmp;
    leafsCached = false;
    if(!OctTree::parametersSet) {
	DBG(1,"using default config\n");
	OctTree::setParameters();
    }
}

/** destructor, deletes all pointers starting from parent and going down
  */
OctTree::~OctTree() {

    if(!leaf) {
	for(unsigned int it=0; it<8; it++) {
	    if(children[it]!=NULL) {
		delete children[it]; //calls destructor of child, so we are ok
		children[it]=NULL;
	    }
	}
    }
    delete myCell;
}

/** adds a point to the index. Iterates down the tree and if necessary
  creates new leafs and splits current ones.
  \note at the moment root is not grown in case of points outside!
  */
void OctTree::addPoint(pcl::PointXYZ point) {

    leafsCached = false;
    if(leaf) {
	double xs,ys,zs;
	myCell->getDimensions(xs,ys,zs);

	double cellSize = (xs+ys+zs)/3.; //average for now

	if(myCell->points.size()<MAX_POINTS && cellSize <= BIG_CELL_SIZE) {
	    if(!myCell->isInside(point)) {
		DBG(1,"OctTree: addPoint (%lf,%lf,%lf) not in boundary!\n",point.x,point.y,point.z);
		return;
	    }
	    myCell->points.push_back(point);
	} else {
	    if(depth>MAX_DEPTH || cellSize <= 2*SMALL_CELL_SIZE )  { 
		//TSV: we have to be sure we won't violate the space constraint if we split
		//just store point, we can't split any more
		if(!myCell->isInside(point)) {
		    DBG(1,"OctTree: addPoint (%lf,%lf,%lf) not in boundary!\n",point.x,point.y,point.z);
		    return;
		}
		myCell->points.push_back(point);
		return; 
	    }

	    pcl::PointXYZ myCenter = myCell->getCenter();

	    //branch leaf 
	    for(int it=0; it<8; it++) {
		pcl::PointXYZ newCenter;
		
		//computes the center of the it'th child
		newCenter.x = (myCenter.x + pow(-1.,it/4)*xs/4.);
		newCenter.y = (myCenter.y + pow(-1.,it/2)*ys/4.);
		newCenter.z = (myCenter.z + pow(-1.,(it+1)/2)*zs/4.);

		children[it] = new OctTree(newCenter,xs/2,ys/2,
			zs/2, myCell, this, depth+1);
	    }
	    //add current points
	    for(unsigned int jt=0; jt<myCell->points.size(); jt++) {
		size_t ind = getIndexForPoint(myCell->points[jt]);
		children[ind]->addPoint(myCell->points[jt]);
	    }
	    //finally add the new point
	    size_t ind = getIndexForPoint(point);
	    children[ind]->addPoint(point);
	    leaf=false;
	    myCell->points.clear();
	}
    } else {
	//pass down to correct child
	size_t ind = getIndexForPoint(point);
	children[ind]->addPoint(point);
    }
}

/** getter for the cell
  */
Cell* OctTree::getMyCell() {
    return myCell;
}

/** returns the cell that should hold the point
  */
Cell* OctTree::getCellForPoint(pcl::PointXYZ point) {

    OctTree* pointLeaf = this->getLeafForPoint(point);
    return (pointLeaf==NULL) ? NULL : pointLeaf->myCell;

}

/** finds the leaf that should hold the point
  */
OctTree* OctTree::getLeafForPoint(pcl::PointXYZ point) {

    if(this->leaf && myCell!= NULL) {
	if(myCell->isInside(point)) {
	    return this;
	}
    } else {
	size_t ind = getIndexForPoint(point);
	if(children[ind]!=NULL) {
	    return children[ind]->getLeafForPoint(point);
	}
    }
    return NULL;

}
	    
/** finds all children cells that are located at current leafs 
  */
void OctTree::computeLeafCells() {
    if(this->isLeaf()) {
	myLeafs.push_back(this->myCell);
	return;
    }

    myLeafs.clear();
    vector<OctTree*> next;
    next.push_back(this);
    
    while(next.size()>0) {
	OctTree *cur = next.front();
	if(cur!=NULL) {
	    if(cur->isLeaf()) {
		myLeafs.push_back(cur->myCell);
	    } else {
		for(int i=0; i<8; i++) {
		    OctTree* tmp = cur->getChild(i);
		    if(tmp!=NULL) {
			next.push_back(tmp);
		    }
		}
	    }
	}
	next.erase(next.begin());
    }
}

/** sets the cell factory type
  */
void OctTree::setCellType(Cell *type) {

    myCell = dynamic_cast<OctCell*>(type->clone());
    if(myCell == NULL) {
	//cast failed, it's not a derivative of oct cell
	myCell = new OctCell();
	DBG(1,"please provide a cell type derived from OctCell!\n");
    }

}

/** iterator poining to the first leaf cell.
  * \note recomputes the vector when changes have occured!
  */
vector<Cell*>::iterator OctTree::begin() {
    if(!leafsCached) {
	myLeafs.clear();
	computeLeafCells();
    }
    leafsCached = true;
    return myLeafs.begin();
}

/** iterator poining at last leaf cell
  */
vector<Cell*>::iterator OctTree::end() {
    if(!leafsCached) {
	myLeafs.clear();
	computeLeafCells();
    }
    leafsCached = true;
    return myLeafs.end();
}
	    
/** output to vrml
  */
void OctTree::writeToVRML(const char* filename) {
    if(filename == NULL) {
	ERR("OctTree: attempting to print to a null filename \n");
	return;
    }

    FILE* fout = fopen (filename,"w");
    if(fout == NULL) {
	ERR("OctTree: attempting to write to a null file %s\n",filename);
	return;
    }
    fprintf(fout,"#VRML V2.0 utf8\n");
    this->writeToVRML(fout);
    fclose(fout);
}

/** output to vrml using file pointer
  */
void OctTree::writeToVRML(FILE* fout) {
    
    this->computeLeafCells();
    vector<Cell*>::iterator it = this->begin();
    Eigen::Vector3f col;
    col<<0,0,0;
    while(it!= this->end()) {
	OctCell* cell = dynamic_cast<OctCell*>(*it);
	if(cell!=NULL) {
	    cell->writeToVRML(fout,col);
	}
	it++;
    }
}

/** returns a new OctTree spatial index
  */
SpatialIndex* OctTree::clone() {
    if(myCell == NULL) {
	return new OctTree();
    }

    OctTree *tr = new OctTree();
    tr->setCellType(myCell);
    return tr;
}

/** copies the spatial index
  */
SpatialIndex* OctTree::copy() {
    if(myCell == NULL) {
	return new OctTree();
    }

    pcl::PointXYZ center = myCell->getCenter();
    double sx,sy,sz;
    myCell->getDimensions(sx,sy,sz);

    OctTree *tr = new OctTree(center,sx,sy,sz,myCell);
    return tr;
}

/** sets the center. 
  \note this is not going to re-compute cells that are already in the tree!
  */
void OctTree::setCenter(const double &cx, const double &cy, const double &cz) {
    if(myCell == NULL) {
	return;
    }
    pcl::PointXYZ center;
    center.x = cx;
    center.y = cy;
    center.z = cz;

    myCell->setCenter(center);
}

/** sets the size
  \note this is not going to re-compute cells that are already in the tree!
  */
void OctTree::setSize(const double &sx, const double &sy, const double &sz) {
    if(myCell == NULL) {
	return;
    }
    myCell->setDimensions(sx,sy,sz);
}

/** returns all the neighboring cells within a radius
   \param point the location around which we are looking for neighbors. The point must be inside the boundaries of a current leaf!
   \param radius the neighbor tolerance
   \param cells output 
  */
void OctTree::getNeighbors(pcl::PointXYZ point, const double &radius, std::vector<Cell*> &cells) {

    cells.clear();
    //first find the leaf that contains the point
    OctTree *target = this->getLeafForPoint(point);
    if(target==NULL) return;

    OctTree *mparent = target->parent;
    OctTree *mthis = target;
    vector<OctTree*> toExpand;
    //check if any of the siblings intersect the sphere (key,constraint)

    while(mparent!=NULL) {
	for(unsigned int it=0; it<8; it++) {
	    if(mparent->children[it] == NULL) continue;
	    if(mparent->children[it]->intersectSphere(point,radius)
		    && mparent->children[it]!=mthis ) {
		//if yes, add them to the list to expand
		toExpand.push_back(mparent->children[it]);
	    }
	}
	//go up to parent
	mthis=mparent;
	mparent=mparent->parent;
	//for all nodes in list, go down to leafs that intersect
	for(unsigned int nt=0; nt<toExpand.size(); nt++) {
	    if(toExpand[nt] == NULL ) {
		ERR("ERROR in nearest neighbor!!\n");
		continue;
	    }

	    pcl::PointXYZ center = (toExpand[nt]->myCell->getCenter());
	    Eigen::Vector3d d;
	    d<<center.x-point.x, center.y-point.y, center.z-point.z;
	    if(toExpand[nt]->isLeaf() && 
		    d.norm() < radius) {
		cells.push_back(toExpand[nt]->myCell);
	    } else {
		for(unsigned int it=0; it<8; it++) {
		    if(toExpand[nt]->children[it]==NULL) continue;
		    if(toExpand[nt]->children[it]->intersectSphere(point,radius)) {
			toExpand.push_back(toExpand[nt]->children[it]);
		    }
		}
	    }
	}

	toExpand.clear();
    }

}

/** checks if the tree node intersects the sphere located at center and of size radius
  */
bool OctTree::intersectSphere(pcl::PointXYZ center, const double &radius) const {
    
    pcl::PointXYZ mcenter = myCell->getCenter();
    Eigen::Vector3d d;
    d<<center.x-mcenter.x, center.y-mcenter.y, center.z-mcenter.z;
    double dist = d.norm();
    Eigen::Vector3d localRadius;
    myCell->getDimensions(localRadius(0),localRadius(1),localRadius(2));
    double lRad = localRadius.norm()/2;
    double interDist = lRad+radius;
    return (interDist>dist);

}

Cell* OctTree::getClosestLeafCell(pcl::PointXYZ point) {
    if(this->leaf && myCell!= NULL) {
	if(myCell->isInside(point)) {
	    return myCell;
	}
    } else {
	size_t ind = getIndexForPoint(point);
	if(children[ind]!=NULL) {
	    return children[ind]->getClosestLeafCell(point);
	} else {
	    //the leaf we should be in is empty
	    //start from here and find closest neighbor
	    double minDist = INT_MAX;
	    int index = -1;
	    for(int i=0; i<8; i++) {
		if(children[i]==NULL) continue;
		pcl::PointXYZ center = children[i]->myCell->getCenter();
		Eigen::Vector3d d;
		d <<center.x-point.x, center.y-point.y, center.z-point.z;
		double dist = d.norm();

		if(dist<minDist) {
		    index = i;
		    minDist = dist;
		}
	    }
	    cout<<"minDist"<<minDist<<endl;
	    if(index>=0 && index<8)
		return children[index]->getClosestLeafCell(point);
	}
    }
    return myCell;
}

NDTCell* OctTree::getClosestNDTCell(const pcl::PointXYZ point) {
    
    if(this->leaf) {
	//we have reached the bottom of the tree. 
	//if we have a valid ndt cell, return it. 
	if(myCell->isInside(point)) {
	    NDTCell* nd = dynamic_cast<NDTCell*>(myCell);
	    if(nd!=NULL) {
		if(nd->hasGaussian) {
//		    cout<<"easy pissy\n";
		    return nd;
		}
	    }
	}
    } 
    
    //we go down the tree recursively
    size_t ind = getIndexForPoint(point);
    if(children[ind]!=NULL) {
	return children[ind]->getClosestNDTCell(point);
    } 
    

    //the leaf we should be in is empty
    //iterate through all leafs connected to current node, find closest ndt cell
    OctTree *my_parent = this->parent;
    OctTree *me = this;
    vector<Cell*>::iterator it;
    NDTCell *closest = NULL, *temp = NULL;
    double minDist = INT_MAX, dist = INT_MAX;

    while(true) {
	it = me->begin();
	while(it!=me->end()) {
	    temp = dynamic_cast<NDTCell*> (*it);
	    if(temp!=NULL) {
		if(temp->hasGaussian) {
		    dist = lslgeneric::geomDist(temp->getCenter(),point);
		    if(dist < minDist) {
			minDist = dist;
			closest = temp;
		    }
		}
	    }
	    it++;
	}
	if(closest!=NULL) {
//	    cout<<"got it!\n";
	    break;
	}
	if(my_parent != NULL) {
	    me = my_parent;
	    my_parent = me->parent;
	} else {
//	    cout<<"givin up\n";
//	    cout<<point.x<<" "<<point.y<<" "<<point.z<<endl;
	    //nothing more can be done...
	    break;
	}
    }
    
    return closest;

}
