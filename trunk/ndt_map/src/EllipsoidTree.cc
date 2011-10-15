#include <EllipsoidTree.hh>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <Debug.hh>

using namespace std;
using namespace lslgeneric;


bool EllipsoidTree::parametersSet = false;
double EllipsoidTree::MIN_CELL_SIZE;
double EllipsoidTree::FLAT_FACTOR;
	    
void EllipsoidTree::setParameters( double _MIN_CELL_SIZE,
                                    double _FLAT_FACTOR
				    ) {


    EllipsoidTree::MIN_CELL_SIZE        = _MIN_CELL_SIZE;
    EllipsoidTree::FLAT_FACTOR          = _FLAT_FACTOR;
    
    parametersSet = true;
}  

/** empty! default constructor
  */
EllipsoidTree::EllipsoidTree() : OctTree() {
    if(!EllipsoidTree::parametersSet) {
	DBG(1,"using default config\n");
	EllipsoidTree::setParameters();
    }
    myCell = new NDTCell();
}

/** constructor, calls parent OctTree constructor
  */
EllipsoidTree::EllipsoidTree(pcl::PointXYZ center, double xsize, double ysize, 
	double zsize, OctCell* type, OctTree *_parent, unsigned int _depth) : 
	OctTree(center,xsize,ysize,zsize,type,_parent,_depth) {

    if(!EllipsoidTree::parametersSet) {
	DBG(1,"using default config\n");
	EllipsoidTree::setParameters(NULL);
    }

    myCell = new NDTCell();
}

/** empty destructor, all data are deallocated by parent class
  */
EllipsoidTree::~EllipsoidTree() {

}
	    
//adds points to myCell
void EllipsoidTree::addPoint(pcl::PointXYZ point) {

}

/**
  finds all leafs of this tree and fills the vector of pointers to the leafs
  */
void EllipsoidTree::computeTreeLeafs() {
    if(this->isLeaf()) {
	myTreeLeafs.push_back(this);
	return;
    }

    myTreeLeafs.clear();
    vector<OctTree*> next;
    next.push_back(this);
    
    while(next.size()>0) {
	OctTree *cur = next.front();
	if(cur!=NULL) {
	    if(cur->isLeaf()) {
		myTreeLeafs.push_back(cur);
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

/**
go through all leafs and split the ones with high residuals
  */
void EllipsoidTree::postProcessPoints() {

    //compute leafs as OctTree*
    computeTreeLeafs();
    //cout<<"leafs :"<<myTreeLeafs.size()<<endl;

    for(unsigned int i=0; i<myTreeLeafs.size(); i++) {
	NDTCell * nd = dynamic_cast<NDTCell*>((myTreeLeafs[i])->myCell);
	if(nd == NULL) continue;
	nd->computeGaussian();
	if(!nd->hasGaussian) {
	    continue;
	}

	Eigen::Vector3d evals = nd->getEvals();
	int idMin, idMax;
	double minEval = evals.minCoeff(&idMin);
	double maxEval = evals.maxCoeff(&idMax);
	int idMiddle = -1;
	for(int j=0; j<3; j++) {
	    if(j!=idMin && j!=idMax) {
		idMiddle =  j;	
	    }	    
	}
	if(idMiddle < 0) continue;

	if(minEval*FLAT_FACTOR > evals(idMiddle)) {
	    vector<OctTree*> newLeafs = splitTree(myTreeLeafs[i]);
	    myTreeLeafs.insert(myTreeLeafs.end(),newLeafs.begin(),newLeafs.end());
	}
    }

    leafsCached = false;
}

//TODO: fix that 
/**
  splits a cell according to the eigenvectors and returns a vector of the newly created children
  iterates points down.
  */
vector<OctTree*> EllipsoidTree::splitTree(OctTree *octLeaf) {
    vector<OctTree*> newLeafs;

    if(octLeaf->isLeaf()) {
	double xs,ys,zs;
	octLeaf->myCell->getDimensions(xs,ys,zs);

	double cellSize = (xs+ys+zs)/3.; //average for now

	if(octLeaf->depth>MAX_DEPTH || cellSize <= MIN_CELL_SIZE )  {
	    //just store point, we can't split any more
	    return newLeafs; 
	}

	pcl::PointXYZ myCenter = octLeaf->myCell->getCenter();

	//branch leaf 
	for(unsigned int it=0; it<8; it++) {

	    pcl::PointXYZ newCenter;

	    //computes the center of the it'th child
	    newCenter.x = (myCenter.x + pow(-1.,it/4)*xs/4.);
	    newCenter.y = (myCenter.y + pow(-1.,it/2)*ys/4.);
	    newCenter.z = (myCenter.z + pow(-1.,(it+1)/2)*zs/4.);

	    octLeaf->children[it] = new OctTree(newCenter,xs/2,ys/2,
		    zs/2, octLeaf->myCell, this, depth+1);
	    newLeafs.push_back(octLeaf->children[it]);
	}

	//add current points
	for(unsigned int jt=0; jt<octLeaf->myCell->points.size(); jt++) {
	    size_t ind = octLeaf->getIndexForPoint(octLeaf->myCell->points[jt]);
	    octLeaf->children[ind]->addPoint(octLeaf->myCell->points[jt]);
	}
	octLeaf->leaf=false;
	octLeaf->myCell->points.clear();
    }

    return newLeafs; 
}

/**
  creates an oct tree with the same parameters. 
  \note the points are not copied in teh returned instance
  */
SpatialIndex* EllipsoidTree::clone() {
    if(myCell == NULL) {
	return new EllipsoidTree();
    }
    double sx,sy,sz;
    myCell->getDimensions(sx,sy,sz);
    EllipsoidTree *tr = new EllipsoidTree(myCell->getCenter(),sx,sy,sz,myCell);
    return tr;
}
