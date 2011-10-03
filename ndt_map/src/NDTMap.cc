#include <NDTMap.hh>
#include <NDTCell.hh>
#include <OctTree.hh>
#include <AdaptiveOctTree.hh>
#include <Debug.hh>
#include <Eigen/Eigen>
#include <pcl/features/feature.h>
#include <LazzyGrid.hh>
#include <PointCloudUtils.hh>

#include <string>
#include <climits>
using namespace std;
using namespace lslgeneric;

bool NDTMap::parametersSet = false;
	    
void NDTMap::setParameters() {

    //no parameters atm
    parametersSet = true;
}

/** default constructor. The SpatialIndex sent as a paramter 
  is used as a factory every time that loadPointCloud is called. 
  it can/should be deallocated outside the class after the destruction of the NDTMap
  */
NDTMap::NDTMap(SpatialIndex *si) {
    if(!parametersSet) {
	DBG(1,"using default config\n");
	setParameters();
    }

    index = si;

    //this is used to prevent memory de-allocation of the *si
    //si was allocated outside the NDT class and should be deallocated outside
    isFirstLoad=true;
}

//copy constructor, clones the current points and cells in the index...
NDTMap::NDTMap(const NDTMap& other) {

    if(other.index != NULL) {
	this->index = index->copy();
	isFirstLoad = false;
    }	
    parametersSet = true;
}

/** destructor, cleans up the index
  */
NDTMap::~NDTMap() {
    if(index !=NULL && !isFirstLoad) {
	//cout<<"deleting index\n";
	delete index;
    }
}

/** Main implemented method for now. Loads a single point cloud into an NDTMap.
  * \param pc the PointCloud that is to be loaded
  * \note every subsequent call will destroy the previous map!
  */
void NDTMap::loadPointCloud(pcl::PointCloud<pcl::PointXYZ> pc) {
    if(index != NULL) {
	SpatialIndex *si = index->clone();
	//cout<<"allocating index\n";
	if(!isFirstLoad) { 
	    //cout<<"deleting old index\n";
	    delete index;
	}
	index = si;
    } else {
	//NULL index in constructor, abort!
	ERR("constructor must specify a non-NULL spatial index\n");
	return;
    }
    
    if(index == NULL) {
	ERR("Problem creating index, unimplemented method\n");
	return;
    }
    
    double maxDist = 0, distCeil = 40;


    //cout<<"Points = " <<pc.points.size()<<endl;
    pcl::PointCloud<pcl::PointXYZ>::iterator it = pc.points.begin();
    while(it!=pc.points.end()) {
	Eigen::Vector3d d;
	d << it->x, it->y, it->z;
	double dist = d.norm();
	if(dist > distCeil || fabsf(it->x) > distCeil|| fabsf(it->y) > distCeil|| fabsf(it->z) > distCeil) {
	    pcl::PointCloud<pcl::PointXYZ>::iterator itTMP = it;
	    it++;
	    pc.points.erase(itTMP);
	} else {
	    it++;
	}
    }
    
    Eigen::Vector4f centroid(0,0,0,0);
    pcl::compute3DCentroid(pc,centroid);
    
    //compute distance to furthest point
    it = pc.points.begin();
    while(it!=pc.points.end()) {
	Eigen::Vector3d d;
	d << centroid(0)-it->x, centroid(1)-it->y, centroid(2)-it->z;
	double dist = d.norm();
	maxDist = (dist > maxDist) ? dist : maxDist;
	it++;
    }
   // cout<<"Points = " <<pc.points.size()<<" maxDist = "<<maxDist<<endl;

    NDTCell *ptCell = new NDTCell();
    index->setCellType(ptCell);
    delete ptCell;
    index->setCenter(centroid(0),centroid(1),centroid(2));
    index->setSize(4*maxDist,4*maxDist,4*maxDist);

//    ROS_INFO("centroid is %f,%f,%f", centroid(0),centroid(1),centroid(2));
//    ROS_INFO("maxDist is %lf", maxDist);

    it = pc.points.begin();
    while(it!=pc.points.end()) {
	index->addPoint(*it);
	it++;
    }

    isFirstLoad = false;
}

/** Helper function, computes the  NDTCells
  */
void NDTMap::computeNDTCells() {
    AdaptiveOctTree *aot = dynamic_cast<AdaptiveOctTree*>(index);
    if(aot!=NULL) {
	ROS_INFO("post processing points!\n");
	aot->postProcessPoints();
    }

    vector<Cell*>::iterator it = index->begin();
    while (it != index->end()) {
	NDTCell *cell = dynamic_cast<NDTCell*> (*it);
	if(cell!=NULL) {
	    cell->computeGaussian();
	} else {
	    ERR("problem casting cell to NDT!\n");
	}
	it++;
    }

    LazzyGrid *lz = dynamic_cast<LazzyGrid*>(index);
    if(lz!=NULL) {
	lz->initKDTree();
    }
}

/** output method, saves as vrml the oct tree and all the ellipsoids
 */
void NDTMap::writeToVRML(const char* filename) {

    if(filename == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }
    /*
       std::string fn(filename);
       fn = "oct_"+fn;

       FILE *fo = fopen(fn.c_str(),"w");
       if(fo == NULL) {
       ERR("problem outputing to vrml\n");
       return;
       }
       fprintf(fo,"#VRML V2.0 utf8\n");
       writeToVRML(fo,true);
       fclose(fo);
     */   

    FILE *fout = fopen(filename,"w");
    if(fout == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }
    fprintf(fout,"#VRML V2.0 utf8\n");
    writeToVRML(fout,false);
    fclose(fout);
}

/** helper output method
 */
void NDTMap::writeToVRML(FILE* fout, bool bOctMap) {
    if(fout == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }

    if(bOctMap) {
	OctTree* ind = dynamic_cast<OctTree*>(index);
	ind->writeToVRML(fout);
	return;
    }

    //move the ellipsoid stuff to NDTCell
    vector<Cell*>::iterator it = index->begin();
    while (it != index->end()) {
	NDTCell *cell = dynamic_cast<NDTCell*> (*it);
	//	double xs,ys,zs;
	if(cell!=NULL) {
	    cell->writeToVRML(fout);
	} else {
	    //	    ERR("problem casting cell to NDT!\n");
	}
	it++;
    }
   /* 
    LazzyGrid* ind = dynamic_cast<LazzyGrid*>(index);
    if(ind!=NULL) {
	ind->writeLinksVRML(fout);
    }
    */

}
void NDTMap::writeToVRML(FILE* fout, Eigen::Vector3d col) {
    if(fout == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }

    //move the ellipsoid stuff to NDTCell
    vector<Cell*>::iterator it = index->begin();
    while (it != index->end()) {
	NDTCell *cell = dynamic_cast<NDTCell*> (*it);
	if(cell!=NULL) {
	    cell->writeToVRML(fout,col);
	} else {
	}
	it++;
    }
}

/** returns the current spatial index
 */
SpatialIndex* NDTMap::getMyIndex() const {
    return index;
}

//computes the *negative log likelihood* of a single observation
double NDTMap::getLikelihoodForPoint(pcl::PointXYZ pt) {
    double uniform=0.00100;
    NDTCell* ndCell = NULL;
    OctTree* tr = dynamic_cast<OctTree*>(index);

    if(tr==NULL) { 
	LazzyGrid *gr = dynamic_cast<LazzyGrid*>(index);
	if(gr==NULL) {	
	    cout<<"bad index\n"; 
	    return uniform; 
	}
	ndCell = gr->getClosestNDTCell(pt);
    } else {
	ndCell = tr->getClosestNDTCell(pt);
    }
    if(ndCell == NULL) return uniform;

    double prob = ndCell->getLikelihood(pt);
    prob = (prob<0) ? 0 : prob; //uniform!! TSV
    return prob;
}

//use trilinear interpolation from available immediate neighbors
double NDTMap::getLikelihoodForPointWithInterpolation(pcl::PointXYZ pt) {

    //ATM only for grid map
    //     tll------tlr
    //     /|       /|
    //    / |      / |
    //  tul------tur |    z
    //   | bll----|-blr   ^  y
    //   | /      | /     | /
    //   |/       |/      |/
    //  bul------bur      ---> x
    double uniform=0;//0.00100;
    Cell* cell = NULL;
    NDTCell* ndCell = NULL;
    double cumProb = 0;
    double weight = 0;
    int evals = 1;

    LazzyGrid *gr = dynamic_cast<LazzyGrid*>(index);
    if(gr==NULL) {	
	cout<<"bad index\n"; 
	return uniform; 
    }
    cell = gr->getCellForPoint(pt);
    if(cell == NULL) return uniform;

    
    //get coordinates of cell
    int indXn, indYn, indZn;
    pcl::PointXYZ centerGrid, sizeCell, centerCell;
    int sizeGridX, sizeGridY,sizeGridZ;
    centerCell = cell->getCenter();
    gr->getCenter(centerGrid.x,centerGrid.y,centerGrid.z);
    gr->getGridSize(sizeGridX,sizeGridY,sizeGridZ);
    gr->getCellSize(sizeCell.x,sizeCell.y,sizeCell.z);
    gr->getIndexForPoint(pt,indXn,indYn,indZn);
    
    double x,y,z;
    x = (pt.x - centerCell.x)/sizeCell.x;
    y = (pt.y - centerCell.y)/sizeCell.y;
    z = (pt.z - centerCell.z)/sizeCell.z;
    if(x <0 ) x = 0; 
    if(y <0 ) y = 0;
    if(z <0 ) z = 0;
    if(x >1 ) x = 1;
    if(y >1 ) y = 1;
    if(z >1 ) z = 1;
    
    //bul
    double prob = 0;
    ndCell = dynamic_cast<NDTCell*> (cell);
    if(ndCell != NULL) {
	prob = ndCell->getLikelihood(pt);
	prob = (prob<0) ? uniform : prob;
	weight = (1 - x + 1 - y + 1 - z)/(3.0);
	if(weight < 0) cerr<<weight<<endl; 
	cumProb += prob*weight;
	//cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	evals++;
    }

    //tul
    Cell* c = gr->getCellAt(indXn,indYn,indZn+1);
    if(c != NULL) {
	ndCell = dynamic_cast<NDTCell*> (c);
	if(ndCell != NULL) {
	    prob = ndCell->getLikelihood(pt);
	    prob = (prob<0) ? uniform : prob;
	    weight = (1 - x + 1 - y + z)/(3.0);
	    if(weight < 0) cerr<<weight<<endl; 
	    cumProb += prob*weight;
	    //cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	    evals++;
	}
    }
    //tur
    c = gr->getCellAt(indXn+1,indYn,indZn+1);
    if(c != NULL) {
	ndCell = dynamic_cast<NDTCell*> (c);
	if(ndCell != NULL) {
	    prob = ndCell->getLikelihood(pt);
	    prob = (prob<0) ? uniform : prob;
	    weight = (x + 1-y + z )/(3.0);
	    if(weight < 0) cerr<<weight<<endl; 
	    cumProb += prob*weight;
	    //cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	    evals++;
	}
    }
    //tll
    c = gr->getCellAt(indXn,indYn+1,indZn+1);
    if(c != NULL) {
	ndCell = dynamic_cast<NDTCell*> (c);
	if(ndCell != NULL) {
	    prob = ndCell->getLikelihood(pt);
	    prob = (prob<0) ? uniform : prob;
	    weight = (1-x + y + z )/(3.0);
	    if(weight < 0) cerr<<weight<<endl; 
	    cumProb += prob*weight;
	    //cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	    evals++;
	}
    }
    //tlr
    c = gr->getCellAt(indXn+1,indYn+1,indZn+1);
    if(c != NULL) {
	ndCell = dynamic_cast<NDTCell*> (c);
	if(ndCell != NULL) {
	    prob = ndCell->getLikelihood(pt);
	    prob = (prob<0) ? uniform : prob;
	    weight = (x + y + z )/(3.0);
	    if(weight < 0) cerr<<weight<<endl; 
	    cumProb += prob*weight;
	    //cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	    evals++;
	}
    }
    //bur
    c = gr->getCellAt(indXn+1,indYn,indZn);
    if(c != NULL) {
	ndCell = dynamic_cast<NDTCell*> (c);
	if(ndCell != NULL) {
	    prob = ndCell->getLikelihood(pt);
	    prob = (prob<0) ? uniform : prob;
	    weight = (x + 1-y + 1-z )/(3.0);
	    if(weight < 0) cerr<<weight<<endl; 
	    cumProb += prob*weight;
	    //cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	    evals++;
	}
    }
    //bll
    c = gr->getCellAt(indXn,indYn+1,indZn);
    if(c != NULL) {
	ndCell = dynamic_cast<NDTCell*> (c);
	if(ndCell != NULL) {
	    prob = ndCell->getLikelihood(pt);
	    prob = (prob<0) ? uniform : prob;
	    weight = (1-x + y + 1-z )/(3.0);
	    if(weight < 0) cerr<<weight<<endl; 
	    cumProb += prob*weight;
	    //cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	    evals++;
	}
    }
    //blr
    c = gr->getCellAt(indXn+1,indYn+1,indZn);
    if(c != NULL) {
	ndCell = dynamic_cast<NDTCell*> (c);
	if(ndCell != NULL) {
	    prob = ndCell->getLikelihood(pt);
	    prob = (prob<0) ? uniform : prob;
	    weight = (x + y + 1-z )/(3.0);
	    if(weight < 0) cerr<<weight<<endl; 
	    cumProb += prob*weight;
	    //cout<<"\t"<<weight<<" "<<prob<<" --> "<<cumProb<<endl;
	    evals++;
	}
    }

    //cout<<"== "<<cumProb<<endl;
    return cumProb;
}

std::vector<NDTCell*> NDTMap::getCellsForPoint(const pcl::PointXYZ pt, double radius) {
    
    std::vector<NDTCell*> cells;   
    OctTree* tr = dynamic_cast<OctTree*>(index);
    if(tr==NULL) {
        LazzyGrid *gr = dynamic_cast<LazzyGrid*>(index);
	if(gr==NULL) {	
	    cout<<"bad index\n"; 
	    return cells; 
	}
	cells = gr->getClosestNDTCells(pt,radius);
	return cells;
    }
    //TODO:implement for ocTree
    return cells;
}

bool NDTMap::getCellForPoint(const pcl::PointXYZ &pt, NDTCell *&out_cell) {
    
    OctTree* tr = dynamic_cast<OctTree*>(index);
    if(tr==NULL) {
        LazzyGrid *gr = dynamic_cast<LazzyGrid*>(index);
	if(gr==NULL) {	
	    cout<<"bad index\n"; 
	    return false; 
	}
	out_cell = gr->getClosestNDTCell(pt);
	return true;
    }
    out_cell = tr->getClosestNDTCell(pt);
    return true;
    
    /*
    if(c==NULL) {
	//we have to find the closest leaf cell
	c = tr->getClosestLeafCell(pt);
    }
    if(c==NULL) {
	cout<<"null cell\n"; 
	return false;
    }
    NDTCell* ndCell = dynamic_cast<NDTCell*>(c);
    if(ndCell == NULL) {
	cout<<"not ndt cell\n"; 
	return false;
    }
    if(!ndCell->hasGaussian) {
	Eigen::Vector3d dim;
	ndCell->getDimensions(dim(0),dim(1),dim(2));
	vector<Cell*> neigh;
	index->getNeighbors(pt,20*dim.norm(),neigh);
	double minDist = INT_MAX;
	cout<<"neighs "<<neigh.size()<<endl;
	for(int i=0; i<neigh.size(); i++) {
	    NDTCell *n = dynamic_cast<NDTCell*>(neigh[i]);
	    if(n==NULL) continue;
	    if(n->hasGaussian) {
		double d = lslgeneric::geomDist(n->getCenter(),pt);
		if(d<minDist) {
		    ndCell = n;
		}
	    }
	}
	if(!ndCell->hasGaussian) {
	    cout<<"no gaussian\n";
	    return false;
	}
    }
    out_cell = ndCell;
    return true;
    */
}
	    
void NDTMap::debugToVRML(const char* fname, pcl::PointCloud<pcl::PointXYZ> &pc) {

    FILE* fout = fopen(fname, "w");

    fprintf(fout,"#VRML V2.0 utf8\n");
    this->writeToVRML(fout);
    lslgeneric::writeToVRML(fout,pc,Eigen::Vector3d(1,0,0));

    fprintf(fout,"Shape {\n\tgeometry IndexedLineSet {\n\tcoord Coordinate {\n\t point [\n\t");

    int n_lines = 0;
    pcl::PointXYZ centerCell; 
    for(int i=0; i<pc.points.size(); i++) {
	NDTCell* link;
	if(this->getCellForPoint(pc.points[i], link)) {
	    if(link == NULL) continue;
	    centerCell = link->getCenter();
	    if(link->hasGaussian) {
		centerCell.x = link->getMean()(0);
		centerCell.y = link->getMean()(1);
		centerCell.z = link->getMean()(2);
	    }
	    fprintf(fout,"%lf %lf %lf\n\t%lf %lf %lf\n\t",
		    pc.points[i].x, pc.points[i].y,pc.points[i].z,
		    centerCell.x, centerCell.y, centerCell.z);
	    n_lines++;
	}
    }
    
    fprintf(fout, "]\n\t}\n\tcoordIndex [\n\t");
    for(int i = 0; i<n_lines; i++) {
	fprintf(fout,"%d, %d, -1\n\t",2*i, 2*i+1);	
    }
    fprintf(fout, "]\n}\n}\n");




    fclose(fout);
}
	    
std::vector<NDTCell*> NDTMap::pseudoTransformNDT(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T) {

    std::vector<NDTCell*> ret;  

    vector<Cell*>::iterator it = index->begin();
    while (it != index->end()) {
	NDTCell *cell = dynamic_cast<NDTCell*> (*it);
	if(cell!=NULL) {
	    if(cell->hasGaussian) {
		Eigen::Vector3d mean = cell->getMean();
		Eigen::Matrix3d cov = cell->getCov();
		mean = T*mean;
		cov = T.rotation().transpose()*cov*T.rotation();
		NDTCell* nd = (NDTCell*)cell->clone();
		nd->setMean(mean);
		nd->setCov(cov);
		ret.push_back(nd);
	    }
	} else {
	    ERR("problem casting cell to NDT!\n");
	}
	it++;
    }
    return ret;
}
