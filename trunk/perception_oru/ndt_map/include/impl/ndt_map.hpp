#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/features/feature.h>

#include <string>
#include <climits>

#include <oc_tree.h>
#include <lazy_grid.h>
#include <cell_vector.h>

#include <cstring>
#include <cstdio>

#define _JFFVERSION_ "#JFF V0.50"
#define JFFERR(x) std::cerr << x << std::endl; return -1;

namespace lslgeneric {

/** Main implemented method for now. Loads a single point cloud into an NDTMap.
  * \param pc the PointCloud that is to be loaded
  * \note every subsequent call will destroy the previous map!
  */
template<typename PointT>
void NDTMap<PointT>::loadPointCloud(const pcl::PointCloud<PointT> &pc, double range_limit) {
    if(index_ != NULL) {
		//std::cout<<"CLONE INDEX\n";
		SpatialIndex<PointT> *si = index_->clone();
		//cout<<"allocating index\n";
		if(!isFirstLoad_) { 
	    //std::cout<<"deleting old index\n";
	    delete index_;
		}
		isFirstLoad_ = false;
		index_ = si;
	} else {
		//NULL index in constructor, abort!
		//ERR("constructor must specify a non-NULL spatial index\n");
		return;
	}
    
	if(index_ == NULL) {
	//ERR("Problem creating index, unimplemented method\n");
		return;
	}
    

	double maxDist = 0;//, distCeil = 200;

	typename pcl::PointCloud<PointT>::const_iterator it = pc.points.begin();
	Eigen::Vector3d centroid(0,0,0);
	int npts = 0;
	while(it!=pc.points.end()) {
	    Eigen::Vector3d d;
	    if(std::isnan(it->x) ||std::isnan(it->y) ||std::isnan(it->z))
	    {
		it++;
		continue;
	    }
	    d << it->x, it->y, it->z;
	    if(range_limit>0) {
		if(d.norm()>range_limit) {
		    it++;
		    continue;
		}
	    }
	    centroid += d;
	    it++;
	    npts++;
	}

	centroid /= (double)npts;

	//Eigen::Vector4f centroid(0,0,0,0);
	//pcl::compute3DCentroid(pc,centroid);

	//compute distance to furthest point
	it = pc.points.begin();
	while(it!=pc.points.end()) {
	    Eigen::Vector3d d;
	    if(std::isnan(it->x) ||std::isnan(it->y) ||std::isnan(it->z))
	    {
		it++;
		continue;
	    }
	    if(range_limit>0) {
		d << it->x, it->y, it->z;
		if(d.norm()>range_limit) {
		    it++;
		    continue;
		}
	    }
	    d << centroid(0)-it->x, centroid(1)-it->y, centroid(2)-it->z;
	    double dist = d.norm();
	    maxDist = (dist > maxDist) ? dist : maxDist;
	    it++;
	}
	// cout<<"Points = " <<pc.points.size()<<" maxDist = "<<maxDist<<endl;

	NDTCell<PointT> *ptCell = new NDTCell<PointT>();
	index_->setCellType(ptCell);
	delete ptCell;
	index_->setCenter(centroid(0),centroid(1),centroid(2));
	index_->setSize(4*maxDist,4*maxDist,4*maxDist);

	//    ROS_INFO("centroid is %f,%f,%f", centroid(0),centroid(1),centroid(2));
	//    ROS_INFO("maxDist is %lf", maxDist);

	it = pc.points.begin();
	while(it!=pc.points.end()) {
	    Eigen::Vector3d d;
	    if(std::isnan(it->x) ||std::isnan(it->y) ||std::isnan(it->z))
	    {
		it++;
		continue;
	    }
	    if(range_limit>0) {
		d << it->x, it->y, it->z;
		if(d.norm()>range_limit) {
		    it++;
		    continue;
		}
	    }
	    index_->addPoint(*it);
	    it++;
	}

	isFirstLoad_ = false;
}


/**
* Adds a new cloud - NOTE::highly Experimental
* 
*/
template<typename PointT>    
void NDTMap<PointT>::addPointCloud(const pcl::PointCloud<PointT> &pc) {
	if(isFirstLoad_) {
		loadPointCloud(pc);
	}
	
	if(index_ == NULL) {
	//ERR("Problem creating index, unimplemented method\n");
	return;
  }
	typename pcl::PointCloud<PointT>::const_iterator it = pc.points.begin();  
  it = pc.points.begin();
  while(it!=pc.points.end()) {
		if(std::isnan(it->x) ||std::isnan(it->y) ||std::isnan(it->z))
		{
				it++;
				continue;
		}
		index_->addPoint(*it);
		it++;
  }
    isFirstLoad_ = false;
}





template<typename PointT>    
void NDTMap<PointT>::loadPointCloud(const pcl::PointCloud<PointT> &pc, const std::vector<std::vector<size_t> > &indices) {

     loadPointCloud(pc);
     // Specific function related to CellVector
     CellVector<PointT> *cl = dynamic_cast<CellVector<PointT>*>(index_);
     if (cl != NULL)
     {
	  for (size_t i = 0; i < indices.size(); i++)
	  {
	       cl->addCellPoints(pc, indices[i]);
	  }
	  
     }
     else 
     {
	  //ERR("loading point clouds using indices are currently supported in CellVector index_.");
     }
}

template<typename PointT>    
void NDTMap<PointT>::loadDepthImage(const cv::Mat& depthImage, DepthCamera<PointT> &cameraParams) 
{
    pcl::PointCloud<PointT> pc;
    cameraParams.convertDepthImageToPointCloud(depthImage, pc);
    this->loadPointCloud(pc);
}

template<typename PointT>    
pcl::PointCloud<PointT> NDTMap<PointT>::loadDepthImageFeatures(const cv::Mat& depthImage, std::vector<cv::KeyPoint> &keypoints, 
      size_t &supportSize, double maxVar, DepthCamera<PointT> &cameraParams, bool estimateParamsDI, bool nonMean)
{
     std::vector<cv::KeyPoint> good_keypoints;
     Eigen::Vector3d mean;
     PointT mn;
     pcl::PointCloud<PointT> cloudOut;
     CellVector<PointT> *cl = dynamic_cast<CellVector<PointT>*>(index_);
     if(cl==NULL) 
     {
	 std::cerr<<"wrong index type!\n";
	 return cloudOut;
     }
     for (size_t i=0; i<keypoints.size(); i++) 
     {
	 if(!estimateParamsDI)
	 { 
	     pcl::PointCloud<PointT> points;
	     PointT center;
	     cameraParams.computePointsAtIndex(depthImage,keypoints[i],supportSize,points,center);
	     NDTCell<PointT> *ndcell = new NDTCell<PointT>();
	     typename pcl::PointCloud<PointT>::iterator it = points.points.begin();
	     while (it!= points.points.end() ) 
	     {
		 if(std::isnan(it->x) ||std::isnan(it->y) ||std::isnan(it->z))
		 {
		     it++;
		     continue;
		 }
		 ndcell->addPoint(*it);
		 it++;
	     }
	     ndcell->computeGaussian();
	     if(ndcell->hasGaussian_) {
		 Eigen::Vector3d evals = ndcell->getEvals();
		 if(sqrt(evals(2)) < maxVar)
		 {
		   if (nonMean)
		   {
		     if(std::isnan(center.x) ||std::isnan(center.y) ||std::isnan(center.z))
		     {
			 continue;
		     }
		     mn = center;
		   }
		   else
		   {
		     mean = ndcell->getMean();
		     mn.x = mean(0); mn.y = mean(1); mn.z = mean(2);
		   }
		   cloudOut.points.push_back(mn);
		   ndcell->setCenter(mn);
		   cl->addCell(ndcell);
		   good_keypoints.push_back(keypoints[i]);
		 }
	     }
	 }
	 else
	 {
             assert(nonMean = false); // Not implemented / used.
	     Eigen::Vector3d mean;
	     Eigen::Matrix3d cov;
	     cameraParams.computeParamsAtIndex(depthImage,keypoints[i],supportSize,mean,cov);
	     NDTCell<PointT> *ndcell = new NDTCell<PointT>();
	     ndcell->setMean(mean);
	     ndcell->setCov(cov);

	     if(ndcell->hasGaussian_) {
		 Eigen::Vector3d evals = ndcell->getEvals();
		 //std::cout<<evals.transpose()<<std::endl;
		 if(sqrt(evals(2)) < maxVar)
		 {
		     mean = ndcell->getMean();
		     mn.x = mean(0); mn.y = mean(1); mn.z = mean(2);
		     cloudOut.points.push_back(mn);
		     ndcell->setCenter(mn);
		     cl->addCell(ndcell);
		     good_keypoints.push_back(keypoints[i]);
		 }
	     }

	 }
     }

     //TODO
     keypoints = good_keypoints;
     return cloudOut;
}

/** Helper function, computes the  NDTCells
  */
template<typename PointT>    
void NDTMap<PointT>::computeNDTCells() {
    /*
    AdaptiveOctTree *aot = dynamic_cast<AdaptiveOctTree*>(index_);
    if(aot!=NULL) {
	ROS_INFO("post processing points!\n");
	aot->postProcessPoints();
    }
    */
    CellVector<PointT> *cv = dynamic_cast<CellVector<PointT>*>(index_);
   
    typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
	NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
	if(cell!=NULL) {
	    cell->computeGaussian();
	    if (cv!=NULL)
	    {
		 // Set the mean to the cell's centre.
		 Eigen::Vector3d mean = cell->getMean();
		 //cout << "mean : " << mean << std::endl;
		 PointT pt;
		 pt.x = mean[0];
		 pt.y = mean[1];
		 pt.z = mean[2];
		 
		 cell->setCenter(pt);
	    }
	} else {
	    //ERR("problem casting cell to NDT!\n");
	}
	it++;
    }

    LazyGrid<PointT> *lz = dynamic_cast<LazyGrid<PointT>*>(index_);
    if(lz!=NULL) {
	lz->initKDTree();
    }

    CellVector<PointT> *cl = dynamic_cast<CellVector<PointT>*>(index_);
    if(cl!=NULL) {
	cl->initKDTree();
    }
}

/** output method, saves as vrml the oct tree and all the ellipsoids
 */
template<typename PointT>    
void NDTMap<PointT>::writeToVRML(const char* filename) {

    if(filename == NULL) {
	//ERR("problem outputing to vrml\n");
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
	//ERR("problem outputing to vrml\n");
	return;
    }
    fprintf(fout,"#VRML V2.0 utf8\n");
    writeToVRML(fout);
    fclose(fout);
}

/** helper output method
 */
template<typename PointT>    
void NDTMap<PointT>::writeToVRML(FILE* fout) {
    if(fout == NULL) {
	//ERR("problem outputing to vrml\n");
	return;
    }

    //move the ellipsoid stuff to NDTCell
    typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
	NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
	//	double xs,ys,zs;
	if(cell!=NULL) {
	    cell->writeToVRML(fout);
	} else {
	    //	    ERR("problem casting cell to NDT!\n");
	}
	it++;
    }

}

template<typename PointT>    
void NDTMap<PointT>::writeToVRML(FILE* fout, Eigen::Vector3d col) {
    if(fout == NULL) {
	//ERR("problem outputing to vrml\n");
	return;
    }

    //move the ellipsoid stuff to NDTCell
    typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
	NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
	if(cell!=NULL) {
	    cell->writeToVRML(fout,col);
	} else {
	}
	it++;
    }
}


/** output methods for saving the map in the jff format
 */
template<typename PointT>    
int NDTMap<PointT>::writeToJFF(const char* filename) {

    if(filename == NULL) {
		//ERR("problem outputing to jff\n");
		return -1;
    }
    
    FILE * jffout = fopen(filename, "w+b");

    fwrite(_JFFVERSION_, sizeof(char), strlen(_JFFVERSION_), jffout);

	switch(this->getMyIndexInt())
    {
    	case 1:
    		writeCellVectorJFF(jffout);
    		break;
    	case 2:
    		//writeOctTreeJFF(jffout);
    		break;
    	case 3:
    		writeLazyGridJFF(jffout);
    		break;
    	default:
    		//ERR("unknown index type\n");
			return -1;
    }

    fclose(jffout);

    return 0;
}


template<typename PointT>
int NDTMap<PointT>::writeCellVectorJFF(FILE * jffout){
	int indexType[1] = {1};
	fwrite(indexType, sizeof(int), 1, jffout);
	
	// TODO: add CellVector specific stuff
	
	typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
	while (it != index_->end()) {
		NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
		if(cell!=NULL) {
			if(cell->hasGaussian_) {
				// TODO: add index specific content smartly
				if(cell->writeToJFF(jffout) < 0)
		    		return -1;
		    }
		} else {
			// do nothing
		}
		it++;
	}

	return 0;

}


template<typename PointT>
int NDTMap<PointT>::writeOctTreeJFF(FILE * jffout){
	int indexType[1] = {2};
	fwrite(indexType, sizeof(int), 1, jffout);
	
	// TODO: add OctTree specific stuff
	
	typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
		NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
		if(cell!=NULL) {
			if(cell->hasGaussian_) {
				// TODO: add index specific content smartly
				if(cell->writeToJFF(jffout) < 0)
		    		return -1;
		    }
		} else {
			// do nothing
		}
		it++;
	}

	return 0;

}

template<typename PointT>
int NDTMap<PointT>::writeLazyGridJFF(FILE * jffout){
	int indexType[1] = {3};
	fwrite(indexType, sizeof(int), 1, jffout);

	// add LazyGrid specific stuff
	double sizeXmeters, sizeYmeters, sizeZmeters;
	double cellSizeX, cellSizeY, cellSizeZ;
	double centerX, centerY, centerZ;
	LazyGrid<PointT> *ind = dynamic_cast<LazyGrid<PointT>*>(index_);

	ind->getGridSizeInMeters(sizeXmeters, sizeYmeters, sizeZmeters);
	ind->getCellSize(cellSizeX, cellSizeY, cellSizeZ);
	ind->getCenter(centerX, centerY, centerZ);
	
	double lazyGridData[9] = { sizeXmeters, sizeYmeters, sizeZmeters,
	                           cellSizeX,   cellSizeY,   cellSizeZ,
	                           centerX,     centerY,     centerZ };
	
	fwrite(lazyGridData, sizeof(double), 9, jffout);
	
	fwrite(ind->getProtoType(), sizeof(Cell<PointT>), 1, jffout);

	// loop through all active cells
	typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
		NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
		if(cell!=NULL) {
			if(cell->hasGaussian_) {
				if(cell->writeToJFF(jffout) < 0)
		    		return -1;
		    }
		} else {
			// do nothing
		}
		it++;
	}

	return 0;

}

/** method to load NDT maps from .jff files
	USAGE:	create NDTMap with desired index and PointType (index type is
			checked, but Point type is NOT checked) via e.g.

			lslgeneric::NDTMap<pcl::PointXYZ> nd1(
				new lslgeneric::LazyGrid<pcl::PointXYZ>(0.4)); --> (*)

			and then call

			nd1.loadFromJFF("map0027.jff");

			*) use this constructor so index is not initialized and attributes
			   can be set manually
 */
template<typename PointT>    
int NDTMap<PointT>::loadFromJFF(const char* filename) {
	
	FILE * jffin;

	if(filename == NULL) {
		JFFERR("problem outputing to jff");
	}

    jffin = fopen(filename,"r+b");
    
    char versionBuf[16];
    if(fread(&versionBuf, sizeof(char), strlen(_JFFVERSION_), jffin) <= 0){
		JFFERR("reading version failed");
	}
	versionBuf[strlen(_JFFVERSION_)] = '\0';

    int indexType;
    if(fread(&indexType, sizeof(int), 1, jffin) <= 0){
		JFFERR("reading version failed");
	}
    
    if(indexType != this->getMyIndexInt()){
    	switch(indexType){
			case 1:
				std::cerr << "Map uses CellVector\n";
				return -1;
				break;
			case 2:
				std::cerr << "Map uses OctTree\n";
				return -2;
				break;
			case 3:
				std::cerr << "Map uses LazyGrid\n";
				return -3;
				break;
		}
	}

	switch(indexType){
		case 1:{
			CellVector<PointT>* cv = dynamic_cast<CellVector<PointT> * >(index_);
			if(cv->loadFromJFF(jffin) < 0){
				JFFERR("Error loading CellVector");
			}
			break;
		}
		case 2:{
			OctTree<PointT>* tr = dynamic_cast<OctTree<PointT>*>(index_);
			if(tr->loadFromJFF(jffin) < 0){
				JFFERR("Error loading OctTree");
			}
			break;
		}
		case 3:{
			LazyGrid<PointT>* gr = dynamic_cast<LazyGrid<PointT>*>(index_);
			if(gr->loadFromJFF(jffin) < 0){
				JFFERR("Error loading LazyGrid");
			}
			break;
		}
		default:
			JFFERR("error casting index");
	}

	NDTCell<PointT> *ptCell = new NDTCell<PointT>();
	index_->setCellType(ptCell);
	delete ptCell;

    fclose(jffin);

    std::cout << "map loaded successfully " << versionBuf << std::endl;

    // isFirstLoad_ = false;

    return 0;

}


/// returns the current spatial index as a string (debugging function)
template<typename PointT>    
std::string NDTMap<PointT>::getMyIndexStr() const 
{
     CellVector<PointT>* cl = dynamic_cast<CellVector<PointT> * >(index_);
     if(cl!=NULL) {
	  return std::string("CellVector");
      }
      OctTree<PointT>* tr = dynamic_cast<OctTree<PointT>*>(index_);
      if(tr!=NULL) {
	   return std::string("OctTree");
      }
      LazyGrid<PointT> *gr = dynamic_cast<LazyGrid<PointT>*>(index_);
      if(gr!=NULL) {
	   return std::string("LazyGrid<PointT>");
      }	
      
     return std::string("Unknown index type");
}

/// returns the current spatial index as an integer (debugging function)
template<typename PointT>    
int NDTMap<PointT>::getMyIndexInt() const 
{
     CellVector<PointT>* cl = dynamic_cast<CellVector<PointT> * >(index_);
     if(cl!=NULL) {
	  return 1;
      }
      OctTree<PointT>* tr = dynamic_cast<OctTree<PointT>*>(index_);
      if(tr!=NULL) {
	   return 2;
      }
      LazyGrid<PointT> *gr = dynamic_cast<LazyGrid<PointT>*>(index_);
      if(gr!=NULL) {
	   return 3;
      }	
      
     return -1;
}

//computes the *negative log likelihood* of a single observation
template<typename PointT>    
double NDTMap<PointT>::getLikelihoodForPoint(PointT pt) {
     //assert(false);
     double uniform=0.00100;
    NDTCell<PointT>* ndCell = NULL;
    OctTree<PointT>* tr = dynamic_cast<OctTree<PointT>*>(index_);

    if(tr==NULL) { 
	LazyGrid<PointT> *gr = dynamic_cast<LazyGrid<PointT>*>(index_);
	if(gr==NULL) {	
	    //cout<<"bad index - getLikelihoodForPoint\n"; 
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

/*
//use trilinear interpolation from available immediate neighbors
template<typename PointT>    
double NDTMap<PointT>::getLikelihoodForPointWithInterpolation(PointT pt) {

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
    NDTCell<PointT>* ndCell = NULL;
    double cumProb = 0;
    double weight = 0;
    int evals = 1;

    LazyGrid<PointT> *gr = dynamic_cast<LazyGrid<PointT>*>(index_);
    if(gr==NULL) {	
	//cout<<"bad index - getLikelihoodForPointWithInterpolation\n"; 
	return uniform; 
    }
    cell = gr->getCellForPoint(pt);
    if(cell == NULL) return uniform;

    
    //get coordinates of cell
    int indXn, indYn, indZn;
    PointT centerGrid, sizeCell, centerCell;
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
    ndCell = dynamic_cast<NDTCell<PointT>*> (cell);
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
	ndCell = dynamic_cast<NDTCell<PointT>*> (c);
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
	ndCell = dynamic_cast<NDTCell<PointT>*> (c);
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
	ndCell = dynamic_cast<NDTCell<PointT>*> (c);
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
	ndCell = dynamic_cast<NDTCell<PointT>*> (c);
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
	ndCell = dynamic_cast<NDTCell<PointT>*> (c);
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
	ndCell = dynamic_cast<NDTCell<PointT>*> (c);
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
	ndCell = dynamic_cast<NDTCell<PointT>*> (c);
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
*/

template<typename PointT>    
std::vector<NDTCell<PointT>*> NDTMap<PointT>::getCellsForPoint(const PointT pt, double radius) {
     //assert(false);
    std::vector<NDTCell<PointT>*> cells;   
    OctTree<PointT>* tr = dynamic_cast<OctTree<PointT>*>(index_);
    if(tr==NULL) {
        LazyGrid<PointT> *gr = dynamic_cast<LazyGrid<PointT>*>(index_);
	if(gr==NULL) {	
	    //cout<<"bad index - getCellsForPoint\n"; 
	    return cells; 
	}
	cells = gr->getClosestNDTCells(pt,radius);
	return cells;
    }
    //TODO:implement for ocTree
    return cells;
}

template<typename PointT>    
bool NDTMap<PointT>::getCellForPoint(const PointT &pt, NDTCell<PointT> *&out_cell) {
     
#if 0
    OctTree<PointT>* tr = dynamic_cast<OctTree<PointT>*>(index);
    if(tr==NULL) {
        LazyGrid<PointT> *gr = dynamic_cast<LazyGrid<PointT>*>(index);
	if(gr==NULL) {	
	     CellVector *cl = dynamic_cast<CellVector*>(index);
	     if(cl==NULL) {
		  cout<<"bad index - getCellForPoint\n"; 
		  return false; 
	     }
	     out_cell = cl->getClosestNDTCell(pt);
	     return true;
	}
	out_cell = gr->getClosestNDTCell(pt);
	return true;
    }
    out_cell = tr->getClosestNDTCell(pt);
    return true;
#endif
      CellVector<PointT> *cl = dynamic_cast<CellVector<PointT>*>(index_);
      if(cl!=NULL) {
           out_cell = cl->getClosestNDTCell(pt);
	   return true;
      }
      OctTree<PointT>* tr = dynamic_cast<OctTree<PointT>*>(index_);
      if(tr!=NULL) {
	   out_cell = tr->getClosestNDTCell(pt);
	   return true;
      }
      LazyGrid<PointT> *gr = dynamic_cast<LazyGrid<PointT>*>(index_);
      if(gr!=NULL) {
    	out_cell = gr->getClosestNDTCell(pt);
	return true;
      }	
      //cout<<"bad index - getCellForPoint\n"; 
      return false; 

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
	typename SpatialIndex<PointT>::CellVectorItr neigh;
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
	    
template<typename PointT>    
void NDTMap<PointT>::debugToVRML(const char* fname, pcl::PointCloud<PointT> &pc) {

    FILE* fout = fopen(fname, "w");

    fprintf(fout,"#VRML V2.0 utf8\n");
    this->writeToVRML(fout);
    lslgeneric::writeToVRML(fout,pc,Eigen::Vector3d(1,0,0));

    fprintf(fout,"Shape {\n\tgeometry IndexedLineSet {\n\tcoord Coordinate {\n\t point [\n\t");

    int n_lines = 0;
    PointT centerCell; 
    for(size_t i=0; i<pc.points.size(); i++) {
	NDTCell<PointT>* link;
	if(this->getCellForPoint(pc.points[i], link)) {
	    if(link == NULL) continue;
	    centerCell = link->getCenter();
	    if(link->hasGaussian_) {
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
	    
template<typename PointT>    
NDTMap<PointT>* NDTMap<PointT>::pseudoTransformNDTMap(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T) {
   NDTMap<PointT>* map = new NDTMap<PointT>(new CellVector<PointT>());
   CellVector<PointT>* idx = dynamic_cast<CellVector<PointT>*> (map->getMyIndex()); 
   typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();

   while (it != index_->end()) {
       NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
       if(cell!=NULL) {
	   if(cell->hasGaussian_) {
	       Eigen::Vector3d mean = cell->getMean();
	       Eigen::Matrix3d cov = cell->getCov();
	       mean = T*mean;
	       cov = T.rotation().transpose()*cov*T.rotation();
	       NDTCell<PointT>* nd = (NDTCell<PointT>*)cell->clone();
	       nd->setMean(mean);
	       nd->setCov(cov);
	       idx->addNDTCell(nd);
	   }
       } else {
	   //ERR("problem casting cell to NDT!\n");
       }
       it++;
   }
   return map;
}

template<typename PointT>    
std::vector<NDTCell<PointT>*> NDTMap<PointT>::pseudoTransformNDT(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T) {

    std::vector<NDTCell<PointT>*> ret;  
    typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
	NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
	if(cell!=NULL) {
	    if(cell->hasGaussian_) {
		Eigen::Vector3d mean = cell->getMean();
		Eigen::Matrix3d cov = cell->getCov();
		mean = T*mean;
		cov = T.rotation().transpose()*cov*T.rotation();
		NDTCell<PointT>* nd = (NDTCell<PointT>*)cell->clone();
		nd->setMean(mean);
		nd->setCov(cov);
		ret.push_back(nd);
	    }
	} else {
	    //ERR("problem casting cell to NDT!\n");
	}
	it++;
    }
    return ret;
}

template<typename PointT>    
NDTCell<PointT>* 
NDTMap<PointT>::getCellIdx(unsigned int idx)
{
     CellVector<PointT> *cl = dynamic_cast<CellVector<PointT>*>(index_);
     if (cl != NULL)
     {
	  return cl->getCellIdx(idx);
     }
     return NULL;
}
// ???
template<typename PointT>    
//std::vector<lslgeneric::NDTCell<pcl::PointXYZ>*> NDTMap<PointT>::getAllCells(){
std::vector<lslgeneric::NDTCell<PointT>*> NDTMap<PointT>::getAllCells(){

    std::vector<NDTCell<PointT>*> ret;  
    typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
	NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
	if(cell!=NULL) {
	    if(cell->hasGaussian_) {
		NDTCell<PointT>* nd = (NDTCell<PointT>*)cell->copy();
		ret.push_back(nd);
	    }
	} else {
	}
	it++;
    }
    return ret;
}

template<typename PointT>    
int NDTMap<PointT>::numberOfActiveCells()
{
    int ret = 0;
    if(index_ == NULL) return ret;
    typename SpatialIndex<PointT>::CellVectorItr it = index_->begin();
    while (it != index_->end()) {
	NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
	if(cell!=NULL) {
	    if(cell->hasGaussian_) {
		ret++; 
	    }
	} 
	it++;
    }
    return ret;
}

}
