#include <vector>
#include <climits>
#include <robot_kinematics.h>

namespace lslgeneric {

template<typename PointT>
bool NDTWavefrontPlanner<PointT>::parametersSet = false;
template<typename PointT>
double NDTWavefrontPlanner<PointT>::MAX_COST;
	    
template<typename PointT>
void NDTWavefrontPlanner<PointT>::setParameters(double _MAX_COST) {
    //defaults
    //NDTWavefrontPlanner::MAX_COST = 10000;
    NDTWavefrontPlanner<PointT>::MAX_COST = _MAX_COST;

    parametersSet = true;
}

/////////constructors//////////
///default constructor, sets pointers to NULL
template<typename PointT>
NDTWavefrontPlanner<PointT>::NDTWavefrontPlanner() {
    myMap = NULL;
    myKin = NULL;
    isValid = true;
    
    if(!parametersSet) {
	printf("using default config\n");
	setParameters(NULL);
    }
}

///parametrized constructor
///@param _myMap an NDT map pointer. this can be updated outside the planner class
///@param _myKin robot kinematics pointer, used to determine dimensions and max roll/pitch constraints
template<typename PointT>
NDTWavefrontPlanner<PointT>::NDTWavefrontPlanner(NDTMap<PointT>* _myMap, RobotKinematics * _myKin) {
    myMap = _myMap;
    myKin = _myKin;
    isValid = true;
    
    if(!parametersSet) {
	printf("using default config\n");
	setParameters(NULL);
    }
}

///////get-set methods////////
///getter for NDT map
template<typename PointT>
NDTMap<PointT>* NDTWavefrontPlanner<PointT>::getMyMap() const {
    return myMap;
}

///change the underlying NDT map pointer
template<typename PointT>
void NDTWavefrontPlanner<PointT>::setMyMap(NDTMap<PointT> *_myMap) {
    myMap = _myMap;
}

///getter for kinematics
template<typename PointT>
RobotKinematics* NDTWavefrontPlanner<PointT>::getMyKinematics() const {
    return myKin;
}

///change teh underlying kinematic model
template<typename PointT>
void NDTWavefrontPlanner<PointT>::setMyKinematics(RobotKinematics *_myKin) {
    myKin = _myKin;
}

//////////////core//////////////
///main method of path planner, uses an iterative cell update
///@param start initial position
///@param goal final position
///currently just the potential map is computed, in the future a vector of poses
///will describe the path TODO return the path

template<typename PointT>
void NDTWavefrontPlanner<PointT>::planPath(const Pose3 start, const Pose3 goal) {

    if(myMap == NULL || myKin == NULL) {
	printf("please set ndt map and robot kinematics before planning a path\n");
	isValid = false;
	return;
    }

    SpatialIndex<PointT> *si = myMap->getMyIndex();
    typename std::vector<Cell<PointT>*>::iterator it = si->begin();
    while (it!=si->end()) {
	//set costs of all cells to inf
	NDTCell<PointT> *c = dynamic_cast<NDTCell<PointT>*> (*it);
	c->cost = INT_MAX;
	++it;
    }

    pcl::PointXYZ center;
    center.x = goal.pos(0); center.y = goal.pos(1); center.z = goal.pos(2);
    //find cell with goal position
    Cell<PointT> *tmp = si->getCellForPoint(center);
    if(tmp == NULL) {
	printf("goal point is not in map!\n");
	isValid = false;
	return;
    }
    goalCell = dynamic_cast<NDTCell<PointT>*> (tmp);
    if(goalCell == NULL) {
	printf("goal cell is not an NDT cell!\n");
	isValid = false;
	return;
    } 
    if(!goalCell->hasGaussian_) {
	printf("goal cell has no gaussians!\n");
	//try to find a proper cell within the robot radius!
	double cellSize = myKin->robotRadius3d(); //2* cell span in xy
	NDTCell<PointT>* gc = findClosestProper(goalCell, cellSize);
	if(gc == NULL) {
	    isValid = false;
	    return;
	}
	if(!gc->hasGaussian_ ) {
	    isValid = false;
	    return;
	}
	goalCell = gc;
	printf("moved goal to (%lf,%lf,%lf)\n",gc->getCenter().x,
		gc->getCenter().y,gc->getCenter().z);
    }

    center.x = start.pos(0); center.y = start.pos(1); center.z = start.pos(2);
    ///find cell with goal position
    tmp = si->getCellForPoint(center);
    if(tmp == NULL) {
	printf("start point is not in map!\n");
	isValid = false;
	return;
    }
    startCell = dynamic_cast<NDTCell<PointT>*> (tmp);
    if(startCell == NULL) {
	printf("start cell is not an NDT cell!\n");
	isValid = false;
	return;
    }
    if(!startCell->hasGaussian_) {
	printf("start cell has no gaussians!\n");
	//try to find a proper cell within the robot radius!
	double cellSize = myKin->robotRadius3d(); //2* cell span in xy
	NDTCell<PointT>* sc = findClosestProper(goalCell, cellSize);
	if(sc == NULL) {
	    isValid = false;
	    return;
	}
	if(! sc->hasGaussian_ ) {
	    isValid = false;
	    return;
	}
	startCell = sc;
	printf("moved start to (%lf,%lf,%lf)\n",sc->getCenter().x,
		sc->getCenter().y,sc->getCenter().z);
    } 

    //set it's cost to 0
    goalCell->cost = 0;
    //initialize Q
    std::vector<NDTCell<PointT>*> Q;
    Q.push_back(goalCell);
    int itr =0;
    //while there are active cells:
    while(Q.size() > 0) {
	NDTCell<PointT> *cur = Q.front();
	Eigen::Vector3d myMean = cur->getMean();
	if(!isCollision(cur)) {
	    computeAccessibleNeighbors(cur);
	    for(unsigned int i =0; i<neighbors.size(); i++) {
		Eigen::Vector3d m = neighbors[i]->getMean() - myMean;
		double toAdd = m.norm() + 1; 
		if(neighbors[i]->cost > cur->cost+toAdd) {
		    neighbors[i]->cost = cur->cost+toAdd;
		    Q.push_back(neighbors[i]);
		}
	    }
	    neighbors.clear();
	} else {
	    cur->cost = -1;
	}
	Q.erase(Q.begin());
	itr++;
	if(itr>MAX_COST) break;
    }

    if(goalCell->cost < 0 || startCell->cost < 0) {
	printf("goal unreachable (<0) \n");
	return;
    }
    if(goalCell->cost > MAX_COST || startCell->cost > MAX_COST) {
	printf("goal unreachable (cost = inf) \n");
	return;
    }
    //now follow gradient to get path
    followGradient(); 
}

template<typename PointT>
void NDTWavefrontPlanner<PointT>::followGradient() {
    
    NDTCell<PointT> *current = startCell;
    Pose3 ps;
    pcl::PointXYZ pt, nextPt, ndPt;
    NDTCell<PointT> *ndCell, *next; 
    double sx,sy,sz;
    Eigen::Vector3d cMean;

    std::vector<Cell<PointT>*> tmpNeighbors; 
    SpatialIndex<PointT> *si = myMap->getMyIndex();
    //double radius = myKin->robotRadius3d();
    double minCost = INT_MAX;
    
    while(current != goalCell) {
	minCost = INT_MAX;
	next = NULL;
	pt = current->getCenter();
	cMean = current->getMean();
//	ps = Pose3(pt->x(),pt->y(),pt->z(),0);
	ps = Pose3(cMean(0),cMean(1),cMean(2),0);
	path.push_back(ps);
	tmpNeighbors.clear();
	
	current->getDimensions(sx,sy,sz);
	double cellSize = 2*sqrt(sx*sx+sy*sy); //2* cell span in xy;//
	if(cellSize < 2*myKin->robotRadius3d()) {
	    cellSize = 2*myKin->robotRadius3d();
	}
//	cout<<"following "<<cellSize<<endl;

	si->getNeighbors(current->getCenter(), cellSize, tmpNeighbors);
	
//	cout<<"check "<<tmpNeighbors.size()<<" neigh | ";
	for(unsigned int i=0; i<tmpNeighbors.size(); i++) {
	    ndCell = dynamic_cast<NDTCell<PointT>*> (tmpNeighbors[i]);
	    //if(!transitPossible(ndCell,current)) continue;
	    if(ndCell->cost < minCost && ndCell->cost >= 0) {
		minCost = ndCell->cost;
		next = ndCell;
	    }
	    
	    if(ndCell->cost == minCost && ndCell->cost >= 0 ) {
		if(next != NULL) {
		    nextPt = next->getCenter();
		    ndPt = ndCell->getCenter();
		    if(geomDist(pt,nextPt) < geomDist(pt,ndPt) ) {
			continue;
		    }
		}
		minCost = ndCell->cost;
		next = ndCell;
	    }
	}

	if(next == NULL) {
	    printf("follow gradient failed, null next cell\n");
	    isValid = false;
	    path.clear();
	    break;
	}
	if(next == current) {
	    printf("follow gradient failed, no options\n");
	    isValid = false;
	    path.clear();
	    break;
	}
        if(next->cost > current->cost){
	    printf("follow gradient failed, next cell too expensive\n");
	    isValid = false;
	    path.clear();
	    break;
	}
	//cout<<current->cost<<" -> "<<next->cost<<endl;
	current = next;
    }
    
    cMean = current->getMean();
    ps = Pose3(cMean(0),cMean(1),cMean(2),0);
    path.push_back(ps);

    if(isValid) std::cout<<"OK\n";
}

///internal planner function. checks if a configuration is collision-free/traversable
template<typename PointT>
bool NDTWavefrontPlanner<PointT>::isCollision(NDTCell<PointT>* cur) {

    if(cur->getClass() != NDTCell<PointT>::HORIZONTAL &&    
       cur->getClass() != NDTCell<PointT>::INCLINED ) {
	return true;
    }
    if(!cur->hasGaussian_) return true;

    std::vector<Cell<PointT>*> tmpNeighbors; 
    neighbors.clear();
    SpatialIndex<PointT> *si = myMap->getMyIndex();
    pcl::PointXYZ centerCur  = cur->getCenter();
    Eigen::Vector3d meanCur = cur->getMean();
    Eigen::Vector3d mean;
    pcl::PointXYZ center;
    Eigen::Vector3d dimensions;
    NDTCell<PointT> *nd;
    
    Eigen::Vector3d e3;
    e3<<0,0,1;
    //TODO this is the extra stuff about covering the cells properly
   /* Eigen::Vector3d myEvals = cur->getEvals();
    double sx,sy,sz;
    cur->getDimensions(sx,sy,sz);
    if(myEvals(0)*myEvals(1)*myEvals(2) < sx*sy/40 ) {
	cout<<"buahaha\n";
	return true;
    }
    */

    double angle,len;
    double THR = 2*cos(M_PI/2 - myKin->getInclConstraint()(0));

    double radius = myKin->robotRadius3d();
    double pradius = myKin->robotRadius2d(); 
    //double radiusSquared = radius*radius;
    double height = centerCur.z + (myKin->getRobotSize())(2);
    double cellMin=0;
    //cout<<"rad: "<<radius<<endl;

    si->getNeighbors(cur->getCenter(), radius, tmpNeighbors);

    //check for points within robot boundary
    //cout<<tmpNeighbors.size()<<endl;
    for(unsigned int i=0; i<tmpNeighbors.size(); i++) {
       nd = dynamic_cast<NDTCell<PointT>*> (tmpNeighbors[i]);

       if(!nd->hasGaussian_) {
	   continue;
       }

       center = nd->getCenter();
       nd->getDimensions(dimensions(0),dimensions(1),dimensions(2));
       
       //collision from above occurs?
       cellMin = center.z - dimensions(2);
       //cell is above us
       if(centerCur.z < center.z) {
	   //our height is more then the bottom of the cell
	   if(height > cellMin) {
	       //check if the offending cell's projection is inside robot 2dradius
	       double pdist = sqrt(pow(center.x-centerCur.x,2) +
		       pow(center.y-centerCur.y,2));
	       if(pdist < pradius) {
		   //cout<<endl;
		   //final checks:
		   //get the vector between this cell and the neighbour mean
		   
		   Eigen::Vector3d testVector;
		   testVector<<centerCur.x,centerCur.y,meanCur(2);
		   testVector-=nd->getMean();

		   angle = testVector.dot(e3);
		   len = e3.norm() * testVector.norm();
		   if(len < 10e-10) len = 10e-10;
		   angle = angle/len;

		   //cout<<"angle "<<fabsf(angle)<<" thr "<<THR<<endl; 
		   //if the angle of the vector with vertical axis < myKin roll constraint,
		   //then the cell is part of support plane and not in collision
		   if(fabsf(angle) > THR) {

		       //otherwise, we check if the distance is bigger then radius
		       if(testVector.norm() < radius) {
			   //cout<<"mean diff length: "<<testVector.length()<<" radius "<<radius<<endl;
			  // cout<<"collision - closer than radius"<<endl;
			   return true;
		       }

		       //scale to get the part that has to be outside the ellipse
		       Eigen::LLT<Eigen::Matrix3d> lltOfCov = nd->getCov().llt();
		       testVector *= (testVector.norm() - radius)/testVector.norm();
		       Eigen::Vector3d tmp = lltOfCov.solve(testVector);

		       double mahalanobis = testVector.dot(tmp);

		       //cout<<"Mahalanobis "<<mahalanobis<<" radSqr "<<radiusSquared<<endl;
		       //if(fabsf(mahalanobis) < radiusSquared) {
		       if(mahalanobis < 1 ) {
			 //  cout<<"collision, too close to ellipse\n";
			   return true;
		       } else {
			 //  cout<<"no collision\n";
		       }
		   } else {
		       if(nd->getClass() != NDTCell<PointT>::HORIZONTAL &&
			       nd->getClass() != NDTCell<PointT>::INCLINED) {
			  //  cout<<"unstable support plane, collision\n";
			    return true;
		       }
		     //  cout<<"part of support plane, not a collision\n";
		   }
	       }
	   }
       } 
       neighbors.push_back(nd);
    }
        //cout<<neighbors.size()<<endl; 
        
    return false;    
}
	
template<typename PointT>
 std::vector<NDTCell<PointT>*> NDTWavefrontPlanner<PointT>::computeAccessibleNeighbors(NDTCell<PointT> *cur) {
     
    double sx,sy,sz;
    cur->getDimensions(sx,sy,sz);
    double cellSize = 2*sqrt(sx*sx+sy*sy); //2* cell span in xy
    std::vector<Cell<PointT>*> tmpNeighbors; 
    NDTCell<PointT>* nd = NULL;
    neighbors.clear();
    
    SpatialIndex<PointT> *si = myMap->getMyIndex();
    si->getNeighbors(cur->getCenter(), cellSize, tmpNeighbors);
    
    for(unsigned int i=0; i<tmpNeighbors.size(); i++) {
	nd = dynamic_cast<NDTCell<PointT>*>(tmpNeighbors[i]);
	if(transitPossible(cur,nd)) {
	    neighbors.push_back(nd);
	}
    }
    //cout<<"new neigh = "<<neighbors.size()<<endl;
    return neighbors;
}

template<typename PointT>
bool NDTWavefrontPlanner<PointT>::transitPossible(NDTCell<PointT>* from, NDTCell<PointT>* to) {

    if (!(to->hasGaussian_&&from->hasGaussian_)) {
	return false;
    }
    double COS_ANGLE_THRESHOLD = 1-cos(M_PI/2 - myKin->getInclConstraint()(0));
    if(from->getClass()!=NDTCell<PointT>::HORIZONTAL && from->getClass()!=NDTCell<PointT>::INCLINED) {
	return false;
    }
    if(to->getClass()!=NDTCell<PointT>::HORIZONTAL && to->getClass()!=NDTCell<PointT>::INCLINED) {
	return false;
    }
   
    pcl::PointXYZ centerFrom, centerTo;
    centerFrom	= from->getCenter();
    centerTo	= to->getCenter();
   /* 
    printf("comparing cells at (%lf,%lf,%lf) and (%lf,%lf,%lf)\n",
	   centerFrom.x, centerFrom.y, centerFrom.z,
	   centerTo.x, centerTo.y, centerTo.z); 
    */

    Eigen::Vector3d meanTo, meanFrom;
    meanFrom= from->getMean();
    meanTo  = to->getMean();

    meanFrom -= meanTo;
    
    Eigen::Vector3d dFrom, dTo;

    from->getDimensions(dFrom(0),dFrom(1),dFrom(2));
    to->getDimensions(dTo(0),dTo(1),dTo(2));
    double halfdiagonalFrom = dFrom.norm();//sqrt(8);
    double halfdiagonalTo = dTo.norm();//sqrt(8);
    
    //immediate neighbor -> dist between centers <=sum of half diagonals 
    //if(centerFrom->geomDist(centerTo) > halfdiagonalFrom+halfdiagonalTo) {
    if(meanFrom.norm() > halfdiagonalFrom+halfdiagonalTo) {
	//printf("too far from each other %lf > %lf\n",meanFrom.norm(),
	//	halfdiagonalFrom+halfdiagonalTo);
	return false;
    }

    Eigen::Matrix3d evecsTo, evecsFrom;
    Eigen::Vector3d ev3From, ev3To;
    Eigen::Vector3d e3;
    e3<<0,0,1;

    //angle between vertical axes < Thresh
    evecsFrom	= from->getEvecs();
    evecsTo	= to->getEvecs();
  /* 
    cout<<"to   "<<evecsTo<<endl;
    cout<<"evals"<<to->getEvals()<<endl;
    cout<<"from "<<evecsFrom<<endl;
    cout<<"evals"<<from->getEvals()<<endl;
  */ 
    ev3From = evecsFrom.col(from->getEvals().minCoeff());
    ev3To   = evecsTo.col(to->getEvals().minCoeff());
    
   // cout<<ev3From<<endl<<ev3To<<endl;

    double angle = ev3From.dot(ev3To);
    double len = ev3From.norm() * ev3To.norm();
    if(len < 10e-10) len = 10e-10;
    angle = angle/len;


    if(fabsf(angle) < COS_ANGLE_THRESHOLD) {
	//printf("angle between normal directions too steep %lf < %lf\n",
	//	fabsf(angle), COS_ANGLE_THRESHOLD);
	return false;
    }

    //the cell we are transfering to is not steeper than allowed?
    angle = ev3To.dot(e3);
    len = ev3To.norm();
    if(len < 10e-10) len = 10e-10;
    angle = angle/len;
    //angle should be near M_PI/2
    double deg = myKin->getInclConstraint()(0) ;//15*M_PI/180;
    if(acos(angle) < (M_PI - deg) &&  acos(angle) > (deg) ) { 
	//printf("our cell is too steep %lf\n",
	//	acos(angle));
	return false;
    }

    //mean-tomean vector should be nearly horizontal as well
    //meanFrom -= meanTo;
    angle = meanFrom.dot(e3);
    len = meanFrom.norm();
    if(len < 10e-10) len = 10e-10;
    angle = angle/len;
    //if(acos(angle) < (M_PI - deg) &&  acos(angle) > (deg) ) { 
    if(acos(angle) < (M_PI/2 - deg) ||  acos(angle) > (M_PI/2 + deg) ) { 
	//cout<<"meandiff "<<meanFrom.transpose()<<endl;
	//printf("mean to mean is too steep %lf\n",
	//	acos(angle));
	return false;
    }
    ////

    return true; 
}

/// searches the index for the closest planar cell within robot radius
template<typename PointT>
NDTCell<PointT>* NDTWavefrontPlanner<PointT>::findClosestProper(NDTCell<PointT> *cell, double cellSize) {
    
    double mindist = INT_MAX;
    std::vector<Cell<PointT>*> tmpNeighbors; 
    NDTCell<PointT> *nd = NULL, *closest = NULL;
    
    SpatialIndex<PointT> *si = myMap->getMyIndex();
    si->getNeighbors(cell->getCenter(), cellSize, tmpNeighbors);
    
    for(unsigned int i=0; i<tmpNeighbors.size(); i++) {
	nd = dynamic_cast<NDTCell<PointT>*>(tmpNeighbors[i]);
	if(nd == NULL) continue;
	if(!nd->hasGaussian_) continue;
	if(nd->getClass() != NDTCell<PointT>::HORIZONTAL) continue;
	double dist = geomDist(nd->getCenter(),cell->getCenter());
	if(dist < mindist) {
	    mindist = dist;
	    closest = nd;
//	    cout<<"dist to closest: "<<dist<<" center "<<nd->getCenter()<<endl;
	}
    }
    return closest;
}

///////////output/////////////
template<typename PointT>
void NDTWavefrontPlanner<PointT>::writeToVRML(const char* filename) {
    if(filename == NULL) {
	printf("problem outputing to vrml\n");
	return;
    }
    FILE *fout = fopen(filename,"w");
    if(fout == NULL) {
	printf("problem outputing to vrml\n");
	return;
    }
    fprintf(fout,"#VRML V2.0 utf8\n");
    this->writeToVRML(fout);
    fclose(fout);

}

template<typename PointT>
void NDTWavefrontPlanner<PointT>::writeToVRML(FILE* fout) {
    bool bOctMap = false;
    if(fout == NULL) {
	printf("problem outputing to vrml\n");
	return;
    }

    if(myMap!=NULL) {
	//myMap->writeToVRML(fout,true);
	//here duplicate the code from NDTMap<PointT>, but use the cost values to get the colors!

	SpatialIndex<PointT> *index = myMap->getMyIndex();
	typename std::vector<Cell<PointT>*>::iterator it = index->begin();
	int i = 0;
	while (it != index->end()) {
	    i++;
	    //if(i%2) continue;
	    NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
	    double xs,ys,zs;
	    if(cell!=NULL) {
		if(cell->hasGaussian_ && cell->cost<MAX_COST && cell->cost >=0) {
		    cell->writeToVRML(fout);
		    /*
		    cell->getDimensions(xs,ys,zs);
		    double c = 1-cell->cost/650;
		    if(c<0) c = 0;
		    Vector3 color(c,c,c);
		    switch(cell->getClass()) {
			case NDTCell<PointT>::HORIZONTAL :
			    color = Vector3(0,1,0);
			    break;	
			case NDTCell<PointT>::INCLINED :
			    color = Vector3(0,0,1);
			    break;	
			case NDTCell<PointT>::VERTICAL :
			    color = Vector3(1,0,0);
			    break;	
			case NDTCell<PointT>::ROUGH :
			    color = Vector3(0,1,1);
			    break;
			default:
			    color = Vector3(1,1,1);	    
		    }
		
		    if(bOctMap) {
			((OctCell<PointT>*)cell)->writeToVRML(fout,color); 
		    } else {
			Ellipsoid el (cell->getCov(), cell->getMean(), xs, &color);
			el.writeToVRML(fout);
		    }
		    */
		}
		    
	    } else {
		//printf("problem casting cell to NDT!\n");
	    }
	    it++;
	}
    }

    if(!isValid) {
	printf("problem outputing to vrml, plan is not valid\n");
	return;
    }

    pcl::PointXYZ pt = startCell->getCenter();
    Pose3 ps(pt.x,pt.y,pt.z,0); 
    ps.writeToVRML(fout);
    pt = goalCell->getCenter();
    ps = Pose3(pt.x,pt.y,pt.z,0); 
    ps.writeToVRML(fout);

    //indexed line set for the path ...

    if(path.size() == 0) return;

    fprintf(fout,"Shape {\n\tgeometry IndexedLineSet {\n\t\tcoord \
	    Coordinate {\n\t\tpoint [\n");

    for(unsigned int i=0; i<path.size(); i++) {
	fprintf(fout,"%lf %lf %lf\n", path[i].x(), path[i].y(), path[i].z()+1);
    }

    fprintf(fout,"]\n}\ncolor Color {\n\t color [\n");

    for(unsigned int i=0; i<path.size(); i++) {
	fprintf(fout,"1 0 0\n"); 
    }

    fprintf(fout,"]\n}\n coordIndex [\n");

    for(unsigned int i=0; i<path.size(); i++) {
	fprintf(fout,"%d ",i);
    }

    fprintf(fout,"-1 \n]\n}\n}");

}
};
