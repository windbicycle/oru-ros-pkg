#include <NDTWavefrontPlanner.hh>
#include <NDTMap.hh>
#include <NDTCell.hh>
#include <SpatialIndex.hh>
#include <RobotKinematics.hh>
#include <Debug.hh>
//#define DBG_LVL 5
#include <vector>
#include <climits>
using namespace lslgeneric;
using namespace std;

bool NDTWavefrontPlanner::parametersSet = false;
double NDTWavefrontPlanner::MAX_COST;
	    
void NDTWavefrontPlanner::setParameters(double _MAX_COST) {
    //defaults
    //NDTWavefrontPlanner::MAX_COST = 10000;
    NDTWavefrontPlanner::MAX_COST = _MAX_COST;

    parametersSet = true;
}

/////////constructors//////////
///default constructor, sets pointers to NULL
NDTWavefrontPlanner::NDTWavefrontPlanner() {
    myMap = NULL;
    myKin = NULL;
    isValid = true;
    
    if(!parametersSet) {
	DBG(1,"using default config\n");
	setParameters(NULL);
    }
}

///parametrized constructor
///@param _myMap an NDT map pointer. this can be updated outside the planner class
///@param _myKin robot kinematics pointer, used to determine dimensions and max roll/pitch constraints
NDTWavefrontPlanner::NDTWavefrontPlanner(NDTMap* _myMap, RobotKinematics * _myKin) {
    myMap = _myMap;
    myKin = _myKin;
    isValid = true;
    
    if(!parametersSet) {
	DBG(1,"using default config\n");
	setParameters(NULL);
    }
}

///////get-set methods////////
///getter for NDT map
NDTMap* NDTWavefrontPlanner::getMyMap() const {
    return myMap;
}

///change the underlying NDT map pointer
void NDTWavefrontPlanner::setMyMap(NDTMap *_myMap) {
    myMap = _myMap;
}

///getter for kinematics
RobotKinematics* NDTWavefrontPlanner::getMyKinematics() const {
    return myKin;
}

///change teh underlying kinematic model
void NDTWavefrontPlanner::setMyKinematics(RobotKinematics *_myKin) {
    myKin = _myKin;
}

//////////////core//////////////
///main method of path planner, uses an iterative cell update
///@param start initial position
///@param goal final position
///currently just the potential map is computed, in the future a vector of poses
///will describe the path TODO return the path

void NDTWavefrontPlanner::planPath(const Pose3 start, const Pose3 goal) {

    if(myMap == NULL || myKin == NULL) {
	ERR("please set ndt map and robot kinematics before planning a path\n");
	isValid = false;
	return;
    }

    SpatialIndex *si = myMap->getMyIndex();
    vector<Cell*>::iterator it = si->begin();
    while (it!=si->end()) {
	//set costs of all cells to inf
	NDTCell *c = dynamic_cast<NDTCell*> (*it);
	c->cost = INT_MAX;
	++it;
    }

    pcl::PointXYZ center;
    center.x = goal.pos(0); center.y = goal.pos(1); center.z = goal.pos(2);
    //find cell with goal position
    Cell *tmp = si->getCellForPoint(center);
    if(tmp == NULL) {
	DBG(0,"goal point is not in map!\n");
	isValid = false;
	return;
    }
    goalCell = dynamic_cast<NDTCell*> (tmp);
    if(goalCell == NULL) {
	DBG(0,"goal cell is not an NDT cell!\n");
	isValid = false;
	return;
    } 
    if(!goalCell->hasGaussian) {
	DBG(0,"goal cell has no gaussians!\n");
	//try to find a proper cell within the robot radius!
	double cellSize = myKin->robotRadius3d(); //2* cell span in xy
	NDTCell* gc = findClosestProper(goalCell, cellSize);
	if(gc == NULL) {
	    isValid = false;
	    return;
	}
	if(!gc->hasGaussian ) {
	    isValid = false;
	    return;
	}
	goalCell = gc;
	DBG(0,"moved goal to (%lf,%lf,%lf)\n",gc->getCenter().x,
		gc->getCenter().y,gc->getCenter().z);
    }

    center.x = start.pos(0); center.y = start.pos(1); center.z = start.pos(2);
    ///find cell with goal position
    tmp = si->getCellForPoint(center);
    if(tmp == NULL) {
	DBG(0,"start point is not in map!\n");
	isValid = false;
	return;
    }
    startCell = dynamic_cast<NDTCell*> (tmp);
    if(startCell == NULL) {
	DBG(0,"start cell is not an NDT cell!\n");
	isValid = false;
	return;
    }
    if(!startCell->hasGaussian) {
	DBG(0,"start cell has no gaussians!\n");
	//try to find a proper cell within the robot radius!
	double cellSize = myKin->robotRadius3d(); //2* cell span in xy
	NDTCell* sc = findClosestProper(goalCell, cellSize);
	if(sc == NULL) {
	    isValid = false;
	    return;
	}
	if(! sc->hasGaussian ) {
	    isValid = false;
	    return;
	}
	startCell = sc;
	DBG(0,"moved start to (%lf,%lf,%lf)\n",sc->getCenter().x,
		sc->getCenter().y,sc->getCenter().z);
    } 

    //set it's cost to 0
    goalCell->cost = 0;
    //initialize Q
    vector<NDTCell*> Q;
    Q.push_back(goalCell);
    int itr =0;
    //while there are active cells:
    while(Q.size() > 0) {
	NDTCell *cur = Q.front();
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
	DBG(0,"goal unreachable (<0) \n");
	return;
    }
    if(goalCell->cost > MAX_COST || startCell->cost > MAX_COST) {
	DBG(0,"goal unreachable (cost = inf) \n");
	return;
    }
    //now follow gradient to get path
    followGradient(); 
}

void NDTWavefrontPlanner::followGradient() {
    
    NDTCell *current = startCell;
    Pose3 ps;
    pcl::PointXYZ pt, nextPt, ndPt;
    NDTCell *ndCell, *next; 
    double sx,sy,sz;
    Eigen::Vector3d cMean;

    vector<Cell*> tmpNeighbors; 
    SpatialIndex *si = myMap->getMyIndex();
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
	    ndCell = dynamic_cast<NDTCell*> (tmpNeighbors[i]);
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
	    DBG(0,"follow gradient failed, null next cell\n");
	    isValid = false;
	    path.clear();
	    break;
	}
	if(next == current) {
	    DBG(0,"follow gradient failed, no options\n");
	    isValid = false;
	    path.clear();
	    break;
	}
        if(next->cost > current->cost){
	    DBG(0,"follow gradient failed, next cell too expensive\n");
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

    if(isValid) cout<<"OK\n";
}

///internal planner function. checks if a configuration is collision-free/traversable
bool NDTWavefrontPlanner::isCollision(NDTCell* cur) {

    if(cur->getClass() != NDTCell::HORIZONTAL &&    
       cur->getClass() != NDTCell::INCLINED ) {
	return true;
    }
    if(!cur->hasGaussian) return true;

    vector<Cell*> tmpNeighbors; 
    neighbors.clear();
    SpatialIndex *si = myMap->getMyIndex();
    pcl::PointXYZ centerCur  = cur->getCenter();
    Eigen::Vector3d meanCur = cur->getMean();
    Eigen::Vector3d mean;
    pcl::PointXYZ center;
    Eigen::Vector3d dimensions;
    NDTCell *nd;
    
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
       nd = dynamic_cast<NDTCell*> (tmpNeighbors[i]);

       if(!nd->hasGaussian) {
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
		       if(nd->getClass() != NDTCell::HORIZONTAL &&
			       nd->getClass() != NDTCell::INCLINED) {
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
	
vector<NDTCell*> NDTWavefrontPlanner::computeAccessibleNeighbors(NDTCell *cur) {
     
    double sx,sy,sz;
    cur->getDimensions(sx,sy,sz);
    double cellSize = 2*sqrt(sx*sx+sy*sy); //2* cell span in xy
    vector<Cell*> tmpNeighbors; 
    NDTCell* nd = NULL;
    neighbors.clear();
    
    SpatialIndex *si = myMap->getMyIndex();
    si->getNeighbors(cur->getCenter(), cellSize, tmpNeighbors);
    
    for(unsigned int i=0; i<tmpNeighbors.size(); i++) {
	nd = dynamic_cast<NDTCell*>(tmpNeighbors[i]);
	if(transitPossible(cur,nd)) {
	    neighbors.push_back(nd);
	}
    }
    //cout<<"new neigh = "<<neighbors.size()<<endl;
    return neighbors;
}

bool NDTWavefrontPlanner::transitPossible(NDTCell* from, NDTCell* to) {

    double COS_ANGLE_THRESHOLD = 1-cos(M_PI/2 - myKin->getInclConstraint()(0));
    if(from->getClass()!=NDTCell::HORIZONTAL && from->getClass()!=NDTCell::INCLINED) {
	return false;
    }
    if(to->getClass()!=NDTCell::HORIZONTAL && to->getClass()!=NDTCell::INCLINED) {
	return false;
    }
   
    pcl::PointXYZ centerFrom, centerTo;
    centerFrom	= from->getCenter();
    centerTo	= to->getCenter();
    
    DBG(0,"comparing cells at (%lf,%lf,%lf) and (%lf,%lf,%lf)\n",
	   centerFrom.x, centerFrom.y, centerFrom.z,
	   centerTo.x, centerTo.y, centerTo.z); 
    

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
	DBG(0,"too far from each other %lf > %lf\n",meanFrom.norm(),
		halfdiagonalFrom+halfdiagonalTo);
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
	DBG(0,"angle between normal directions too steep %lf < %lf\n",
		fabsf(angle), COS_ANGLE_THRESHOLD);
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
	DBG(0,"our cell is too steep %lf\n",
		acos(angle));
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
	DBG(0,"mean to mean is too steep %lf\n",
		acos(angle));
	return false;
    }
    ////

    return true; 
}

/// searches the index for the closest planar cell within robot radius
NDTCell* NDTWavefrontPlanner::findClosestProper(NDTCell *cell, double cellSize) {
    
    double mindist = INT_MAX;
    vector<Cell*> tmpNeighbors; 
    NDTCell *nd = NULL, *closest = NULL;
    
    SpatialIndex *si = myMap->getMyIndex();
    si->getNeighbors(cell->getCenter(), cellSize, tmpNeighbors);
    
    for(unsigned int i=0; i<tmpNeighbors.size(); i++) {
	nd = dynamic_cast<NDTCell*>(tmpNeighbors[i]);
	if(nd == NULL) continue;
	if(!nd->hasGaussian) continue;
	if(nd->getClass() != NDTCell::HORIZONTAL) continue;
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
void NDTWavefrontPlanner::writeToVRML(const char* filename) {
    if(filename == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }
    FILE *fout = fopen(filename,"w");
    if(fout == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }
    fprintf(fout,"#VRML V2.0 utf8\n");
    this->writeToVRML(fout);
    fclose(fout);

}

void NDTWavefrontPlanner::writeToVRML(FILE* fout) {
    bool bOctMap = false;
    if(fout == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }

    if(myMap!=NULL) {
	//myMap->writeToVRML(fout,true);
	//here duplicate the code from NDTMap, but use the cost values to get the colors!

	SpatialIndex *index = myMap->getMyIndex();
	vector<Cell*>::iterator it = index->begin();
	int i = 0;
	while (it != index->end()) {
	    i++;
	    //if(i%2) continue;
	    NDTCell *cell = dynamic_cast<NDTCell*> (*it);
	    double xs,ys,zs;
	    if(cell!=NULL) {
		if(cell->hasGaussian && cell->cost<MAX_COST && cell->cost >=0) {
		    cell->writeToVRML(fout);
		    /*
		    cell->getDimensions(xs,ys,zs);
		    double c = 1-cell->cost/650;
		    if(c<0) c = 0;
		    Vector3 color(c,c,c);
		    switch(cell->getClass()) {
			case NDTCell::HORIZONTAL :
			    color = Vector3(0,1,0);
			    break;	
			case NDTCell::INCLINED :
			    color = Vector3(0,0,1);
			    break;	
			case NDTCell::VERTICAL :
			    color = Vector3(1,0,0);
			    break;	
			case NDTCell::ROUGH :
			    color = Vector3(0,1,1);
			    break;
			default:
			    color = Vector3(1,1,1);	    
		    }
		
		    if(bOctMap) {
			((OctCell*)cell)->writeToVRML(fout,color); 
		    } else {
			Ellipsoid el (cell->getCov(), cell->getMean(), xs, &color);
			el.writeToVRML(fout);
		    }
		    */
		}
		    
	    } else {
		ERR("problem casting cell to NDT!\n");
	    }
	    it++;
	}
    }

    if(!isValid) {
	DBG(0,"problem outputing to vrml, plan is not valid\n");
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
