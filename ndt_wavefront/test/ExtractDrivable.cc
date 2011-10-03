#include <NDTWavefrontPlanner.hh>
#include <climits>
#include<vector>
#include <NDTMap.hh>
#include <NDTCell.hh>
#include <SpatialIndex.hh>
#include <RobotKinematics.hh>
#include <Debug.hh>
#include <PointCloudUtils.hh>
#include <OctTree.hh>
#include<LazzyGrid.hh>
//#define DBG_LVL 5

using namespace std;
using namespace lslgeneric;

int main (int argc, char **argv) {

    int MAX_COST = 10000;
    if(argc != 3) {
	cout<<"Usage: "<<argv[0]<<" originalPointCloud outputPointCloud\n";
	return -1;
    }

    pcl::PointCloud<pcl::PointXYZ> in = lslgeneric::readVRML(argv[1]);

    /*
    double RS = 5, NS = 100;
    //add the points under the robot...
    for(double xi=-RS/2; xi<RS/2; xi+=RS/NS) {
	for(double yi=-RS/2; yi<RS/2; yi+=RS/NS) {
	    pcl::PointXYZ pt;
	    pt.x = xi; pt.y = yi; pt.z = -1;
	    in.points.push_back(pt);
	}
    }
    */

    pcl::PointCloud<pcl::PointXYZ> out;

    LazzyGrid prototype(0.5);
    NDTMap *myMap = new NDTMap(&prototype);

    myMap->loadPointCloud(in);
    myMap->computeNDTCells();
    Eigen::Vector3d robotSize, inclinationConstraints;
    robotSize<<0.5,0.5,0.5;
    inclinationConstraints<<15*M_PI/180,15*M_PI/180,2*M_PI;
    lslgeneric::RobotKinematics kin(robotSize,inclinationConstraints);

    NDTWavefrontPlanner planner(myMap,&kin);
    myMap->writeToVRML("/home/tsv/ndt_tmp/wave.wrl");
    
    SpatialIndex *si = myMap->getMyIndex();
    vector<Cell*>::iterator it = si->begin();
    while (it!=si->end()) {
	//set costs of all cells to inf
	NDTCell *c = dynamic_cast<NDTCell*> (*it);
	c->cost = INT_MAX;
	++it;
    }
    
    //find closest point to scan origin
    pcl::PointXYZ center;
    center.x = 0 ; center.y = 0 ; center.z = -1;
    //find cell with goal position
    Cell *tmp = si->getCellForPoint(center);
    if(tmp == NULL) {
	DBG(0,"goal point is not in map!\n");
	//try to find closest cell in front of the robot
	vector<Cell*> closest;
	si->getNeighbors(center, 5, closest);

	double MIN_DIST = INT_MAX;
	Eigen::Vector3d e2(1,0,0);
	for(int i=0; i<closest.size(); i++) {
	    pcl::PointXYZ c = closest[i]->getCenter();
	    Eigen::Vector3d dv(c.x-center.x, c.y-center.y, c.z-center.z);
	    double angle = dv.dot(e2);
	    double len = dv.norm();
	    if(len < 10e-10) len = 10e-10;
	    angle = angle/len;
	    //angle should be near M_PI/2
	    double deg = 15*M_PI/180;
	    if(acos(angle) < (M_PI - deg) &&  acos(angle) > (deg) ) continue;

	    if(len<MIN_DIST) {
		MIN_DIST=len;
		tmp = closest[i];
	    }
	    
	}
    
	if(tmp == NULL) {
	    return -1;
	} else {
	    pcl::PointXYZ t = tmp->getCenter();
	    cout<<"moved point to "<<t.x<<" "<<t.y<<" "<<t.z<<endl;
	}
    }
    NDTCell* goalCell = dynamic_cast<NDTCell*> (tmp);
    if(goalCell == NULL) {
	DBG(0,"goal cell is not an NDT cell!\n");
	return -1;
    } 
    if(!goalCell->hasGaussian) {
	DBG(0,"goal cell has no gaussians!\n");
	//try to find a proper cell within the robot radius!
	NDTCell* gc = planner.findClosestProper(goalCell, 20);
	if(gc == NULL) {
	    return -1;
	}
	if(!gc->hasGaussian ) {
	    return -1;
	}
	goalCell = gc;
	DBG(0,"moved goal to (%lf,%lf,%lf)\n",gc->getCenter().x,
		gc->getCenter().y,gc->getCenter().z);
    }
    
    goalCell->cost = 0;
    vector<NDTCell*> Q;
    Q.push_back(goalCell);
    int itr = 0;

    while(Q.size() > 0) {
	cout<<"itr "<<itr<<endl;
	NDTCell *cur = Q.front();
	Eigen::Vector3d myMean = cur->getMean();
	if(!planner.isCollision(cur)) {
	    vector<NDTCell*> neighbors = planner.computeAccessibleNeighbors(cur);
	    cout<<"neighbors "<<neighbors.size()<<endl;;
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

    FILE *fout = fopen(argv[2],"w");
    fprintf(fout,"#VRML V2.0 utf8\n");

    it = si->begin();
    pcl::PointCloud<pcl::PointXYZ> drivable, occupied, unreachable;
    while (it!=si->end()) {
	//set costs of all cells to inf
	NDTCell *c = dynamic_cast<NDTCell*> (*it);
	if(c->points.size() > 0) {
	    pcl::PointCloud<pcl::PointXYZ> thiscloud;
	    thiscloud.points.insert(thiscloud.points.begin(),
		    c->points.begin(),c->points.end());
	    if(c->cost == -1) {
		occupied+=thiscloud;
	    } 
	    else if(c->cost < MAX_COST) {
		drivable+=thiscloud;
	    } else {
		unreachable+=thiscloud;
	    }
	}
	++it;
    }	
    writeToVRML(fout,occupied,Eigen::Vector3d(1,0,0));
    writeToVRML(fout,drivable,Eigen::Vector3d(0,1,0));
    writeToVRML(fout,unreachable,Eigen::Vector3d(0.6,0.5,0));

    fclose(fout);

    return 0;
}
