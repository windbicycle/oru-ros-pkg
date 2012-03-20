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
#include <Eigen/Eigen>

#include "pcl/kdtree/kdtree_flann.h"

//#define DBG_LVL 5

using namespace std;
using namespace lslgeneric;

int extractDrivable(pcl::PointCloud<pcl::PointXYZ> &in, pcl::PointXYZ center,
		     pcl::PointCloud<pcl::PointXYZ> &drivable,
		     pcl::PointCloud<pcl::PointXYZ> &occupied) {

    //pcl::PointCloud<pcl::PointXYZ> in = global; //lslgeneric::readVRML(argv[1]);
    
   struct timeval tv_start, tv_end;
   gettimeofday(&tv_start,NULL);

   int MAX_COST = 100000;

    //pcl::PointCloud<pcl::PointXYZ> out;

//    LazzyGrid tr(0.5);
    
    lslgeneric::OctTree tr;
    lslgeneric::OctTree::BIG_CELL_SIZE = 2;
    lslgeneric::OctTree::SMALL_CELL_SIZE = 0.5;

    //NDTMap *myMap = new NDTMap(&prototype);
    NDTMap *myMap = new NDTMap(&tr);

    myMap->loadPointCloud(in);
    myMap->computeNDTCells();
    Eigen::Vector3d robotSize, inclinationConstraints;
    robotSize<<1,1,0.5;
    inclinationConstraints<<15*M_PI/180,15*M_PI/180,2*M_PI;
    lslgeneric::RobotKinematics kin(robotSize,inclinationConstraints);

    NDTWavefrontPlanner planner(myMap,&kin);
    myMap->writeToVRML("wave.wrl");
    
    SpatialIndex *si = myMap->getMyIndex();
    vector<Cell*>::iterator it = si->begin();
    while (it!=si->end()) {
	//set costs of all cells to inf
	NDTCell *c = dynamic_cast<NDTCell*> (*it);
	c->cost = INT_MAX;
	++it;
    }
    
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
//	    if(acos(angle) < (M_PI - deg) &&  acos(angle) > (deg) ) continue;
	    if(acos(angle) < (M_PI/2 - deg) ||  acos(angle) > (M_PI/2 + deg) ) continue;

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
//	cout<<"itr "<<itr<<endl;
	NDTCell *cur = Q.front();
	Eigen::Vector3d myMean = cur->getMean();
	if(!planner.isCollision(cur)) {
	    vector<NDTCell*> neighbors = planner.computeAccessibleNeighbors(cur);
//	    cout<<"neighbors "<<neighbors.size()<<endl;;
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

    it = si->begin();
    pcl::PointCloud<pcl::PointXYZ> unreachable;
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
//		if(c->getCenter().z <0 ) { 
		    unreachable+=thiscloud;
//		}
	    }
	}
	++it;
    }
    occupied += unreachable;    
   
   gettimeofday(&tv_end,NULL);
   cout<<" TIME: "<<
                   (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.<<endl;

    return 0; 
}

int main (int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZ> curr, global;
    char currName[500];
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> currPose;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Pr, R;

    if(argc != 2) {
	std::cout<<"Usage: "<<argv[0]<<" configFile\n";
	return -1;
    }

    FILE *fin = fopen(argv[1],"r");
    double xd,yd,zd, x,y,z,w;
    int ts;
    string prefix;
    char *line = NULL;
    size_t len;
    int ctr = 0;

    //get first line
    int n = getline(&line,&len,fin);
    if(n <=0 ) return -1;
    prefix = line;
    *(prefix.rbegin()) = '\0';

    while(getline(&line,&len,fin) > 0) {

	int n = sscanf(line,"%d %lf %lf %lf %lf %lf %lf %lf",
		&ts,&xd,&yd,&zd,&x,&y,&z,&w);
	if(n != 8) {
	    cout<<"wrong format of pose at : "<<line<<endl;
	    break;
	}

	currPose =Eigen::Translation<double,3>(xd,yd,zd)*
	    Eigen::Quaternion<double>(w,x,y,z);
	snprintf(currName,499,"%s%03d.wrl",prefix.c_str(),ts);
	curr = lslgeneric::readVRML(currName);

	lslgeneric::transformPointCloudInPlace(currPose,curr);
	global += curr;

	ctr++;
    }
    fclose (fin);

    //find closest point to scan origin
    pcl::PointXYZ center;
    center.x = 1.5 ; center.y = -0.5 ; center.z = -1.1;
    pcl::PointCloud<pcl::PointXYZ> drivable, occupied;

    if(extractDrivable(global, center, drivable, occupied) < 0) return -1;

    //kdtree on drivable
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pcptr(new pcl::PointCloud<pcl::PointXYZ>());
    *pcptr = drivable;
    boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ> > tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    tree->setEpsilon(0.01);
    tree->setInputCloud(pcptr);

    drivable.points.push_back(center);

    snprintf(currName,499,"%s_res.wrl",prefix.c_str());
    FILE *fout = fopen(currName,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    writeToVRML(fout,occupied,Eigen::Vector3d(1,0,0));
    writeToVRML(fout,drivable,Eigen::Vector3d(0,1,0));
    fclose(fout);

    //now, we go through point clouds again
    fin = fopen(argv[1],"r");
    ctr = 0;

    //get first line
    n = getline(&line,&len,fin);
    if(n <=0 ) return -1;
    prefix = line;
    *(prefix.rbegin()) = '\0';


    while(getline(&line,&len,fin) > 0) {

	n = sscanf(line,"%d %lf %lf %lf %lf %lf %lf %lf",
		&ts,&xd,&yd,&zd,&x,&y,&z,&w);
	if(n != 8) {
	    cout<<"wrong format of pose at : "<<line<<endl;
	    break;
	}

	currPose =Eigen::Translation<double,3>(xd,yd,zd)*
	    Eigen::Quaternion<double>(w,x,y,z);
	snprintf(currName,499,"%s%03d.wrl",prefix.c_str(),ts);
	curr = lslgeneric::readVRML(currName);

	lslgeneric::transformPointCloudInPlace(currPose,curr);
	//transform also the starting point...
	pcl::PointXYZ centerNew;
	Eigen::Vector3d xxx;
	xxx<<center.x,center.y,center.z;
	xxx = currPose*xxx;
	centerNew.x = xxx(0);centerNew.y = xxx(1);centerNew.z = xxx(2);

	pcl::PointCloud<pcl::PointXYZ> ldrivable, loccupied, fpdrive;
	//extract drivable in the new scan...
	if(extractDrivable(curr, centerNew, ldrivable, loccupied) < 0) return -1;

	std::vector<int> idx;
	std::vector<float> dist;
	float ptDist = 0.001;
	int tp=0, fp =0;

	//for all points in ldrivable, check distance to closest in drivable
	for(int q=0; q<ldrivable.points.size(); q++) {
	    
	    pcl::PointXYZ pt = ldrivable.points[q];
	    idx.clear(); dist.clear();

	    tree->radiusSearch(pt,ptDist,idx,dist);
	    if(idx.size() != 0) {
		if(dist[0] < ptDist) {
		    tp++; 
		    continue;
		}
	    }
	    fp++;
	    fpdrive.points.push_back(ldrivable.points[q]);
	}

	cout<<"scan number <<"<<ctr<<" TP = "<<tp<<" FP = "<<fp<<" all drivable: "<<ldrivable.points.size()<<endl;
	
	snprintf(currName,499,"%s_res_%d.wrl",prefix.c_str(),ctr);
	fout = fopen(currName,"w");
	fprintf(fout,"#VRML V2.0 utf8\n");
	writeToVRML(fout,loccupied,Eigen::Vector3d(1,0,0));
	writeToVRML(fout,ldrivable,Eigen::Vector3d(0,1,0));
	writeToVRML(fout,fpdrive,Eigen::Vector3d(0,0,0));
	fclose(fout);
	
	ctr++;
    }
    fclose (fin);

    return 0;
}
