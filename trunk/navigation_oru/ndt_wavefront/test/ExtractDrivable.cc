#include <climits>
#include <vector>
#include <ndt_wavefront.h>
#include <spatial_index.h>
#include <robot_kinematics.h>
#include <pointcloud_utils.h>
#include <oc_tree.h>
#include <lazy_grid.h>

#include <Eigen/Eigen>

#include "pcl/kdtree/kdtree_flann.h"

using namespace std;
using namespace lslgeneric;


void writeToVRML(FILE* fout, 
		std::vector<NDTCell<pcl::PointXYZ>,Eigen::aligned_allocator<NDTCell<pcl::PointXYZ> > > &ndts,
		Eigen::Vector3d color) {

    for(int i=0; i<ndts.size(); i++) 
    {
	ndts[i].writeToVRML(fout,color);
    }
}

int extractDrivable(pcl::PointCloud<pcl::PointXYZ> &in, pcl::PointXYZ center,
		     std::vector<NDTCell<pcl::PointXYZ>,Eigen::aligned_allocator<NDTCell<pcl::PointXYZ> > > &drivable,
		     std::vector<NDTCell<pcl::PointXYZ>,Eigen::aligned_allocator<NDTCell<pcl::PointXYZ> > > &occupied)
{

    //pcl::PointCloud<pcl::PointXYZ> in = global; //lslgeneric::readVRML(argv[1]);
    
   struct timeval tv_start, tv_end;
   gettimeofday(&tv_start,NULL);

   int MAX_COST = 100000;

    //pcl::PointCloud<pcl::PointXYZ> out;

//    LazzyGrid tr(0.5);
    
    lslgeneric::OctTree<pcl::PointXYZ> tr;
    tr.BIG_CELL_SIZE = 2;
    tr.SMALL_CELL_SIZE = 0.5;

    //NDTMap *myMap = new NDTMap(&prototype);
    NDTMap<pcl::PointXYZ> *myMap = new NDTMap<pcl::PointXYZ>(&tr);

    myMap->loadPointCloud(in);
    myMap->computeNDTCells();
    Eigen::Vector3d robotSize, inclinationConstraints;
    robotSize<<1,1,0.5;
    inclinationConstraints<<15*M_PI/180,15*M_PI/180,2*M_PI;
    lslgeneric::RobotKinematics kin(robotSize,inclinationConstraints);

    NDTWavefrontPlanner<pcl::PointXYZ> planner(myMap,&kin);
    
    SpatialIndex<pcl::PointXYZ> *si = myMap->getMyIndex();
    vector<Cell<pcl::PointXYZ>*>::iterator it = si->begin();
    while (it!=si->end()) {
	//set costs of all cells to inf
	NDTCell<pcl::PointXYZ> *c = dynamic_cast<NDTCell<pcl::PointXYZ>*> (*it);
	c->cost = INT_MAX;
	++it;
    }
    
    //find cell with goal position
    Cell<pcl::PointXYZ> *tmp = si->getCellForPoint(center);
    if(tmp == NULL) {
	printf("goal point is not in map!\n");
	//try to find closest cell in front of the robot
	vector<Cell<pcl::PointXYZ>*> closest;
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
    NDTCell<pcl::PointXYZ>* goalCell = dynamic_cast<NDTCell<pcl::PointXYZ>*> (tmp);
    if(goalCell == NULL) {
	printf("goal cell is not an NDT cell!\n");
	return -1;
    } 
    if(!goalCell->hasGaussian_) {
	printf("goal cell has no gaussians!\n");
	//try to find a proper cell within the robot radius!
	NDTCell<pcl::PointXYZ>* gc = planner.findClosestProper(goalCell, 20);
	if(gc == NULL) {
	    return -1;
	}
	if(!gc->hasGaussian_ ) {
	    return -1;
	}
	goalCell = gc;
	printf("moved goal to (%lf,%lf,%lf)\n",gc->getCenter().x,
		gc->getCenter().y,gc->getCenter().z);
    }
    
    goalCell->cost = 0;
    vector<NDTCell<pcl::PointXYZ>*> Q;
    Q.push_back(goalCell);
    int itr = 0;

    while(Q.size() > 0) {
//	cout<<"itr "<<itr<<endl;
	NDTCell<pcl::PointXYZ> *cur = Q.front();
	Eigen::Vector3d myMean = cur->getMean();
	if(!planner.isCollision(cur)) {
	    vector<NDTCell<pcl::PointXYZ>*> neighbors = planner.computeAccessibleNeighbors(cur);
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
	NDTCell<pcl::PointXYZ> *c = dynamic_cast<NDTCell<pcl::PointXYZ>*> (*it);
	if( c->hasGaussian_ ) {
	    if(c->cost == -1) {
		occupied.push_back(*c);
	    } 
	    else if(c->cost < MAX_COST) {
		drivable.push_back(*c);
	    } else {
		occupied.push_back(*c);
	    }
	}
	++it;
    }
   
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
	curr = lslgeneric::readVRML<pcl::PointXYZ>(currName);

	lslgeneric::transformPointCloudInPlace(currPose,curr);
	global += curr;

	ctr++;
    }
    fclose (fin);

    //find closest point to scan origin
    pcl::PointXYZ center;
    center.x = 1.5 ; center.y = -0.5 ; center.z = -1.1;
    std::vector<NDTCell<pcl::PointXYZ>,Eigen::aligned_allocator<NDTCell<pcl::PointXYZ> > > drivable,occupied;

    if(extractDrivable(global, center, drivable, occupied) < 0) return -1;

    //kdtree on drivable
    /*
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pcptr(new pcl::PointCloud<pcl::PointXYZ>());
    *pcptr = drivable;
    boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ> > tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    tree->setEpsilon(0.01);
    tree->setInputCloud(pcptr);
    */

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
	curr = lslgeneric::readVRML<pcl::PointXYZ>(currName);

	lslgeneric::transformPointCloudInPlace(currPose,curr);
	//transform also the starting point...
	pcl::PointXYZ centerNew;
	Eigen::Vector3d xxx;
	xxx<<center.x,center.y,center.z;
	xxx = currPose*xxx;
	centerNew.x = xxx(0);centerNew.y = xxx(1);centerNew.z = xxx(2);

	pcl::PointCloud<pcl::PointXYZ> fpdrive;
	std::vector<NDTCell<pcl::PointXYZ>,Eigen::aligned_allocator<NDTCell<pcl::PointXYZ> > > ldrivable, loccupied;
	//extract drivable in the new scan...
	if(extractDrivable(curr, centerNew, ldrivable, loccupied) < 0) return -1;
/*	
	snprintf(currName,499,"%s_orig_%d.wrl",prefix.c_str(),ctr);
	fout = fopen(currName,"w");
	fprintf(fout,"#VRML V2.0 utf8\n");
	writeToVRML<pcl::PointXYZ>(fout,curr,Eigen::Vector3d(1,1,0));
	fclose(fout);
*/
/*
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
	
    */
	snprintf(currName,499,"%s_res_%d.wrl",prefix.c_str(),ctr);
	fout = fopen(currName,"w");
	fprintf(fout,"#VRML V2.0 utf8\n");
	writeToVRML(fout,loccupied,Eigen::Vector3d(1,0,0));
	writeToVRML(fout,ldrivable,Eigen::Vector3d(0,1,0));
//	writeToVRML<pcl::PointXYZ>(fout,fpdrive,Eigen::Vector3d(0,0,0));
	fclose(fout);
	
	ctr++;
    }
    fclose (fin);

    return 0;
}
