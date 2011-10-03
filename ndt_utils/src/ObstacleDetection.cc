#include <ObstacleDetection.hh>

using namespace lsl_od;
using namespace lslgeneric;
using namespace std;

//simple clustering of points
void lsl_od::clusterPoints(pcl::PointCloud<pcl::PointXYZ> pc,
	    std::vector<pcl::PointCloud<pcl::PointXYZ> > &clusters) {

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pcptr(new pcl::PointCloud<pcl::PointXYZ>());
    *pcptr = pc;
    boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ> > tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    tree->setEpsilon(0.1);
    //tree->setInputCloud(pcptr);
    std::vector<pcl::PointIndices> IDX;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cl;
    cl.setInputCloud(pcptr);
    cl.setSearchMethod(tree);
    cl.setClusterTolerance(2);
    cl.setMinClusterSize(3);
    cl.extract(IDX);

    //pcl::extractEuclideanClusters(pc,tree,0.5,idx);

    for(int i =0 ; i<IDX.size(); i++) {
	//cout<<i<<" idx "<<IDX[i].indices.size()<<endl;
	pcl::PointCloud<pcl::PointXYZ> cluster;
	for(int j=0; j<IDX[i].indices.size(); j++) {
	    cluster.push_back(pc.points[IDX[i].indices[j]]);
	}
	clusters.push_back(cluster);
    }
}

//estimate volume of a point cluster
double lsl_od::clusterVolume(pcl::PointCloud<pcl::PointXYZ> pc) {
    NDTCell cell;
    double MIN_LEN = 1;
    double volume = 0; 
    for(int i=0; i<pc.points.size(); i++) {
	cell.addPoint(pc.points[i]);
    }	
    cell.computeGaussian();
    if(cell.hasGaussian) {
	Eigen::Vector3d evals = cell.getEvals();
	evals *= 3;
	cell.setEvals(evals);
	volume = 1000*4*M_PI*evals(0)*evals(1)*evals(2)/3; 
	if(volume<0.01) {
	    if(evals(0)>MIN_LEN || evals(1)>MIN_LEN || evals(2)>MIN_LEN) {
		volume = 0.11;
		cout<<"setting based on length "<<evals.transpose()<<endl;
		cerr<<"setting based on length "<<evals.transpose()<<endl;
	    } else {
		//TSV!!!!
		cout<<"cluster size   "<<pc.size()<<endl;
		cout<<"cluster mean   "<<cell.getMean().transpose()<<endl;
		cout<<"cluster volume "<<volume<<endl;

		cell.writeToVRML(ffout);
		return volume;
	    }
	}
	cout<<"cluster size   "<<pc.size()<<endl;
	cout<<"cluster mean   "<<cell.getMean().transpose()<<endl;
	cout<<"cluster volume "<<volume<<endl;

	cell.writeToVRML(ffout);
    } else {
	cout<<"cluster not valid!\n";
    }
    return volume;
}

//partition into drivable and obstacle regions (from wavefront)
void lsl_od::extractDrivable(pcl::PointCloud<pcl::PointXYZ> pc,
	pcl::PointCloud<pcl::PointXYZ> &drivable,
	pcl::PointCloud<pcl::PointXYZ> &occupied) {

    int MAX_COST = 10000;
    double NS = 200;
    double robotInclination = 3*(M_PI/180);
    //don't look at obstacles higher then 1 meter above the scanner
    double MAX_OBST_HEIGHT = 1; 
    pcl::PointCloud<pcl::PointXYZ> in,out;
    
    pcl::PointXYZ scannerPosition;
    scannerPosition.x =4;
    scannerPosition.y =0;
    scannerPosition.z =0;
    Eigen::Vector3d robotSize, inclinationConstraints;
    robotSize<<8,5,3;
    inclinationConstraints<<10*M_PI/180,9*M_PI/180,2*M_PI;
    lslgeneric::RobotKinematics kin(robotSize,inclinationConstraints);

    pcl::PointXYZ pt = scannerPosition;
    //clean points from the scan that are inside the robot body
    for(int i=0; i<pc.points.size(); i++) {
	pcl::PointXYZ pt2 = pc.points[i];
	double dist = sqrt( (pt.x-pt2.x)*(pt.x-pt2.x) + (pt.y-pt2.y)*(pt.y-pt2.y) 
		+(pt.z-pt2.z)*(pt.z-pt2.z));
	if(dist > kin.robotRadius3d() && pt2.z < MAX_OBST_HEIGHT) {
	    in.push_back(pt2);
	}
		
    }

    //add expected ground support points
    //add the points under the robot...
/*    for(double xi=-robotSize.x()/2; xi<robotSize.x()/2; xi+=robotSize.x()/NS) {
	for(double yi=-robotSize.y()/2; yi<robotSize.y()/2; yi+=robotSize.y()/NS) {
	    pcl::PointXYZ pt;
	    pt.x = xi+scannerPosition.x; 
	    pt.y = yi+scannerPosition.y;
	    double zoff = pt.x*sin(robotInclination); 
	    pt.z = scannerPosition.z-robotSize.z()+zoff;
	    in.points.push_back(pt);
	}
    }
*/
//    writeToVRML("ndt_tmp/init.wrl",in);

    LazzyGrid prototype(2);
    NDTMap *myMap = new NDTMap(&prototype);

    myMap->loadPointCloud(in);
    myMap->computeNDTCells();

    NDTWavefrontPlanner planner(myMap,&kin);
    myMap->writeToVRML("ndt_tmp/wave.wrl");
    
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
    center.x = 3 ; center.y = 0 ; center.z = -1;
    //find cell with goal position
    Cell *tmp = si->getCellForPoint(center);
    if(tmp == NULL) {
	//try to find closest cell in front of the robot
	vector<Cell*> closest;
	si->getNeighbors(center, 10, closest);

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
	    cerr<<"no close point to expected ground\n";
	    occupied = pc;
	    return ;
	} else {
	    pcl::PointXYZ t = tmp->getCenter();
	    cout<<"moved point to "<<t.x<<" "<<t.y<<" "<<t.z<<endl;
	}
    }
    NDTCell* goalCell = dynamic_cast<NDTCell*> (tmp);
    if(goalCell == NULL) {
	occupied = pc;
	return;
    } 
    if(!goalCell->hasGaussian) {
	//try to find a proper cell within the robot radius!
	NDTCell* gc = planner.findClosestProper(goalCell, 20);
	if(gc == NULL) {
	    occupied = pc;
	    return ;
	}
	if(!gc->hasGaussian ) {
	    occupied = pc;
	    return ;
	}
	goalCell = gc;
	printf("moved goal to (%lf,%lf,%lf)\n",gc->getCenter().x,
		gc->getCenter().y,gc->getCenter().z);
    }
    
    goalCell->cost = 0;
    vector<NDTCell*> Q;
    Q.push_back(goalCell);
    int itr = 0;

    while(Q.size() > 0) {
	NDTCell *cur = Q.front();
	Eigen::Vector3d myMean = cur->getMean();
	if(!planner.isCollision(cur)) {
	    vector<NDTCell*> neighbors = planner.computeAccessibleNeighbors(cur);
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

    FILE *fout = fopen("ndt_tmp/drivability.wrl","w");
    fprintf(fout,"#VRML V2.0 utf8\n");

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
		unreachable+=thiscloud;
	    }
	}
	++it;
    }	
    writeToVRML(fout,occupied,Eigen::Vector3d(1,0,0));
    writeToVRML(fout,drivable,Eigen::Vector3d(0,1,0));
    writeToVRML(fout,unreachable,Eigen::Vector3d(0.6,0.5,0));
    fclose(fout);

    occupied+=unreachable;
    return;
}

//extract outliers using 3DNDT likelihood
void lsl_od::detectOutliersNDT(pcl::PointCloud<pcl::PointXYZ> pcStatic,
	pcl::PointCloud<pcl::PointXYZ> pcNew,
	pcl::PointCloud<pcl::PointXYZ> &outliers) {
    
    double LTHRESH = 0.05;
    lslgeneric::AdaptiveOctTree tr;
    lslgeneric::OctTree::BIG_CELL_SIZE = 8; 
    lslgeneric::OctTree::SMALL_CELL_SIZE = 2; 
    lslgeneric::AdaptiveOctTree::MIN_CELL_SIZE = 1;
    
    lslgeneric::NDTMap nd(&tr);
    nd.loadPointCloud(pcStatic);
    nd.computeNDTCells(); 
    
    char fname[50];
    snprintf(fname,49,"ndt_tmp/ndt_map.wrl");
    nd.writeToVRML(fname);
    snprintf(fname,49,"ndt_tmp/map.wrl");
    FILE* output = fopen(fname,"w");
    fprintf(output,"#VRML V2.0 utf8\n");
    lslgeneric::writeToVRML(output,pcStatic);
    lslgeneric::writeToVRML(output,pcNew,Eigen::Vector3d(1,0,0));
    fclose(output);

    double maxLikelihood = INT_MIN;
    double sumLikelihoods = 0;
    int noutliers = 0;
    
    //loop through points and compute likelihoods LASER
    for(int i=0; i<pcNew.points.size(); i++) {
	pcl::PointXYZ thisPt = pcNew.points[i];
	double likelihood = nd.getLikelihoodForPoint(thisPt);
	if(likelihood < LTHRESH) {
	    sumLikelihoods += likelihood;
	    maxLikelihood = (likelihood > maxLikelihood) ? 
		likelihood : maxLikelihood;
	    outliers.points.push_back(thisPt);
	    noutliers++;
	}
    }
    cout<<endl;
    cout<<"n outliers: "<<noutliers<<endl;
    cout<<"% = "<<(double)noutliers/pcNew.points.size()<<endl;
//    cout<<"max likelihood "<<maxLikelihood<<endl;
//    cout<<"sum likelihoods "<<sumLikelihoods<<endl;
//    cout<<"average likelihood "<<sumLikelihoods/outliers.points.size()<<endl;
    snprintf(fname,49,"ndt_tmp/outliers_no_cluster.wrl");
    lslgeneric::writeToVRML(fname,outliers);

}

//extract outliers using simple point-to-point distance
void lsl_od::detectOutliersPointBased(pcl::PointCloud<pcl::PointXYZ> pcStatic,
	pcl::PointCloud<pcl::PointXYZ> pcNew,
	pcl::PointCloud<pcl::PointXYZ> &outliers) {

    //kdtree on pcStatic
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pcptr(new pcl::PointCloud<pcl::PointXYZ>());
    *pcptr = pcStatic;
    boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ> > tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    tree->setEpsilon(0.01);
    tree->setInputCloud(pcptr);

    pcl::PointXYZ pt;
    std::vector<int> idx;
    std::vector<float> dist;
    float ptDist = 0.2;
    //go through pcNew and search in tree
    for(int i=0; i<pcNew.points.size(); i++) {
	 pt = pcNew.points[i];
	 tree->radiusSearch(pt,ptDist,idx,dist);
	 if(idx.size() == 0) {
	    outliers.push_back(pt);
	 }
	 idx.clear();
    }
   
    char fname[50]; 
    cout<<"n outliers: "<<outliers.points.size()<<endl;
    cout<<"% = "<<(double)outliers.points.size()/pcNew.points.size()<<endl;
    snprintf(fname,49,"ndt_tmp/outliers_no_cluster.wrl");
    lslgeneric::writeToVRML(fname,outliers);
}

//cleans outliers based on cell class (remove the flat horizontal ones)
void lsl_od::cleanOutliersCellClass(pcl::PointCloud<pcl::PointXYZ> pc,
	pcl::PointCloud<pcl::PointXYZ> outliersNoisy,
	pcl::PointCloud<pcl::PointXYZ> &outliersClean) {

    //color outliers by traversibility
    char fname[50];
    lslgeneric::NDTMap nd2(new lslgeneric::LazzyGrid(0.5));
    nd2.loadPointCloud(pc);
    nd2.computeNDTCells();
    pcl::PointCloud<pcl::PointXYZ> pcHori, pcVert, pcRough, pcIncl;
    for(int i=0; i<outliersNoisy.points.size(); i++) {
	pcl::PointXYZ pt = outliersNoisy.points[i];
	lslgeneric::NDTCell* cell = NULL;
	if(nd2.getCellForPoint(pt,cell) && cell!=NULL) {
	    if(cell->getClass() == lslgeneric::NDTCell::HORIZONTAL) {
		pcHori.push_back(pt);
	    }
	    if(cell->getClass() == lslgeneric::NDTCell::VERTICAL) {
		pcVert.push_back(pt);
		outliersClean.push_back(pt);
	    }
	    if(cell->getClass() == lslgeneric::NDTCell::ROUGH) {
		pcRough.push_back(pt);
		outliersClean.push_back(pt);
	    }
	    if(cell->getClass() == lslgeneric::NDTCell::INCLINED) {
		pcIncl.push_back(pt);
		outliersClean.push_back(pt);
	    }
	}
    }
    snprintf(fname,49,"ndt_tmp/outliers_with_class.wrl");
    FILE *fout = fopen(fname,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    lslgeneric::writeToVRML(fout,pcHori,Eigen::Vector3d(0,1,0));
    lslgeneric::writeToVRML(fout,pcVert,Eigen::Vector3d(1,0,0));
    lslgeneric::writeToVRML(fout,pcRough,Eigen::Vector3d(0,1,1));
    lslgeneric::writeToVRML(fout,pcIncl,Eigen::Vector3d(0,0,1));
    fclose(fout);

    cout<<"n outliers (pre-clean): "<<outliersClean.points.size()<<endl;
    cout<<"% = "<<(double)outliersClean.points.size()/pc.points.size()<<endl;

}
//remove isolated outlier points
void lsl_od::cleanOutliersSaltPepper(pcl::PointCloud<pcl::PointXYZ> pc,
	pcl::PointCloud<pcl::PointXYZ> &outliers) {

    double DTHR = 1;
    int NMIN = 2; 
    char fname[50];
    for(int i=0; i<pc.points.size(); i++) {
	pcl::PointXYZ pt = pc.points[i];
	int n = 0;
	for(int j=0; j<pc.points.size(); j++) {
	    pcl::PointXYZ pt2 = pc.points[j];
	    double dist = sqrt( (pt.x-pt2.x)*(pt.x-pt2.x) + (pt.y-pt2.y)*(pt.y-pt2.y) 
				+(pt.z-pt2.z)*(pt.z-pt2.z));
	    if(dist < DTHR) n++;
	    if(n > NMIN) {
		outliers.points.push_back(pt);
		break;
	    }
	}
    }
    cout<<"n outliers (clean): "<<outliers.points.size()<<endl;
    snprintf(fname,49,"ndt_tmp/outliers_cluster.wrl");
    lslgeneric::writeToVRML(fname,outliers);

}

void lsl_od::clusterCellsOutput(pcl::PointCloud<pcl::PointXYZ> outCloud, pcl::PointCloud<pcl::PointXYZ> testCloud,
			std::string filename) {

    std::vector<pcl::PointCloud<pcl::PointXYZ> >clusters;
    if(outCloud.points.size() > 6000) {
	cerr<<"too many points to cluster, giving up ("<<outCloud.points.size()<<")\n";
	return;
    }
    clusterPoints(outCloud,clusters);
    ffout = fopen(filename.c_str(),"w");
    fprintf(ffout,"#VRML V2.0 utf8\n");

    cout<<"simple clustering found "<<clusters.size()<<" clusters\n";
    for(int i=0; i<clusters.size(); i++) {
	double volume = clusterVolume(clusters[i]);
	//if(volume < 0.1) continue;
	lslgeneric::writeToVRML(ffout,clusters[i],Eigen::Vector3d(1,0,0));
    }
    lslgeneric::writeToVRML(ffout,testCloud);
    fclose(ffout);

}
