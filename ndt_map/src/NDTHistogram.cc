#include <NDTHistogram.hh>
#include <PointCloudUtils.hh>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace lslgeneric;


NDTHistogram::NDTHistogram (NDTMap &map) {

    N_LINE_BINS = 10;
    N_FLAT_BINS = 20;
    N_SPHERE_BINS = 10;

    histogramBinsLine = std::vector<int>(N_LINE_BINS,0);
    histogramBinsFlat = std::vector<int>(N_FLAT_BINS,0);
    histogramBinsSphere = std::vector<int>(N_SPHERE_BINS,0);
    averageDirections = std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >(N_FLAT_BINS,Eigen::Vector3d(0,0,0));
    //populate directions
    computeDirections();
    constructHistogram(map);

}
	
void NDTHistogram::computeDirections() {

    double dlong = M_PI*(3-sqrt(5));  /* ~2.39996323 */
    double dz    = 2.0/N_FLAT_BINS;
    double longitude = 0;
    double z    = 1 - dz/2;
    
    for (int k = 0; k<N_FLAT_BINS; k++) {
	double r    = sqrt(1-z*z);
	Eigen::Vector3d v;
	v<<cos(longitude)*r, sin(longitude)*r, z;
	directions.push_back(v);
	z    = z - dz;
	longitude = longitude + dlong;
    }
}
    
void NDTHistogram::constructHistogram(NDTMap &map) {
    
    SpatialIndex *si = map.getMyIndex();
    if(si==NULL) return;

    double LINEAR_FACTOR = 50;
    double FLAT_FACTOR = 50;

    std::vector<Cell*>::iterator it = si->begin();
    while(it!=si->end()) {

	NDTCell *ndcell = dynamic_cast<NDTCell*>(*it);
        if(ndcell == NULL) {
	    it++;
	    continue;
	}
	if(!ndcell->hasGaussian) {
	    it++;
	    continue;
	}
	Eigen::Matrix3d evecs = ndcell->getEvecs();
	Eigen::Vector3d evals = ndcell->getEvals();

	int idMin,idMax,idMid;
	double minEval = evals.minCoeff(&idMin);
	double maxEval = evals.maxCoeff(&idMax);
	double midEval = -1;
	idMid = -1;
	for(int j=0; j<3; j++) {
	    if(j!=idMin && j!=idMax) {
		midEval = evals(j);
		idMid = j;
	    }
	}
	
	//three cases:
	//maxEval >> midEval -> linear
	if(maxEval > midEval*LINEAR_FACTOR) {
	    incrementLineBin();
	    it++;
	    continue;
	}
	
	//maxEval ~ midEval >> minEval -> planar
	if(midEval > minEval*FLAT_FACTOR) {
	    Eigen::Vector3d normal = evecs.col(idMin);
	    Eigen::Vector3d mean = ndcell->getMean();
	    if(normal.dot(mean) < 0) {
//		std::cout<<"switching normal direction\n";
		normal = -normal;
	    }
	    incrementFlatBin(normal);
	    it++;
	    continue;
	}

	//maxEval ~ midEval ~ minEval -> spherical
	incrementSphereBin();

	it++;
    }

    for(int i=0; i<averageDirections.size(); i++) {
	averageDirections[i].normalize();
    }

}

void NDTHistogram::incrementLineBin() {
    histogramBinsLine[0] ++;
}

void NDTHistogram::incrementFlatBin(Eigen::Vector3d &normal) {
    //std::cout<<"n "<<normal.transpose()<<std::endl;
    normal.normalize();
    //bins are in 3D. go through directions, find smallest difference
    double mindist = INT_MAX;
    int idmin = -1;
    for(unsigned int i=0; i<directions.size(); i++) {
	double dist = (directions[i]-normal).norm();
	if(mindist > dist) {
	    mindist = dist;
	    idmin = i;
	}
    }
    //std::cout<<idmin<<std::endl;
    if(idmin >=0 && idmin < histogramBinsFlat.size()) {
	histogramBinsFlat[idmin] ++;
	averageDirections[idmin] += normal;
    }
}

void NDTHistogram::incrementSphereBin() {
    histogramBinsSphere[0] ++;
}

	
pcl::PointCloud<pcl::PointXYZI> NDTHistogram::getDominantDirections(int nDirections) {

    pcl::PointCloud<pcl::PointXYZI> ret;
    std::vector<bool> dominated (directions.size(),false);
    double NORM_MIN = 0.2;
    int MIN_SUPPORT = 5;

    for(int i=0; i<nDirections; i++) {
	//get the next direction
	pcl::PointXYZI current;
	//find max in histogram, that is not dominated
	bool found = false;
	int maxBin, idMax;
	while (!found) {
	    maxBin = -1;
	    idMax = -1;
	    for(int j=0; j<histogramBinsFlat.size(); j++) {
		if(histogramBinsFlat[j] > maxBin && !dominated[j]) {
		    maxBin = histogramBinsFlat[j];
		    idMax = j;
		}
	    }

	    found = !dominated[idMax];
	    //check if any of the already found directions are "duals"
	   /* for(int j=0; j<ret.points.size() && found; j++) {
		Eigen::Vector3d v(ret.points[j].x,ret.points[j].y,ret.points[j].z);
		v = v + directions[idMax];
		found = found && (v.norm() > NORM_MIN);
	    }*/
	    //suppress this max and neighbours --- independent of the value of found, this is necessarry
	    dominated[idMax] = true;
	    if(idMax-1 >=0) dominated[idMax-1] = true;
	    if(idMax+1 <dominated.size()) dominated[idMax+1] = true;
	    if(maxBin < MIN_SUPPORT) break;
	}

	if(maxBin < MIN_SUPPORT) break;
	//current.x = directions[idMax](0);
	//current.y = directions[idMax](1);
	//current.z = directions[idMax](2);
	current.x = averageDirections[idMax](0);
	current.y = averageDirections[idMax](1);
	current.z = averageDirections[idMax](2);
	current.intensity = maxBin;
	//std::cout<<directions[idMax].transpose()<<" e "<<maxBin<<std::endl;
	//std::cout<<averageDirections[idMax].transpose()<<" e "<<maxBin<<std::endl;
	ret.points.push_back(current);
    }

    return ret;
}

void NDTHistogram::bestFitToHistogram(NDTHistogram &target, Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {

    //find the top N dominant directions
    int N = 3;
    //store vectors as pcl::points
    pcl::PointCloud<pcl::PointXYZI> dominantBinsMine, dominantBinsTarget;

//    std::cout<<"d1 : \n";    
    dominantBinsMine = this->getDominantDirections(N);
//    std::cout<<"d2 : \n";    
    dominantBinsTarget = target.getDominantDirections(N);

    //estimate least-squares fit, assuming correspondence
/*    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> trEst;
    Eigen::Matrix4f TR;
    trEst.estimateRigidTransformation(dominantBinsMine, dominantBinsTarget, TR); 
    T = TR.cast<double>();
*/
    //check for best fitting combination

    pcl::PointCloud<pcl::PointXYZI> mineNew, mineNew2;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> localT, localT2;

    double best = INT_MAX;
    //choose 3 out of N
    for(int a1 = 0; a1<dominantBinsMine.points.size(); a1++) {
    //int a1 = 0; {
	for(int b1 = 0; b1<dominantBinsMine.points.size(); b1++) {
	    if(a1 == b1) continue;
	    for(int c1 = 0; c1<dominantBinsMine.points.size(); c1++) {
		if(b1 == c1 || a1 == c1) continue;
		//int a2 = 0; {
		for(int a2 = 0; a2<dominantBinsTarget.points.size(); a2++) {
		    for(int b2 = 0; b2<dominantBinsTarget.points.size(); b2++) {
			if(a2 == b2) continue;
			for(int c2 = 0; c2<dominantBinsTarget.points.size(); c2++) {
			    if(b2 == c2 || a2 == c2) continue;
			    pcl::PointCloud<pcl::PointXYZI> useMine, useTarget;
			    useMine.points.push_back(dominantBinsMine[a1]);
			    useMine.points.push_back(dominantBinsMine[b1]);
			    //useMine.points.push_back(dominantBinsMine[c1]);
			    useTarget.points.push_back(dominantBinsTarget[a2]);
			    useTarget.points.push_back(dominantBinsTarget[b2]);
			    //useTarget.points.push_back(dominantBinsTarget[c2]);

//			    closedFormSolution(useMine, useTarget, localT2);
			    //don't use them for finding but just for verifying
			    useMine.points.push_back(dominantBinsMine[c1]);
			    useTarget.points.push_back(dominantBinsTarget[c2]);
			    //calculate goodness of fit
//			    mineNew = transformPointCloud(localT,useMine);
			    
			    //now also considering last orientation....
			    closedFormSolution(useMine, useTarget, localT2);
			    mineNew2 = transformPointCloud(localT2,useMine);
			
//			    double good = 0, scale = 0;
//			    for(int i=0; i<mineNew.points.size(); i++) {
//				double factor = (mineNew.points[i].intensity+useTarget.points[i].intensity);
//				good += factor*sqrt( pow(mineNew.points[i].x-useTarget.points[i].x,2) +
//					pow(mineNew.points[i].y-useTarget.points[i].y,2) +
//					pow(mineNew.points[i].z-useTarget.points[i].z,2));
//				scale += factor;
//			    }
//			    good = good/scale;
			    
			    
			    double good2 = 0, scale2 = 0;
			    for(int i=0; i<mineNew2.points.size(); i++) {
				double factor = (mineNew2.points[i].intensity+useTarget.points[i].intensity);
				good2 += factor*sqrt( pow(mineNew2.points[i].x-useTarget.points[i].x,2) +
					pow(mineNew2.points[i].y-useTarget.points[i].y,2) +
					pow(mineNew2.points[i].z-useTarget.points[i].z,2));
				scale2 += factor;
			    }
			    good2 = good2/scale2;
			    

//			    std::cout<<"combo "<<a1<<" "<<b1<<" "<<c1<<" -- "
//				<<a2<<" "<<b2<<" "<<c2<<" fit = "<<good2<<std::endl;
//			    if(good < best) {
//				std::cout<<"local minimum at combo "<<a1<<" "<<b1<<" "<<c1<<" -- "
//					 <<a2<<" "<<b2<<" "<<c2<<" fit = "<<good<<std::endl;
//			       best = good;
//			       T = localT;
//			    } 
			    if(good2 < best) {
//				std::cout<<"local minimum at combo "<<a1<<" "<<b1<<" "<<c1<<" -- "
//					 <<a2<<" "<<b2<<" "<<c2<<" fit = "<<good2<<std::endl;
			       best = good2;
			       T = localT2;
			    }
			}
		    }
		}
	    }
	}
    }

    if(dominantBinsMine.points.size() < 3 || dominantBinsTarget.points.size() < 3) {
	if(dominantBinsMine.points.size() < 2 || dominantBinsTarget.points.size() < 2) {
	    T.setIdentity();
	} else {
	    pcl::PointCloud<pcl::PointXYZI> useMine, useTarget;
	    useMine.points.push_back(dominantBinsMine[0]);
	    useMine.points.push_back(dominantBinsMine[1]);
	    useTarget.points.push_back(dominantBinsTarget[0]);
	    useTarget.points.push_back(dominantBinsTarget[1]);

	    closedFormSolution(useMine, useTarget, T);
	}
    }


    //RANSAC-it?

}
	
void NDTHistogram::closedFormSolution(pcl::PointCloud<pcl::PointXYZI> &src, pcl::PointCloud<pcl::PointXYZI> &tgt,
				Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {

    T.setIdentity ();
    Eigen::Matrix3d H; // = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();
    Eigen::MatrixXd P1, P2;         //temporary points for sets 1 and 2
    unsigned int itr=0;             //loop counters
    size_t size = src.points.size();      
    
    // Assemble the correlation matrix H = source * target'
    P1 = Eigen::MatrixXd(size,3);
    P2 = Eigen::MatrixXd(size,3);
    //compute values needed for the N matrix and for scale
    for(itr=0; itr<size; itr++) {
	P1(itr,0) = src.points[itr].x;
	P1(itr,1) = src.points[itr].y;
	P1(itr,2) = src.points[itr].z;
	P2(itr,0) = tgt.points[itr].x;
	P2(itr,1) = tgt.points[itr].y;
	P2(itr,2) = tgt.points[itr].z;
    }
    H = P1.transpose()*P2;

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d u = svd.matrixU ();
    Eigen::Matrix3d v = svd.matrixV ();

    // Compute R = V * U'
    if (u.determinant () * v.determinant () < 0)
    {
	for (int x = 0; x < 3; ++x)
	    v (x, 2) *= -1;
    }

    Eigen::Matrix3d R = v * u.transpose ();
    // Return the correct transformation
    T = R;

}

void NDTHistogram::printHistogram(bool bMatlab) {

    if(bMatlab) {
	//prints in a format suitable for matlab plotting
/*
	std::cout<<"L=[ ";
	for(unsigned int i=0; i<histogramBinsLine.size(); i++) {
	    std::cout<<histogramBinsLine[i]<<" ";
	}
	std::cout<<"];\n";
*/
	std::cout<<"F=[";
	for(unsigned int i=0; i<histogramBinsFlat.size(); i++) {
	    std::cout<<histogramBinsFlat[i]<<" ";
	}

/*	std::cout<<"];\nS=[";
	for(unsigned int i=0; i<histogramBinsSphere.size(); i++) {
	    std::cout<<histogramBinsSphere[i]<<" ";
	}
*/
	/*	std::cout<<"];\nD=[";
		for(unsigned int i=0; i<directions.size(); i++) {
		std::cout<<directions[i].transpose()<<"; ";
		}
	 */	
	std::cout<<"];\n";

    } else {
	std::cout<<"L: ";
	for(unsigned int i=0; i<histogramBinsLine.size(); i++) {
	    std::cout<<histogramBinsLine[i]<<" ";
	}

	std::cout<<"\nF: ";
	for(unsigned int i=0; i<histogramBinsFlat.size(); i++) {
	    std::cout<<histogramBinsFlat[i]<<" ";
	}

	std::cout<<"\nS: ";
	for(unsigned int i=0; i<histogramBinsSphere.size(); i++) {
	    std::cout<<histogramBinsSphere[i]<<" ";
	}
	std::cout<<"\n";
    }
}
