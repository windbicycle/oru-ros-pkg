#include <AdaptiveOctTree.hh>
#include <Debug.hh>
#include <boost/math/distributions/chi_squared.hpp>
#include <Eigen/Eigen>
#include <ros/ros.h>

using namespace std;
using namespace lslgeneric;
using boost::math::chi_squared;
using boost::math::cdf;


bool AdaptiveOctTree::parametersSet = false;

bool AdaptiveOctTree::useDornikHansen;
bool AdaptiveOctTree::useFlatness;
double AdaptiveOctTree::RSS_THRESHOLD;
double AdaptiveOctTree::DH_SIGNIFICANCE_LVL;
double AdaptiveOctTree::MIN_CELL_SIZE;
double AdaptiveOctTree::FLAT_FACTOR;
	    
void AdaptiveOctTree::setParameters(bool _useDornikHansen,    
                                    bool _useFlatness,
                                    double _RSS_THRESHOLD,
                                    double _DH_SIGNIFICANCE_LVL,
                                    double _MIN_CELL_SIZE,
                                    double _FLAT_FACTOR,
				    double _BIG_CELL_SIZE,
				    double _SMALL_CELL_SIZE
				    ) {

    OctTree::setParameters(_BIG_CELL_SIZE,
			   _SMALL_CELL_SIZE 
			  );

    AdaptiveOctTree::useDornikHansen      = _useDornikHansen;
    AdaptiveOctTree::useFlatness          = _useFlatness;
    AdaptiveOctTree::RSS_THRESHOLD        = _RSS_THRESHOLD;
    AdaptiveOctTree::DH_SIGNIFICANCE_LVL  = _DH_SIGNIFICANCE_LVL;
    AdaptiveOctTree::MIN_CELL_SIZE        = _MIN_CELL_SIZE;
    AdaptiveOctTree::FLAT_FACTOR          = _FLAT_FACTOR;
    
    parametersSet = true;
}   
/** empty! default constructor
  */
AdaptiveOctTree::AdaptiveOctTree() : OctTree() {
    if(!AdaptiveOctTree::parametersSet) {
	DBG(1,"using default config\n");
	AdaptiveOctTree::setParameters();
    }
}

/** constructor, calls parent OctTree constructor
  */
AdaptiveOctTree::AdaptiveOctTree(pcl::PointXYZ center, double xsize, double ysize, 
	double zsize, OctCell* type, OctTree *_parent, unsigned int _depth) : 
	OctTree(center,xsize,ysize,zsize,type,_parent,_depth) {

    if(!AdaptiveOctTree::parametersSet) {
	DBG(1,"using default config\n");
	AdaptiveOctTree::setParameters(NULL);
    }

}

/** empty destructor, all data are deallocated by parent class
  */
AdaptiveOctTree::~AdaptiveOctTree() {

}
	    

/**
  finds all leafs of this tree and fills the vector of pointers to the leafs
  */
void AdaptiveOctTree::computeTreeLeafs() {
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
void AdaptiveOctTree::postProcessPoints() {

    //compute leafs as OctTree*
    computeTreeLeafs();
    //cout<<"leafs :"<<myTreeLeafs.size()<<endl;

    for(unsigned int i=0; i<myTreeLeafs.size(); i++) {
	NDTCell * nd = dynamic_cast<NDTCell*>((myTreeLeafs[i])->myCell);
	if(nd == NULL) continue;
	if(useDornikHansen) {
	    double significance = computeDornikHansen(nd);
	    if(significance < 0) {
		//there were not enough points, let's clear the cell!
		//(myTreeLeafs[i])->myCell->points.clear();
		continue;
	    }
	    if(significance < DH_SIGNIFICANCE_LVL) {
		//split the leafs
		vector<OctTree*> newLeafs = splitTree(myTreeLeafs[i]);
		myTreeLeafs.insert(myTreeLeafs.end(),newLeafs.begin(),newLeafs.end());
	    } 
	} else if(useFlatness) {
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


	} else {
	    double rss = computeResidualSquare(nd);
	    if(rss > RSS_THRESHOLD) {
		//split the leafs
		vector<OctTree*> newLeafs = splitTree(myTreeLeafs[i]);
		myTreeLeafs.insert(myTreeLeafs.end(),newLeafs.begin(),newLeafs.end());
	    }
	}
    }

    leafsCached = false;
}

/**
  performs the Dornik-Hansen Omnibus normality test
*/
double AdaptiveOctTree::computeDornikHansen(NDTCell *cell) {
    double pval = 1;
    double Ep = 0;
    //test statistics breaks down for n<=7
    if(cell->points.size() <= 7) return -1;
    cell->computeGaussian();

    //degree is 2*number_of_dimensions
    chi_squared dist(6);

    Eigen::Vector3d mean = cell->getMean();
    Eigen::Matrix3d C = cell->getCov();
    
    Eigen::MatrixXd Xhat(cell->points.size(),3); //, tmpMat;

    for(unsigned int i=0; i< cell->points.size(); i++) {
	Xhat(i,0) = cell->points[i].x - mean(0);
	Xhat(i,1) = cell->points[i].y - mean(1);
	Xhat(i,2) = cell->points[i].z - mean(2);
    }
    
    //Compute transform for observed points
    Eigen::Matrix3d V;
    Eigen::Matrix3d H;
    Eigen::Vector3d lambda;
    Eigen::Matrix3d L;
    Eigen::Matrix3d R1;

    Eigen::MatrixXd R;

    V(0,0) = 1/sqrt(C(0,0)); 
    V(1,1) = 1/sqrt(C(1,1)); 
    V(2,2) = 1/sqrt(C(2,2));
    
    C = V*C*V;
    Eigen::EigenSolver<Eigen::Matrix3d> eig(C);
    H = eig.eigenvectors().real();
    lambda = eig.eigenvalues().real();
       
    //covariance is not positive semidefinate
    if(lambda.minCoeff() <= 0) return -1;

    L(0,0) = 1/sqrt(lambda(0)); 
    L(1,1) = 1/sqrt(lambda(1)); 
    L(2,2) = 1/sqrt(lambda(2)); 

    //transform observations
    R1 = H*L*H.transpose()*V;
    R = R1*Xhat.transpose();

    //compute skewness and kurtois of new observations (in each dimension)
    //samples are zero mean, compute for each dimension standard deviation
    Ep = 0.0;
    double n = R.cols();
    for(unsigned int dim = 0; dim < 3; dim ++) {
	double m2 = 0, m3 = 0, m4 =0;
	double b1 = 0, b2;
        double beta, omega2, gamma, y;
	double gamma2, a,c, k, alpha, chi;
	double z1,z2;

	for(unsigned int i=0; i<R.cols(); i++) {
	    m2 += pow(R(dim,i),2);
	    m3 += pow(R(dim,i),3);
	    m4 += pow(R(dim,i),4);
	}
	m2 /= R.cols();
	m3 /= R.cols();
	m4 /= R.cols();
//	cout <<"dim "<<dim<<" m2 "<<m2<<" m3 "<<m3<<" m4 "<<m4<<endl;
	b1 = m3/(pow(m2,1.5));
        b2 = m4/(pow(m2,2));

//	cout<<"b1 "<<b1<<" b2 "<<b2<<endl;

	//compute Z1 and Z2
	beta = 3*(n*n + 27*n -70)*(n+1)*(n+3)/((n-2)*(n+5)*(n+7)*(n+9));
	omega2 = -1+sqrt(2*(beta-1));
	gamma = 1/sqrt(log(sqrt(omega2)));
	y = b1*sqrt((omega2-1)*(n+1)*(n+3)/(12*(n-2)));
        z1 = gamma*log(y+sqrt(y*y+1));

	gamma2 = (n-3)*(n+1)*(n*n+15*n-4);
	a = (n-2)*(n+5)*(n+7)*(n*n+27*n-70)/(6*gamma2);
	c = (n-7)*(n+5)*(n+7)*(n*n+2*n-5)/(6*gamma2);
	k = (n+5)*(n+7)*(pow(n,3)+37*n*n+11*n-313)/(12*gamma2);
	alpha = a + b1*b1*c;
	chi = (b2-1-b1*b1)*2*k;
	z2 = (pow((chi/(2*alpha)),(1./3.))-1+(1./(9*alpha)))*sqrt(9*alpha);
	
//	cout<<"z1: "<<z1<<" z2: "<<z2<<endl;

	//compute Ep
	Ep += z1*z1 + z2*z2;
    }

    //compute probability from chi square cdf
    pval = 1-cdf(dist,Ep);

//    cout<<"Ep "<<Ep<<endl;
//    cout<<"P: "<<pval<<endl;
    if(pval>DH_SIGNIFICANCE_LVL){
	double dx,dy,dz;
	cell->getDimensions(dx,dy,dz);
	cout<<"final split was at ("<<dx<<","<<dy<<","<<dz<<"); pval is "<<pval<<endl;
    }
    return 0;

}
/**
fits a 3d gaussian in the cell and computes the residual squares sum 
*/
double AdaptiveOctTree::computeResidualSquare(NDTCell *cell) {
    double rss = 0; //residual sum squared
    double meanResidual = 0;
    double residualVar = 0;
    if(cell->points.size() <= 3) return 0;

    cell->computeGaussian();

    Eigen::Vector3d cur, curProd;
    Eigen::Matrix3d cov = cell->getCov();
    Eigen::Vector3d mean = cell->getMean();
    //Eigen::LLT<Eigen::Matrix3d> lltOfCov = cov.llt();

    for(unsigned int i=0; i< cell->points.size(); i++) {
	
	cur(0) = cell->points[i].x - mean(0);
	cur(1) = cell->points[i].y - mean(1);
	cur(2) = cell->points[i].z - mean(2);
	
	//lltOfCov.solve(cur,&curProd);
	//rss += cur.dot(curProd);
	rss += cur.dot(cur);
	meanResidual += cur.norm()/cell->points.size();
    }

    for(unsigned int i=0; i< cell->points.size(); i++) {
	cur(0) = cell->points[i].x - mean(0);
	cur(1) = cell->points[i].y - mean(1);
	cur(2) = cell->points[i].z - mean(2);

	residualVar += pow(cur.norm()-meanResidual,2)/(cell->points.size()-1);
    }
    double bic = rss/residualVar + log(cell->points.size());
//    ROS_INFO("rss %lf mean %lf var %lf bic %lf",rss,meanResidual,residualVar,bic);    
    return bic;
}
	    
/**
  splits a cell and returns a vector of the newly created children
  iterates points downwards
  */
vector<OctTree*> AdaptiveOctTree::splitTree(OctTree *octLeaf) {
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
SpatialIndex* AdaptiveOctTree::clone() {
    if(myCell == NULL) {
	return new AdaptiveOctTree();
    }
    double sx,sy,sz;
    myCell->getDimensions(sx,sy,sz);
    AdaptiveOctTree *tr = new AdaptiveOctTree(myCell->getCenter(),sx,sy,sz,myCell);
    return tr;
}
