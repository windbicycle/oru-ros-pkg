#include <NDTCell.hh>
#include <Debug.hh>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

using namespace std;
using namespace lslgeneric;

bool NDTCell::parametersSet = false;
double NDTCell::EVAL_ROUGH_THR;
double NDTCell::EVEC_INCLINED_THR;
double NDTCell::EVAL_FACTOR;

double lslgeneric::geomDist(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    Eigen::Vector3d v;
    v << p1.x-p2.x, p1.y-p2.y, p1.z-p2.z;
    return v.norm();
}


void NDTCell::setParameters(double _EVAL_ROUGH_THR   , 
	                    double _EVEC_INCLINED_THR, 
	                    double _EVAL_FACTOR       
			   ) {

    //defaults
    //NDTCell::EVAL_ROUGH_THR = 0.01;
    //NDTCell::EVEC_INCLINED_THR = cos(8*M_PI/18);//10 degree slope;
    //NDTCell::EVAL_FACTOR = 100;
    NDTCell::EVAL_ROUGH_THR    = _EVAL_ROUGH_THR   ; 
    NDTCell::EVEC_INCLINED_THR = cos(_EVEC_INCLINED_THR); 
    NDTCell::EVAL_FACTOR       = _EVAL_FACTOR      ; 

    parametersSet = true;
}

/** default constructor
  */
NDTCell::NDTCell() {
    hasGaussian = false;

    cost =-1;
    if(!parametersSet) {
	DBG(1,"using default config\n");
	setParameters();
    }
}

/** virtual destructor
  */
NDTCell::~NDTCell() {
}

/** produces a new Cell* of the same type
  */
Cell* NDTCell::clone() {
    NDTCell *ret = new NDTCell();
    ret->setDimensions(xsize,ysize,zsize);
    ret->setCenter(center);
    return ret;
}
/** produces a new Cell* of the same type and sets it to have teh same 
  * parameters as this cell.
  */
Cell* NDTCell::copy() {
    NDTCell *ret = new NDTCell();
    
    ret->setDimensions(xsize,ysize,zsize);
    ret->setCenter(center);

    for(int i=0; i<this->points.size(); i++) {
	pcl::PointXYZ pt = this->points[i];
	ret->points.push_back(pt);
    }	
    
    ret->setMean(this->getMean());
    ret->setCov(this->getCov());
    
    return ret;
}
	    
/** attempts to fit a gaussian in the cell. 
  computes covariance and mean using observations
  */
void NDTCell::computeGaussian() {
    if(points.size() <= 3) {
	hasGaussian=false;
	return;
    }
   
    mean<<0,0,0; 
    for(unsigned int i=0; i< points.size(); i++) {
	Eigen::Vector3d tmp;
	tmp<<points[i].x,points[i].y,points[i].z;
	mean += tmp;
    }
    mean /= (points.size());


    Eigen::MatrixXd mp;
    mp.resize(points.size(),3);
    for(unsigned int i=0; i< points.size(); i++) {
	mp(i,0) = points[i].x - mean(0);
	mp(i,1) = points[i].y - mean(1);
	mp(i,2) = points[i].z - mean(2);

    }
    
    cov = mp.transpose()*mp/(points.size()-1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);

    evecs = Sol.eigenvectors().real();
    evals = Sol.eigenvalues().real();

    if(evals(0) <= 0 || evals(1) <= 0 || evals(2) <= 0) {
	hasGaussian = false;
    } else {
	hasGaussian = true;
	bool recalc = false;
	//guard against near singular matrices::
	int idMin,idMax;
	//double minEval = evals.minCoeff(&idMin);
	double maxEval = evals.maxCoeff(&idMax);
	if(maxEval > evals(0)*EVAL_FACTOR) {
	    evals(0) = evals(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals(1)*EVAL_FACTOR) {
	    evals(1) = evals(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals(2)*EVAL_FACTOR) {
	    evals(2) = evals(idMax)/EVAL_FACTOR;
	    recalc = true;
	}

	if(recalc) {
	    Eigen::Matrix3d Lam;
	    Lam = evals.asDiagonal();
//	    cout<<"cov_old:\n"<<cov<<endl;
	    cov = evecs*Lam*(evecs.transpose());
//	    cout<<"cov:\n"<<cov<<endl;
	}
//	cout<<"NDT: cov = "<<*cov<<endl;
	classify();
	//compute inverse covariance
	Eigen::Matrix3d Lam;
	Lam = (evals).asDiagonal();
	icov = evecs*(Lam.inverse())*(evecs.transpose()); 

//	cout<<"L :"<<Lam<<endl;
//        cout<<"Li:"<<Lam.inverse()<<endl;	
	//parameters for likelihood computations
	double c1,c2,d3;
	double integral, outlier_ratio;
	integral = 0.1;
	outlier_ratio = 0.35;
	c1 = (1-outlier_ratio)/integral;
	c2 = outlier_ratio/pow(xsize,3);
	d3 = -log(c2);
	d1 = -log( c1 + c2 ) - d3;
	d2 = -2*log((-log( c1 * exp( -0.5 ) + c2 ) - d3 ) / d1);
    }
}


/** output method to save the ndt map as a vrml file
  */
void NDTCell::writeToVRML(FILE *fout, Eigen::Vector3d color) {
    if(hasGaussian) {
	Eigen::Vector3d col;

	switch (cl) {
	    case ROUGH:
		col<<1,0,0;
		break;
	    case HORIZONTAL:
		col<<0,1,0;
		break;
	    case VERTICAL:
		col<<0,1,1;
		break;
	    case INCLINED:
		col<<0,0,1;
		break;
	    default:
		col<<0,0,0;
		break;
	}

	if(fout == NULL) {
	    ERR("problem outputing to vrml\n");
	    return;
	}

	Eigen::Vector3d ori;
	//opposite order to get local transforms
	ori = evecs.eulerAngles(2,1,0);
	double F_ZERO = 10e-5;

	if (sqrt(evals(0)) < F_ZERO || sqrt(evals(1)) < F_ZERO || 
		sqrt(evals(2)) < F_ZERO) {
	    return;
	}
	if(isnan(ori(0)) || isnan(ori(1)) || isnan(ori(2))) {
	     return;
	}
	if(isinf(ori(0)) || isinf(ori(1)) || isinf(ori(2))) {
	    return;
	}


	fprintf(fout,"Transform {\n\t");
	//VRML transforms are applied like this:
	//P_new = Trans x Rot x Scale x P_old
	//translation is mean location
	//scale is evals x |evec|%lf %lf %lf\n\t 
	//orientation is from rotation matrix
	//FIXME! NOTE: scaled 3x for display reasons!
	if(color != Eigen::Vector3d(0,0,0)) {
	    col = color;
	}
	fprintf(fout,"\
		translation %lf %lf %lf \n\t \
		Transform {\n\
		rotation 0 0 1 %lf\n\t \
		Transform {\n\
		rotation 0 1 0 %lf\n\t\t \
		Transform { \n\
		rotation 1 0 0 %lf\n\t\t \
		Transform { \n\
		scale %lf %lf %lf \
		\n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Sphere {radius 1}\n",
    	        //\n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Box {size 1 1 1}\n",

		mean(0),mean(1),mean(2),
		ori(0),ori(1),-ori(2),
		//3*sqrt(evals(0)),3*sqrt(evals(1)),3*sqrt(evals(2))
		sqrt(evals(0)),sqrt(evals(1)),sqrt(evals(2))
	       );
	fprintf(fout,"\t\t\tappearance Appearance {\n\t\t\tmaterial Material \
		{ diffuseColor %lf %lf %lf }\n}\n}\n]\n}}}}}\n", 
		col(0),col(1),col(2)
	       );
/*
	pcl::PointXYZ c = this->getCenter();
	fprintf(fout,"\nTransform {\n\t");
	fprintf(fout,"\
		center 0 0 0\n\t\
		translation %lf %lf %lf \n\t \
		Transform { \n\
		scale %lf %lf %lf \
    	        \n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Box {size 1 1 1}\n}]}}\n",

		c.x,c.y,c.z,
		xsize,ysize,zsize
	       );
*/
/*	
	//The points
	fprintf(fout,"Shape {\n geometry PointSet {\n coord Coordinate {\n point [\n");
	for(unsigned int pit=0; pit<points.size(); ++pit) {
	    pcl::PointXYZ thisPoint = points[pit];
	    fprintf(fout,"%.5lf %.5lf %.5lf\n", thisPoint.x, thisPoint.y, thisPoint.z);
	}

	fprintf(fout,"]\n}\n color Color {\n color [\n");
	for(unsigned int pit=0; pit<points.size(); ++pit) {
	    fprintf(fout,"%.2f,%.2f,%.2f\n",col(0),col(1),col(2));
	}
	fprintf(fout,"]\n }\n }\n }\n");
	*/
    }
}

	    
/** classifies the cell according to the covariance matrix properties
    if smallest eigenval is bigger then roughness thershold, it's a rough cell
    evaluate inclination of the corresponding evector and classify as vertica, horizontal or inclined
  */
void NDTCell::classify() {

    cl = UNKNOWN;
    
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> tr;
    tr = tr.rotate(evecs);

    int index=-1;
    double minEval = evals.minCoeff(&index);
    if(index<0 || index > 2) return;
    
    if(minEval > EVAL_ROUGH_THR ) {
	cl = ROUGH;
    }
    //if 1 eval << other 2 -> planar distr
    else {
	//default case -> sloping surface
	cl = INCLINED;

	//check the orientation of the vanishing axis
	Eigen::Vector3d e3;
	e3<<0,0,1;

	Eigen::Vector3d minorAxis = evecs.col(index); //tr*e3;

	//dot product with vertical axis gives us the angle
	double d = minorAxis.dot(e3);
	double l = minorAxis.norm();
	double ac = d/l;
	if(fabsf(ac) < EVEC_INCLINED_THR) {
	    //angle is nearly perpendicular => vertical surface
	    cl = VERTICAL;
	}

	if(fabsf(ac) > 1-EVEC_INCLINED_THR) {
	    //angle is nearly 0 => horizontal surface
	    cl = HORIZONTAL;
	}
    }
}
	    
double NDTCell::getLikelihood(pcl::PointXYZ pt) {
    //compute likelihood
    if(!hasGaussian) return -1;
    Eigen::Vector3d vec (pt.x,pt.y,pt.z);
    vec = vec-mean;
    double likelihood = vec.dot(icov*vec);
    if(std::isnan(likelihood)) return -1;
    
    return exp(-likelihood/2);
    //return -d1*exp(-d2*likelihood/2);
}

//////////////////////Getters & Setters//////////////////////////////
/** getter for covariance
  */
Eigen::Matrix3d NDTCell::getCov() {
    return cov;
}

Eigen::Matrix3d NDTCell::getInverseCov() {
    return icov;
}

/** getter for mean
  */
Eigen::Vector3d NDTCell::getMean() {
    return mean;
}

/** setter for mean
  */
void NDTCell::setMean(Eigen::Vector3d _mean) {
    mean = _mean;
}

/** setter for covariance
  */
void NDTCell::setCov(Eigen::Matrix3d _cov) {
    cov = _cov;

    //update inverse covariance estimate
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);

    evecs = Sol.eigenvectors().real();
    evals = Sol.eigenvalues().real();

    if(evals(0) <= 0 || evals(1) <= 0 || evals(2) <= 0) {
	hasGaussian = false;
    } else {
	hasGaussian = true;
	bool recalc = false;
	//guard against near singular matrices::
	int idMin,idMax;
	//double minEval = evals.minCoeff(&idMin);
	double maxEval = evals.maxCoeff(&idMax);
	if(maxEval > evals(0)*EVAL_FACTOR) {
	    evals(0) = evals(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals(1)*EVAL_FACTOR) {
	    evals(1) = evals(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals(2)*EVAL_FACTOR) {
	    evals(2) = evals(idMax)/EVAL_FACTOR;
	    recalc = true;
	}

	if(recalc) {
	    Eigen::Matrix3d Lam;
	    Lam = evals.asDiagonal();
//	    cout<<"cov_old:\n"<<cov<<endl;
	    cov = evecs*Lam*(evecs.transpose());
//	    cout<<"cov:\n"<<cov<<endl;
	}
//	cout<<"NDT: cov = "<<*cov<<endl;
	classify();
	//compute inverse covariance
	Eigen::Matrix3d Lam;
	Lam = (evals).asDiagonal();
	icov = evecs*(Lam.inverse())*(evecs.transpose());

/*	
	double c1,c2,d3;
	double integral, outlier_ratio;
	integral = 0.1;
	outlier_ratio = 0.35;
	c1 = (1-outlier_ratio)/integral;
	c2 = outlier_ratio/pow(xsize,3);
	d3 = -log(c2);
	d1 = -log( c1 + c2 ) - d3;
	d2 = -2*log((-log( c1 * exp( -0.5 ) + c2 ) - d3 ) / d1);
*/

    }	
}

/** getter for cell traversibility class
  */
NDTCell::CellClass NDTCell::getClass() const {
    return cl;
}

/** getter for covariance eigenvectors
  */
Eigen::Matrix3d NDTCell::getEvecs() const {
    return evecs;
}

/** getter for covariance eignevalues
  */
Eigen::Vector3d NDTCell::getEvals() const {
    return evals;
}

/** \todo should be a method to merge with another NDTMap
  */
void NDTCell::updateObservation() {

}
///////////////////////////////////////////////////////////////
