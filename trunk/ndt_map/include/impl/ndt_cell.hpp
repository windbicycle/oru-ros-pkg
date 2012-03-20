namespace lslgeneric {

template<typename PointT>
bool NDTCell<PointT>::parametersSet_ = false;
template<typename PointT>
double NDTCell<PointT>::EVAL_ROUGH_THR;
template<typename PointT>
double NDTCell<PointT>::EVEC_INCLINED_THR;
template<typename PointT>
double NDTCell<PointT>::EVAL_FACTOR;

template<typename PointT>
void NDTCell<PointT>::setParameters(double _EVAL_ROUGH_THR   , 
	                    double _EVEC_INCLINED_THR, 
	                    double _EVAL_FACTOR       
			   ) {

    NDTCell<PointT>::EVAL_ROUGH_THR    = _EVAL_ROUGH_THR   ; 
    NDTCell<PointT>::EVEC_INCLINED_THR = cos(_EVEC_INCLINED_THR); 
    NDTCell<PointT>::EVAL_FACTOR       = _EVAL_FACTOR      ; 
    parametersSet_ = true;
}

/** produces a new Cell* of the same type
  */
template<typename PointT>
Cell<PointT>* NDTCell<PointT>::clone() const{
    NDTCell<PointT> *ret = new NDTCell<PointT>();
    ret->setDimensions(this->xsize_,this->ysize_,this->zsize_);
    ret->setCenter(this->center_);
    return ret;
}
/** produces a new Cell* of the same type and sets it to have the same 
  * parameters as this cell.
  */
template<typename PointT>
Cell<PointT>* NDTCell<PointT>::copy() const {
    NDTCell<PointT> *ret = new NDTCell<PointT>();
    
    ret->setDimensions(this->xsize_,this->ysize_,this->zsize_);
    ret->setCenter(this->center_);

    for(unsigned int i=0; i<this->points_.size(); i++) {
	PointT pt = this->points_[i];
	ret->points_.push_back(pt);
    }	
    
    ret->setMean(this->getMean());
    ret->setCov(this->getCov());
    
    return ret;
}
	    
/** attempts to fit a gaussian in the cell. 
  computes covariance and mean using observations
  */
template<typename PointT>
void NDTCell<PointT>::computeGaussian() {
    //tsv: update here
    if(hasGaussian_) return;

    if(points_.size() <= 3) {
	hasGaussian_=false;
	return;
    }
   
    mean_<<0,0,0; 
    for(unsigned int i=0; i< points_.size(); i++) {
	Eigen::Vector3d tmp;
	tmp<<points_[i].x,points_[i].y,points_[i].z;
	mean_ += tmp;
    }
    mean_ /= (points_.size());

    Eigen::MatrixXd mp;
    mp.resize(points_.size(),3);
    for(unsigned int i=0; i< points_.size(); i++) {
	mp(i,0) = points_[i].x - mean_(0);
	mp(i,1) = points_[i].y - mean_(1);
	mp(i,2) = points_[i].z - mean_(2);

    }
    
    cov_ = mp.transpose()*mp/(points_.size()-1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov_);

    evecs_ = Sol.eigenvectors().real();
    evals_ = Sol.eigenvalues().real();

    if(evals_(0) <= 0 || evals_(1) <= 0 || evals_(2) <= 0) {
	hasGaussian_ = false;
    } else {
	hasGaussian_ = true;
	bool recalc = false;
	//guard against near singular matrices::
	int idMax;
	//double minEval = evals.minCoeff(&idMin);
	double maxEval = evals_.maxCoeff(&idMax);
	if(maxEval > evals_(0)*EVAL_FACTOR) {
	    evals_(0) = evals_(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals_(1)*EVAL_FACTOR) {
	    evals_(1) = evals_(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals_(2)*EVAL_FACTOR) {
	    evals_(2) = evals_(idMax)/EVAL_FACTOR;
	    recalc = true;
	}

	if(recalc) {
	    Eigen::Matrix3d Lam;
	    Lam = evals_.asDiagonal();
	    cov_ = evecs_*Lam*(evecs_.transpose());
	}
	classify();
	//compute inverse covariance
	Eigen::Matrix3d Lam;
	Lam = (evals_).asDiagonal();
	icov_ = evecs_*(Lam.inverse())*(evecs_.transpose()); 

	//parameters for likelihood computations
	/*
	double c1,c2,d3;
	double integral, outlier_ratio;
	integral = 0.1;
	outlier_ratio = 0.35;
	c1 = (1-outlier_ratio)/integral;
	c2 = outlier_ratio/pow(this->xsize_,3);
	d3 = -log(c2);
	d1_ = -log( c1 + c2 ) - d3;
	d2_ = -2*log((-log( c1 * exp( -0.5 ) + c2 ) - d3 ) / d1_);
	*/
    }
}


/** output method to save the ndt map as a vrml file
  */
template<typename PointT>
void NDTCell<PointT>::writeToVRML(FILE *fout, Eigen::Vector3d color) {
    if(hasGaussian_) {
	Eigen::Vector3d col;

	switch (cl_) {
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
	    //ERR("problem outputing to vrml\n");
	    return;
	}


	Eigen::Vector3d ori;
	//opposite order to get local transforms
	ori = evecs_.eulerAngles(2,1,0);
	double F_ZERO = 10e-5;

	if (sqrt(evals_(0)) < F_ZERO || sqrt(evals_(1)) < F_ZERO || 
		sqrt(evals_(2)) < F_ZERO) {
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
	//NOTE: scaled 3x for display reasons!
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

		mean_(0),mean_(1),mean_(2),
		ori(0),ori(1),-ori(2),
		//3*sqrt(evals(0)),3*sqrt(evals(1)),3*sqrt(evals(2))
		sqrt(evals_(0)),sqrt(evals_(1)),sqrt(evals_(2))
	       );
	fprintf(fout,"\t\t\tappearance Appearance {\n\t\t\tmaterial Material \
		{ diffuseColor %lf %lf %lf }\n}\n}\n]\n}}}}}\n", 
		col(0),col(1),col(2)
	       );
    }
    else
    {
	/*
	fprintf(fout,"Shape {\n\tgeometry IndexedFaceSet {\n\t\tcoord \
		Coordinate {\n\t\tpoint [\n");
	fprintf(fout,"%lf %lf %lf\n", this->center_.x-this->xsize_/2, this->center_.y-this->ysize_/2, this->center_.z-this->zsize_/2);
	fprintf(fout,"%lf %lf %lf\n", this->center_.x+this->xsize_/2, this->center_.y-this->ysize_/2, this->center_.z-this->zsize_/2);
	fprintf(fout,"%lf %lf %lf\n", this->center_.x+this->xsize_/2, this->center_.y+this->ysize_/2, this->center_.z-this->zsize_/2);
	fprintf(fout,"%lf %lf %lf\n", this->center_.x-this->xsize_/2, this->center_.y+this->ysize_/2, this->center_.z-this->zsize_/2);

	fprintf(fout,"%lf %lf %lf\n", this->center_.x-this->xsize_/2, this->center_.y-this->ysize_/2, this->center_.z+this->zsize_/2);
	fprintf(fout,"%lf %lf %lf\n", this->center_.x+this->xsize_/2, this->center_.y-this->ysize_/2, this->center_.z+this->zsize_/2);
	fprintf(fout,"%lf %lf %lf\n", this->center_.x+this->xsize_/2, this->center_.y+this->ysize_/2, this->center_.z+this->zsize_/2);
	fprintf(fout,"%lf %lf %lf\n", this->center_.x-this->xsize_/2, this->center_.y+this->ysize_/2, this->center_.z+this->zsize_/2);

	fprintf(fout,"]\n}\ncolor Color {\n\t color [\n");
	for(int i=0; i<8; i++) {
	    fprintf(fout,"%lf %lf %lf\n", color(0),color(1),color(2) );
	}

	fprintf(fout,"]\n}\n coordIndex [\n");
	fprintf(fout,"0 1 2 3 -1\n\
		4 5 6 7 -1\n\
		0 1 5 4 -1\n\
		1 2 6 5 -1\n\
		2 3 7 6 -1\n\
		3 0 4 7 -1\n\
		]\n}\n}");
	*/
    }
}

	    
/** classifies the cell according to the covariance matrix properties
    if smallest eigenval is bigger then roughness thershold, it's a rough cell
    evaluate inclination of the corresponding evector and classify as vertica, horizontal or inclined
  */
template<typename PointT>
void NDTCell<PointT>::classify() {

    cl_ = UNKNOWN;
    
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> tr;
    tr = tr.rotate(evecs_);

    int index=-1;
    double minEval = evals_.minCoeff(&index);
    if(index<0 || index > 2) return;
    
    if(minEval > EVAL_ROUGH_THR ) {
	cl_ = ROUGH;
    }
    //if 1 eval << other 2 -> planar distr
    else {
	//default case -> sloping surface
	cl_ = INCLINED;

	//check the orientation of the vanishing axis
	Eigen::Vector3d e3;
	e3<<0,0,1;

	Eigen::Vector3d minorAxis = evecs_.col(index); //tr*e3;

	//dot product with vertical axis gives us the angle
	double d = minorAxis.dot(e3);
	double l = minorAxis.norm();
	double ac = d/l;
	if(fabsf(ac) < EVEC_INCLINED_THR) {
	    //angle is nearly perpendicular => vertical surface
	    cl_ = VERTICAL;
	}

	if(fabsf(ac) > 1-EVEC_INCLINED_THR) {
	    //angle is nearly 0 => horizontal surface
	    cl_ = HORIZONTAL;
	}
    }
}
	    
template<typename PointT>
double NDTCell<PointT>::getLikelihood(const PointT &pt) const {
    //compute likelihood
    if(!hasGaussian_) return -1;
    Eigen::Vector3d vec (pt.x,pt.y,pt.z);
    vec = vec-mean_;
    double likelihood = vec.dot(icov_*vec);
    if(std::isnan(likelihood)) return -1;
    
    return exp(-likelihood/2);
    //return -d1*exp(-d2*likelihood/2);
}


/** setter for covariance
  */
template<typename PointT>
void NDTCell<PointT>::setCov(const Eigen::Matrix3d &_cov) {
    cov_ = _cov;

    //update inverse covariance estimate
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov_);

    evecs_ = Sol.eigenvectors().real();
    evals_ = Sol.eigenvalues().real();

    if(evals_(0) <= 0 || evals_(1) <= 0 || evals_(2) <= 0) {
	hasGaussian_ = false;
    } else {
	hasGaussian_ = true;
	bool recalc = false;
	//guard against near singular matrices::
	int idMax;
	double maxEval = evals_.maxCoeff(&idMax);
	if(maxEval > evals_(0)*EVAL_FACTOR) {
	    evals_(0) = evals_(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals_(1)*EVAL_FACTOR) {
	    evals_(1) = evals_(idMax)/EVAL_FACTOR;
	    recalc = true;
	}
	if(maxEval > evals_(2)*EVAL_FACTOR) {
	    evals_(2) = evals_(idMax)/EVAL_FACTOR;
	    recalc = true;
	}

	if(recalc) {
	    Eigen::Matrix3d Lam;
	    Lam = evals_.asDiagonal();
	    cov_ = evecs_*Lam*(evecs_.transpose());
	}
	classify();
	//compute inverse covariance
	Eigen::Matrix3d Lam;
	Lam = (evals_).asDiagonal();
	icov_ = evecs_*(Lam.inverse())*(evecs_.transpose());
    }	
}

}; // end namespace
