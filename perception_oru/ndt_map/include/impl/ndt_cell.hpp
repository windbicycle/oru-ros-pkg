#include <cstring>
#include <cstdio>

#define JFFERR(x) std::cerr << x << std::endl; return -1;

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
		ret->setRGB(this->R,this->G,this->B);
		ret->setOccupancy(occ);
		ret->setEmptyval(emptyval);
		ret->setEventData(edata);
		ret->setCellConfidence(cellConfidence);
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
    ret->setRGB(this->R,this->G,this->B);
		ret->setOccupancy(occ);
		ret->setEmptyval(emptyval);
		ret->setEventData(edata);
		ret->setCellConfidence(cellConfidence);
    return ret;
}
	    
/** 
		Attempts to fit a gaussian in the cell. 
		computes covariance and mean using observations
*/
template<typename PointT>
inline
void NDTCell<PointT>::computeGaussian(int mode) {
    
  	///Occupancy update part    
	   if(hasGaussian_){
			if(emptyval > 0 && points_.size() <= 6){
				points_.clear();
				updateOccupancy(-1.0);
				emptyval = 0;
				
				if(occ<=0){
					hasGaussian_ = false;  
					edata.updateSimple(EVENTMAP_FREE);
				}
				return;
			}
		}
		emptyval = 0;

		
    if(points_.size() <= 6){ 
				points_.clear();
// 				updateOccupancy(-1.0);
// 				if(occ<0){
// 					hasGaussian_ = false;
// 					edata.updateSimple(EVENTMAP_FREE);
// 				}
				return;
		}
		updateOccupancy(1.0);
    edata.updateSimple(EVENTMAP_OCCU);
    
		if(!hasGaussian_){
			mean_<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				mean_ += tmp;
			}
			meanSum_ = mean_;
			mean_ /= (points_.size());

			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - mean_(0);
				mp(i,1) = points_[i].y - mean_(1);
				mp(i,2) = points_[i].z - mean_(2);
			}
			covSum_ = mp.transpose()*mp;
			cov_ = covSum_/(points_.size()-1);
			this->rescaleCovariance();
			N = points_.size();
			points_.clear();
			cellConfidence = 0.5;
			
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_COVARIANCE_INTERSECTION){ ///Update using new information
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////						
			Eigen::Vector3d m2;
			m2<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			m2 /= (points_.size());
			
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
			}
			Eigen::Matrix3d c2;
			c2 = mp.transpose()*mp/(points_.size()-1);
			
			//double w1 = N / (N+points_.size());
			double w1 = 0.98;
			Eigen::Matrix3d c3,icov2,icov3;
			bool exists = false;
			double det=0;
			exists = rescaleCovariance(c2, icov2);
			//c2.computeInverseAndDetWithCheck(icov2,det,exists);
			
			if(exists){
				c3 = w1 * icov_ + (1.0-w1) * icov2;
				c3.computeInverseAndDetWithCheck(icov3,det,exists);
				if(exists){
					cov_ = icov3;
					mean_ = icov3 * (w1*icov_*mean_ + (1.0-w1)*icov2*m2);
					
					this->rescaleCovariance();
					N += points_.size();
					points_.clear();
				}else{
					fprintf(stderr,"Covariance Intersection failed - Inverse does not exist (2)\n");
				}
			}else{
				points_.clear();
				fprintf(stderr,"Covariance Intersection failed - Inverse does not exist (1)\n");
			}
		/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			Eigen::Vector3d m2;
			m2<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			Eigen::Vector3d T2 = m2;
			
			m2 /= (points_.size());
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
			}
			
			Eigen::Matrix3d c2;
			c2 = mp.transpose()*mp;
			Eigen::Matrix3d c3;
			
			double w1 =  ((double) N / (double)(points_.size()*(N+points_.size())));
			double w2 = (double) (points_.size())/(double) N;
			
			c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * ( w2 * meanSum_ - (T2)).transpose();
			
			meanSum_ = meanSum_ + T2;
			covSum_ = c3;
			N = N + points_.size();
			
			mean_ = meanSum_ / N;
			cov_ = covSum_ / N;
			this->rescaleCovariance();
			
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_ERROR_REFINEMENT){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			double e_min = 5.0e-4;
			double e_max = 5.0e-2;
			double o_max = 10.0;
			if(occ>10.0) occ=10.0;
			if(occ<-10.0) occ = -10.0;
			
			double epsilon = ((e_min - e_max) / (2.0*o_max)) * (occ+o_max)+e_max;
			
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				mean_ = mean_ + epsilon * (tmp - mean_);
				
				cov_ = cov_+epsilon * ((tmp-mean_) * (tmp-mean_).transpose() - cov_);
			}
			this->rescaleCovariance();
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE_WITH_RESET){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			Eigen::Vector3d m2;
			m2<<0,0,0;
			///Measurement mean
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			
			Eigen::Vector3d T2 = m2;
			m2 /= (points_.size());
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			///Covariance
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
			}
			Eigen::Matrix3d c2,icov2,c2Sum;
			c2 = mp.transpose()*mp;
			Eigen::Matrix3d c3;
			
			double w1 =  ((double) N / (double)(points_.size()*(N+points_.size())));
			double w2 = (double) (points_.size())/(double) N;
			
			c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * ( w2 * meanSum_ - (T2)).transpose();
			
			meanSum_ = meanSum_ + T2;
			covSum_ = c3;
			N = N + points_.size();
			
			/** Adaptation / "Recency weighting" **/
			if(N>1000){
				double coeff = 1000.0/(double)N;
				covSum_ = covSum_ * coeff;
				meanSum_ = meanSum_ * coeff;
				N = 1000;
			}
			
			mean_ = meanSum_ / N;
			cov_ = covSum_ / N;
			this->rescaleCovariance(); ///< this would be the optimal integrated
			c2Sum = c2;
			c2 /= (points_.size()-1);
			bool exists = rescaleCovariance(c2, icov2);
			if(exists){
				///Distance from integrated distribution to measurement
				double dist = ((mean_ - m2).transpose() * icov2 * (mean_-m2)) ;
				///Distance from the measurement to integrated distribution
				dist+= ((mean_ - m2).transpose() * icov_ * (mean_-m2));
				
				double P = exp(- (sqrt(0.5*dist) / 2.0)); /// Likelihood measured as average distance between the distances 
				
				cellConfidence = (( cellConfidence*P) / ( cellConfidence*P + (1.0f-cellConfidence)*(1.0f-P))); 	///< Bayes binary Update of the confidence
				
				///Prevent over confidence
				if( cellConfidence > 0.999) cellConfidence = 0.999;
				if( cellConfidence < 0.001) cellConfidence = 0.001;
				
				///Reset if the confidence of a cell drops below 1%
				if(cellConfidence < 0.01 ){
					cellConfidence = 0.5;
					mean_ = m2;
					cov_ = c2;
					rescaleCovariance();
					N = points_.size();
					meanSum_ = T2;
					covSum_ = c2Sum;
					edata.updateSimple(EVENTMAP_FREE);
				}
			}			
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////			
		}///End of update
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
}

/** attempts to fit a gaussian in the cell. 
computes covariance and mean using observations
*/
template<>
inline
void NDTCell<pcl::PointXYZRGB>::computeGaussian(int mode) {

	///Occupancy update part    
	   if(hasGaussian_){
			if(emptyval > 0 && points_.size() <= 6){
				points_.clear();
				updateOccupancy(-1.0);
				emptyval = 0;
				
				if(occ<=0){
					hasGaussian_ = false;  
					edata.updateSimple(EVENTMAP_FREE);
				}
				return;
			}
		}
		emptyval = 0;

		
    if(points_.size() <= 6){ 
				points_.clear();
// 				updateOccupancy(-1.0);
// 				if(occ<0){
// 					hasGaussian_ = false;
// 					edata.updateSimple(EVENTMAP_FREE);
// 				}
				return;
		}
		
    updateOccupancy(1.0);
    edata.updateSimple(EVENTMAP_OCCU);
	///Update the Gaussian			
		if(!hasGaussian_){
			mean_<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				mean_ += tmp;
			}
			meanSum_ = mean_;
			mean_ /= (points_.size());

			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0;
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - mean_(0);
				mp(i,1) = points_[i].y - mean_(1);
				mp(i,2) = points_[i].z - mean_(2);
				r+= ((float)points_[i].r)/255.0;
				g+= ((float)points_[i].g)/255.0;
				b+= ((float)points_[i].b)/255.0;
			}
			this->setRGB(r/(float)points_.size(), g/(float)points_.size(),b/(float)points_.size());

			this->rescaleCovariance();
			
			
			covSum_ = mp.transpose()*mp;
			cov_ = covSum_/(points_.size()-1);
			this->rescaleCovariance();
			N = points_.size();
			points_.clear();
			cellConfidence = 0.5;
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_COVARIANCE_INTERSECTION){ ///Update using new information
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////						
			Eigen::Vector3d m2;
			m2<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			m2 /= (points_.size());
						
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0;
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
				r+= ((float)points_[i].r)/255.0;
				g+= ((float)points_[i].g)/255.0;
				b+= ((float)points_[i].b)/255.0;
			}
			this->setRGB(r/(float)points_.size(), g/(float)points_.size(),b/(float)points_.size());
			Eigen::Matrix3d c2;			
			c2 = mp.transpose()*mp/(points_.size()-1);
			
			double scale  = 1.0;
			double w1 = 0.98;
			Eigen::Matrix3d c3,icov2,icov3;
			bool exists = false;
			double det=0;
			c2 = scale * c2;
			exists = rescaleCovariance(c2, icov2);
			
			if(exists){
				c3 = w1 * (icov_*1.0/scale) + (1.0-w1) * icov2;
				c3.computeInverseAndDetWithCheck(icov3,det,exists);
				if(exists){
					cov_ = icov3;
					mean_ = icov3 * (w1*(icov_*1.0/scale)*mean_ + (1.0-w1)*icov2*m2);
					
					this->rescaleCovariance();
					N += points_.size();
				}else{
					fprintf(stderr,"Covariance Intersection failed - Inverse does not exist (2)\n");
				}
			}else{
				R = 1.0;
				G = 0.0;
				B = 0.0;
				fprintf(stderr,"Covariance Intersection failed - Inverse does not exist (1)\n");
			}
			points_.clear();
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			Eigen::Vector3d m2;
			m2<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			Eigen::Vector3d T2 = m2;
			
			m2 /= (points_.size());
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0;
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
				r+= ((float)points_[i].r)/255.0;
				g+= ((float)points_[i].g)/255.0;
				b+= ((float)points_[i].b)/255.0;
			}
			this->setRGB(r/(float)points_.size(), g/(float)points_.size(),b/(float)points_.size());
			
			Eigen::Matrix3d c2;
			c2 = mp.transpose()*mp;
			Eigen::Matrix3d c3;
			
			double w1 =  ((double) N / (double)(points_.size()*(N+points_.size())));
			double w2 = (double) (points_.size())/(double) N;
			
			c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * ( w2 * meanSum_ - (T2)).transpose();
			
			meanSum_ = meanSum_ + T2;
			covSum_ = c3;
			N = N + points_.size();
			
			mean_ = meanSum_ / N;
			cov_ = covSum_ / N;
			this->rescaleCovariance();
			
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_ERROR_REFINEMENT){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			double e_min = 5.0e-4;
			double e_max = 5.0e-2;
			double o_max = 10.0;
			if(occ>10.0) occ=10.0;
			if(occ<-10.0) occ = -10.0;
			
			double epsilon = ((e_min - e_max) / (2.0*o_max)) * (occ+o_max)+e_max;
			
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				mean_ = mean_ + epsilon * (tmp - mean_);
				
				cov_ = cov_+epsilon * ((tmp-mean_) * (tmp-mean_).transpose() - cov_);
			}
			if(occ<0){
				R = -occ/10.0;
				B = 0;
			}
			else{
				B = occ/10.0;
				R = 0;
			}
		
			this->rescaleCovariance();
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE_WITH_RESET){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			Eigen::Vector3d m2;
			m2<<0,0,0;
			///Measurement mean
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			
			Eigen::Vector3d T2 = m2;
			m2 /= (points_.size());
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0;
			///Covariance
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
				r+= ((float)points_[i].r)/255.0;
				g+= ((float)points_[i].g)/255.0;
				b+= ((float)points_[i].b)/255.0;
			}
			this->setRGB(r/(float)points_.size(), g/(float)points_.size(),b/(float)points_.size());
			
			Eigen::Matrix3d c2,icov2,c2Sum;
			c2 = mp.transpose()*mp;
			Eigen::Matrix3d c3;
			
			double w1 =  ((double) N / (double)(points_.size()*(N+points_.size())));
			double w2 = (double) (points_.size())/(double) N;
			
			c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * ( w2 * meanSum_ - (T2)).transpose();
			
			meanSum_ = meanSum_ + T2;
			covSum_ = c3;
			N = N + points_.size();
			
			/** Adaptation / "Recency weighting" **/
			if(N>1000){
				double coeff = 1000.0/(double)N;
				covSum_ = covSum_ * coeff;
				meanSum_ = meanSum_ * coeff;
				N = 1000;
			}
			
			mean_ = meanSum_ / N;
			cov_ = covSum_ / N;
			this->rescaleCovariance(); ///< this would be the optimal integrated
			c2Sum = c2;
			c2 /= (points_.size()-1);
			bool exists = rescaleCovariance(c2, icov2);
			if(exists){
				///Distance from integrated distribution to measurement
				double dist = ((mean_ - m2).transpose() * icov2 * (mean_-m2)) ;
				///Distance from the measurement to integrated distribution
				dist+= ((mean_ - m2).transpose() * icov_ * (mean_-m2));
				
				double P = exp(- (sqrt(0.5*dist) / 2.0)); /// Likelihood measured as average distance between the distances 
				
				cellConfidence = (( cellConfidence*P) / ( cellConfidence*P + (1.0f-cellConfidence)*(1.0f-P))); 	///< Bayes binary Update of the confidence
				
				///Prevent over confidence
				if( cellConfidence > 0.999) cellConfidence = 0.999;
				if( cellConfidence < 0.001) cellConfidence = 0.001;
				
				///Reset if the confidence of a cell drops below 1%
				if(cellConfidence < 0.01 ){
					cellConfidence = 0.5;
					mean_ = m2;
					cov_ = c2;
					rescaleCovariance();
					N = points_.size();
					meanSum_ = T2;
					covSum_ = c2Sum;
					edata.updateSimple(EVENTMAP_FREE);
				}
			}			
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////			
		}///End of update
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
	points_.clear();
}

/**
 pcl::PointXYZI specialization
 attempts to fit a gaussian in the cell. 
computes covariance and mean using observations
*/

template<>
inline
void NDTCell<pcl::PointXYZI>::computeGaussian(int mode) {

		if(hasGaussian_){
			if(emptyval > 0 && points_.size() <= 6){
				points_.clear();
				updateOccupancy(-1.0);
				emptyval = 0;
			
				if(occ<=5){
					hasGaussian_ = false;  
					edata.updateSimple(EVENTMAP_FREE);
				}
				return;
			}
		}
		emptyval = 0;

		
    if(points_.size() <= 6){ 
				points_.clear();
				updateOccupancy(-1.0);
				if(!hasGaussian_) edata.updateSimple(EVENTMAP_FREE);
				if(occ<0){
					if(hasGaussian_) edata.updateSimple(EVENTMAP_FREE);
					hasGaussian_ = false;
				}
				return;
		}
    updateOccupancy(1.0);
    edata.updateSimple(EVENTMAP_OCCU);
    
		if(!hasGaussian_){
			mean_<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				mean_ += tmp;
			}
			meanSum_ = mean_;
			mean_ /= (points_.size());

			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0,intensity=0;
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - mean_(0);
				mp(i,1) = points_[i].y - mean_(1);
				mp(i,2) = points_[i].z - mean_(2);
				intensity+= ((float) points_[i].intensity)/255.0;
			}
			r = intensity/(float)points_.size();
			g = intensity/(float)points_.size();
			b = intensity/(float)points_.size();
			R = r;
			G = g;
			B = b;
			//fprintf(stderr,"RGB %f %f %f ", R,G,B);
			covSum_ = mp.transpose()*mp;
			cov_ = covSum_/(points_.size()-1);
			this->rescaleCovariance();
			N = points_.size();
			points_.clear();
			cellConfidence  =0.5;
			//fprintf(stderr,"I HAVE GAUSSIAN!!");
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_COVARIANCE_INTERSECTION){ ///Update using new information
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////						
			Eigen::Vector3d m2;
			m2<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			m2 /= (points_.size());
			
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0,intensity=0;
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
				intensity+= ((float) points_[i].intensity)/255.0;
			}
			r = intensity/(float)points_.size();
			g = intensity/(float)points_.size();
			b = intensity/(float)points_.size();
			R = 0.5*R + 0.5*r;
			G = 0.5*G + 0.5*g;
			//B = 0.5*B + 0.5*b;
			B = fabs(occ)/255.0;
			Eigen::Matrix3d c2;
			
			c2 = mp.transpose()*mp/(points_.size()-1);
			
			//double w1 = N / (N+points_.size());
			double scale  = 1.0;
			double w1 = 0.98;
			Eigen::Matrix3d c3,icov2,icov3;
			bool exists = false;
			double det=0;
			c2 = scale * c2;
			exists = rescaleCovariance(c2, icov2);
			//c2.computeInverseAndDetWithCheck(icov2,det,exists);
			
			if(exists){
				c3 = w1 * (icov_*1.0/scale) + (1.0-w1) * icov2;
				c3.computeInverseAndDetWithCheck(icov3,det,exists);
				if(exists){
					cov_ = icov3;
					mean_ = icov3 * (w1*(icov_*1.0/scale)*mean_ + (1.0-w1)*icov2*m2);
					
					this->rescaleCovariance();
					N += points_.size();
					//points_.clear();
				}else{
					fprintf(stderr,"Covariance Intersection failed - Inverse does not exist (2)\n");
				}
			}else{
				R = 1.0;
				G = 0.0;
				B = 0.0;
				fprintf(stderr,"Covariance Intersection failed - Inverse does not exist (1)\n");
			}
			points_.clear();
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			Eigen::Vector3d m2;
			m2<<0,0,0; 
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			Eigen::Vector3d T2 = m2;
			
			m2 /= (points_.size());
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0,intensity=0;
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
				intensity+= ((float) points_[i].intensity)/255.0;
			}
			r = intensity/(float)points_.size();
			g = intensity/(float)points_.size();
			b = intensity/(float)points_.size();
			R = 0.5*R + 0.5*r;
			G = 0.5*G + 0.5*g;
			//B = 0.5*B + 0.5*b;
			B = fabs(occ)/255.0;
			
			Eigen::Matrix3d c2;
			c2 = mp.transpose()*mp;
			Eigen::Matrix3d c3;
			
			double w1 =  ((double) N / (double)(points_.size()*(N+points_.size())));
			double w2 = (double) (points_.size())/(double) N;
			
			c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * ( w2 * meanSum_ - (T2)).transpose();
			
			meanSum_ = meanSum_ + T2;
			covSum_ = c3;
			N = N + points_.size();
			
			mean_ = meanSum_ / N;
			cov_ = covSum_ / N;
			this->rescaleCovariance();
			
			
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_ERROR_REFINEMENT){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			double e_min = 5.0e-4;
			double e_max = 5.0e-2;
			double o_max = 10.0;
			if(occ>10.0) occ=10.0;
			if(occ<-10.0) occ = -10.0;
			
			double epsilon = ((e_min - e_max) / (2.0*o_max)) * (occ+o_max)+e_max;
			
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				mean_ = mean_ + epsilon * (tmp - mean_);
				
				cov_ = cov_+epsilon * ((tmp-mean_) * (tmp-mean_).transpose() - cov_);
			}
			if(occ<0){
				R = -occ/10.0;
				B = 0;
			}
			else{
				B = occ/10.0;
				R = 0;
			}
		
			this->rescaleCovariance();
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////	
		}else if(mode == CELL_UPDATE_MODE_SAMPLE_VARIANCE_WITH_RESET){
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
			Eigen::Vector3d m2;
			m2<<0,0,0;
			///Measurement mean
			for(unsigned int i=0; i< points_.size(); i++) {
				Eigen::Vector3d tmp;
				tmp<<points_[i].x,points_[i].y,points_[i].z;
				m2 += tmp;
			}
			
			Eigen::Vector3d T2 = m2;
			m2 /= (points_.size());
			
			Eigen::MatrixXd mp;
			mp.resize(points_.size(),3);
			double r=0,g=0,b=0,intensity=0;
			///Covariance
			for(unsigned int i=0; i< points_.size(); i++) {
				mp(i,0) = points_[i].x - m2(0);
				mp(i,1) = points_[i].y - m2(1);
				mp(i,2) = points_[i].z - m2(2);
				intensity+= ((float) points_[i].intensity)/255.0;
			}
			r = intensity/(float)points_.size();
			g = intensity/(float)points_.size();
			b = intensity/(float)points_.size();
			
			G = 0.5*G + 0.5*g;
			B = 0; // fabs(occ)/255.0;
			
			Eigen::Matrix3d c2,icov2,c2Sum;
			c2 = mp.transpose()*mp;
			Eigen::Matrix3d c3;
			
			double w1 =  ((double) N / (double)(points_.size()*(N+points_.size())));
			double w2 = (double) (points_.size())/(double) N;
			
			c3 = covSum_ + c2 + w1 * (w2 * meanSum_ - (T2)) * ( w2 * meanSum_ - (T2)).transpose();
			
			meanSum_ = meanSum_ + T2;
			covSum_ = c3;
			N = N + points_.size();
			
			/** Adaptation / "Recency weighting" **/
			if(N>1000){
				double coeff = 1000.0/(double)N;
				covSum_ = covSum_ * coeff;
				meanSum_ = meanSum_ * coeff;
				N = 1000;
			}
			
			mean_ = meanSum_ / N;
			cov_ = covSum_ / N;
			this->rescaleCovariance(); ///< this would be the optimal integrated
			c2Sum = c2;
			c2 /= (points_.size()-1);
			bool exists = rescaleCovariance(c2, icov2);
			if(exists){
				///Distance from integrated distribution to measurement
				double dist = ((mean_ - m2).transpose() * icov2 * (mean_-m2)) ;
				///Distance from the measurement to integrated distribution
				dist+= ((mean_ - m2).transpose() * icov_ * (mean_-m2));
				
				double P = exp(- (sqrt(0.5*dist) / 2.0)); /// Likelihood measured as average distance between the distances 
				cellConfidence = (( cellConfidence*P) / ( cellConfidence*P + (1.0f-cellConfidence)*(1.0f-P))); 	///< Bayes binary Update of the confidence
				
				///Prevent over confidence
				if( cellConfidence > 0.999) cellConfidence = 0.999;
				if( cellConfidence < 0.001) cellConfidence = 0.001;
				
				///Reset if the confidence of a cell drops below 1%
				if(cellConfidence < 0.01 ){
					cellConfidence = 0.5;
					mean_ = m2;
					cov_ = c2;
					rescaleCovariance();
					N = points_.size();
					meanSum_ = T2;
					covSum_ = c2Sum;
					edata.updateSimple(EVENTMAP_FREE);
				}
			}			
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////			
		}///End of update
/////////////////////////////////////////////////////////////////////////////	
/////////////////////////////////////////////////////////////////////////////
		points_.clear();
}



////////////////////////////////////////////////////////////////////////////////////
/**
* Rescales the covariance to protect against near sigularities
* and computes the inverse - This does not change class member values
*/
template<typename PointT>
bool NDTCell<PointT>::rescaleCovariance(Eigen::Matrix3d &cov, Eigen::Matrix3d &invCov) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);
		
		Eigen::Matrix3d evecs;
		Eigen::Vector3d evals;
			
    evecs = Sol.eigenvectors().real();
    evals = Sol.eigenvalues().real();

    if(evals(0) <= 0 || evals(1) <= 0 || evals(2) <= 0) {
			fprintf(stderr,"Negative Eigenvalues!\n");
			return false;
    } else {
			bool recalc = false;
			//guard against near singular matrices::
			int idMax;
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
					cov = evecs*Lam*(evecs.transpose());
			}
			
			//compute inverse covariance
			Eigen::Matrix3d Lam;
			Lam = evals.asDiagonal();
			invCov = evecs*(Lam.inverse())*(evecs.transpose()); 
    }
    return true;
}




///////////////////////////////////////////////////////////////////////////////////
template<typename PointT>
void NDTCell<PointT>::rescaleCovariance() {
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
	    printf("problem outputing to vrml, wrong file pointer\n");
	    return;
	}

	// ####### added for debugging #######

	// std::cerr << "evals:" << std::endl;
	// std::cerr << evals_ << std::endl;

	// std::cerr << "evecs:" << std::endl;
	// std::cerr << evecs_ << std::endl;

	// std::cerr << "cov:" << std::endl;
	// std::cerr << cov_ << std::endl;

	// ###################################

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

/** helper function for writeToJFF()
  */
void writeJFFMatrix(FILE * jffout, Eigen::Matrix3d &mat){
	
	double dtemp[6];

	dtemp[0] = mat.coeff(0,0);
	dtemp[1] = mat.coeff(1,0);
	dtemp[2] = mat.coeff(2,0);
	dtemp[3] = mat.coeff(1,1);
	dtemp[4] = mat.coeff(2,1);
	dtemp[5] = mat.coeff(2,2);

	fwrite(dtemp, sizeof(double), 6, jffout);
	
}

/** another helper function for writeToJFF()
  */
void writeJFFVector(FILE * jffout, Eigen::Vector3d &vec){
	
	double dtemp[3];

	for(int i=0; i<3; i++){
		dtemp[i] = vec.coeff(i);
	}

	fwrite(dtemp, sizeof(double), 3, jffout);
	
}

/** yet another helper function for writeToJFF()
  */
void writeJFFEventData(FILE * jffout, TEventData &evdata){
	
	float    ftemp[4];
	uint8_t  ocval[1] = {evdata.occval};
	uint64_t evnts[1] = {evdata.events};

	ftemp[0] = evdata.a_exit_event;	
	ftemp[1] = evdata.b_exit_event;
	ftemp[2] = evdata.a_entry_event;
	ftemp[3] = evdata.b_entry_event;
	
	fwrite(ocval, sizeof(uint8_t),  1, jffout);
	fwrite(ftemp, sizeof( float ),  4, jffout);
	fwrite(evnts, sizeof(uint64_t), 1, jffout);

}

/** output method to save the ndt cell as part of a jff v0.5 file
  */
template<typename PointT>
int NDTCell<PointT>::writeToJFF(FILE * jffout) {
    
	PointT * center = &(this->center_);
	fwrite(center, sizeof(PointT), 1, jffout);
	double cell_size[3] = {this->xsize_, this->ysize_, this->zsize_};
	fwrite(cell_size, sizeof(double), 3, jffout);	

	writeJFFMatrix(jffout, cov_);
	writeJFFMatrix(jffout, covSum_);
	writeJFFVector(jffout, mean_);
	writeJFFVector(jffout, meanSum_);
	
	// Temporary arrays to write all cell data
	double dtemp[2] = {d1_, d2_};
	int    itemp[2] = {N, emptyval};
	float  ftemp[5] = {R, G, B, occ, cellConfidence};

	fwrite(dtemp, sizeof(double), 2, jffout);
	fwrite(itemp, sizeof( int ),  2, jffout);
	fwrite(ftemp, sizeof(float),  5, jffout);

	writeJFFEventData(jffout, edata);
	
	return 0;

}

/** helper function for loadFromJFF()
  */
int loadJFFMatrix(FILE * jffin, Eigen::Matrix3d &mat){
	
	double dtemp[6];

	if(fread(&dtemp, sizeof(double), 6, jffin) <= 0)
		return -1;

	mat(0,0) = dtemp[0];
	mat(1,0) = dtemp[1];
	mat(2,0) = dtemp[2];
	mat(1,1) = dtemp[3];
	mat(2,1) = dtemp[4];
	mat(2,2) = dtemp[5];
	mat(0,1) = dtemp[1];
	mat(0,2) = dtemp[2];
	mat(1,2) = dtemp[4];

	return 0;

}

/** another helper function for loadFromJFF()
  */
int loadJFFVector(FILE * jffin, Eigen::Vector3d &vec){
	
	double dtemp[3];

	if(fread(&dtemp, sizeof(double), 3, jffin) <= 0)
		return -1;

	vec << dtemp[0], dtemp[1], dtemp[2];

	return 0;
	
}

/** yet another helper function for loadFromJFF()
  */
int loadJFFEventData(FILE * jffin, TEventData &evdata){
	
	float    ftemp[4];
	uint8_t  ocval;
	uint64_t evnts;

	if(fread(&ocval, sizeof(uint8_t),  1, jffin) <= 0)
		return -1;
	if(fread(&ftemp, sizeof( float ),  4, jffin) <= 0)
		return -1;
	if(fread(&evnts, sizeof(uint64_t), 1, jffin) <= 0)
		return -1;

	evdata.a_exit_event  = ftemp[0];
	evdata.b_exit_event  = ftemp[1];
	evdata.a_entry_event = ftemp[2];
	evdata.b_entry_event = ftemp[3];
	evdata.occval        = ocval;
	evdata.events        = evnts;
	
	return 0;
}

/** input method to load the ndt cell from a jff v0.5 file
  */
template<typename PointT>
int NDTCell<PointT>::loadFromJFF(FILE * jffin) {
    
	PointT center;
	if(fread(&center, sizeof(PointT), 1, jffin) <= 0)
		return -1;
	this->setCenter(center);
	
	double dimensions[3];
	if(fread(&dimensions, sizeof(double), 3, jffin) <= 0)
		return -1;
	this->setDimensions(dimensions[0], dimensions[1], dimensions[2]);
	
	Eigen::Matrix3d temp_matrix;
	Eigen::Vector3d temp_vector;
	if(loadJFFMatrix(jffin, temp_matrix) < 0)
		return -1;
	this->setCov(temp_matrix);
	if(loadJFFMatrix(jffin, temp_matrix) < 0)
		return -1;
	this->setCovSum(temp_matrix);
	if(loadJFFVector(jffin, temp_vector) < 0)
		return -1;
	this->setMean(temp_vector);
	if(loadJFFVector(jffin, temp_vector) < 0)
		return -1;
	this->setMeanSum(temp_vector);

	// Temporary arrays to load all cell data to
	double dtemp[2];// = {d1_, d2_};
	int    itemp[2];// = {N, emptyval};
	float  ftemp[5];// = {R, G, B, occ, cellConfidence};

	if(fread(&dtemp, sizeof(double), 2, jffin) <= 0)
		return -1;
	if(fread(&itemp, sizeof( int ),  2, jffin) <= 0)
		return -1;
	if(fread(&ftemp, sizeof(float),  5, jffin) <= 0)
		return -1;

	this->d1_ = dtemp[0];
	this->d2_ = dtemp[1];

	this->setN(itemp[0]);
	this->setEmptyval(itemp[1]);

	this->setRGB(ftemp[0], ftemp[1], ftemp[2]);
	this->setOccupancy(ftemp[3]);
	this->setCellConfidence(ftemp[4]);
	
	TEventData edata;
	loadJFFEventData(jffin, edata);
	this->setEventData(edata);

	return 0;

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
    this->rescaleCovariance();

    /*
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
    */
}

}; // end namespace
