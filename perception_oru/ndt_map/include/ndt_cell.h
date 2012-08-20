/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef NDT_CELL_HH
#define NDT_CELL_HH

#include <spatial_index.h>
#include <cell.h>
#include<impl/EventCounterData.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cstdio>
#include <Eigen/Eigen>

#include <fstream>

/// A rather unsophisticated way of determining the 
/// update method for a cell
/// Covariance intersection based estimation []
#define CELL_UPDATE_MODE_COVARIANCE_INTERSECTION 		0 
/// Recursive Sample variance method [Chan, Gene, Randall, Updating Formulae and pairwise algorithm for computing sample variances, tech report Standford, 1979] 
#define CELL_UPDATE_MODE_SAMPLE_VARIANCE 				1 
/// Yguel, Vasquez, Aycard, Siegward, Laugier, Error-Driven Refinement of Multi-scale gaussian maps
#define CELL_UPDATE_MODE_ERROR_REFINEMENT				2
///Combined CI and SV
#define CELL_UPDATE_MODE_SAMPLE_VARIANCE_WITH_RESET		3

namespace lslgeneric {

    /** \brief implements a normal distibution cell
        \details The base class for all NDT indeces, contains
	mean, covariance matrix as well as eigen decomposition of covariance
    */
    template<typename PointT>
    class NDTCell : public Cell<PointT> {
	public:
	    bool hasGaussian_;
	    double cost;
	    std::vector<PointT> points_;
	    
	    enum CellClass {HORIZONTAL=0, VERTICAL, INCLINED, ROUGH, UNKNOWN};
	    
	    NDTCell() {
				hasGaussian_ = false;
				if(!parametersSet_) {
						setParameters();
				}
				N = 0;
				R = 0.5;
				G = 0.5;
				B = 0.5;
				occ = 0;
				emptyval = 0;
				cellConfidence = 0.5;
	    }
	    virtual ~NDTCell() 
	    { 
				points_.clear(); 
	    }

	    NDTCell(PointT &center, double &xsize, double &ysize, double &zsize):
	        Cell<PointT>(center,xsize,ysize,zsize) 
	    { 
				hasGaussian_ = false;
				N = 0;
				R = 0.5;
				G = 0.5;
				B = 0.5;
				occ = 0;
				emptyval = 0;
				cellConfidence = 0.5;
				if(!parametersSet_) {
						setParameters();
				}
	    }

	    NDTCell(const NDTCell& other):Cell<PointT>() {
				this->center_ = other.center_;
				this->xsize_ = other.xsize_;
				this->ysize_ = other.ysize_;
				this->zsize_ = other.zsize_;
				this->hasGaussian_ = other.hasGaussian_;
				this->R = other.R;
				this->G = other.G;
				this->B = other.B;
				this->N = other.N;
				this->occ = other.occ;
				this->emptyval = other.emptyval;
				this->edata = other.edata;
				this->cellConfidence = other.cellConfidence;
	    }
	    
	    virtual Cell<PointT>* clone() const;
	    virtual Cell<PointT>* copy() const;

	    inline void computeGaussian(int mode=CELL_UPDATE_MODE_SAMPLE_VARIANCE_WITH_RESET);
			//template<typename PointT>
			//void NDTCell<pcl::PointXYZRGB>::computeGaussian()
			
	    void rescaleCovariance();
			/**
			* Rescales the covariance to protect against near sigularities
			* and computes the inverse - This does not change class member values
			* @return true if success, false if eigen values were negative
			*/
			bool rescaleCovariance(Eigen::Matrix3d &cov, Eigen::Matrix3d &invCov);
			
	    //void updateObservation();
	    void classify();
	    
	    void writeToVRML(FILE *fout, Eigen::Vector3d col = Eigen::Vector3d(0,0,0));
	    int writeToJFF(FILE * jffout);
	    int loadFromJFF(FILE * jffin);

	    inline CellClass getClass() const { return cl_; }
	    inline Eigen::Matrix3d getCov() const { return cov_; }
	    inline Eigen::Matrix3d getInverseCov() const { return icov_; }
	    inline Eigen::Vector3d getMean() const { return mean_; }
	    inline Eigen::Matrix3d getEvecs() const { return evecs_; }	    
	    inline Eigen::Vector3d getEvals() const { return evals_; }
	    inline Eigen::Matrix3d getCovSum() const { return covSum_; }
	    inline Eigen::Vector3d getMeanSum() const { return meanSum_; }
	    inline float getCellConfidence() const { return cellConfidence; }

	    void setCov(const Eigen::Matrix3d &cov);
	    inline void setMean(const Eigen::Vector3d &mean) { mean_ = mean; }
	    inline void setEvals(const Eigen::Vector3d &ev) { evals_ = ev; }
	    inline void setCovSum(const Eigen::Matrix3d &covSum) { covSum_ = covSum; }
	    inline void setMeanSum(const Eigen::Vector3d &meanSum) { meanSum_ = meanSum; }		
	    
	    ///use this to set the parameters for the NDTCell. \note be careful, remember that the parameters are static, thus global
	    static void setParameters(double _EVAL_ROUGH_THR   =0.1, 
				      double _EVEC_INCLINED_THR=8*M_PI/18, 
				      double _EVAL_FACTOR      =100 
				     );
	    double getLikelihood(const PointT &pt) const;
	    
	    virtual void addPoint(PointT &pt) {
				points_.push_back(pt);
	    }
	    
	    
	    virtual void addPoints(pcl::PointCloud<PointT> &pt) {
				points_.insert(points_.begin(),pt.points.begin(),pt.points.end());
	    }
	    
	    void setRGB(float r, float g, float b){
				R=r; G = g; B = b;
			}
	    void getRGB(float &r, float &g, float &b){
				r = R; g = G; b = B;
			}
			
			/**
			* Updates the occupancy value of the cell by summing @occ_val to 
			* class variable
			*/
			void updateOccupancy(float occ_val){
				occ+=occ_val;
				if(occ>255.0) occ = 255.0;
				if(occ<-255.0) occ = -255.0;
			}
			
			/**
			* Returns the current accumulated occupancy value
			*/
			float getOccupancy(){return occ;}
			
			void updateEmpty(){
				emptyval++;
			}
			
			float getDynamicLikelihood(unsigned int N){
				return edata.computeSemiStaticLikelihood(N);
			}
			void setOccupancy(float occ_){occ = occ_;}
			void setEmptyval(int emptyval_){emptyval=emptyval_;}
			void setEventData(TEventData _ed){edata = _ed;}
			void setCellConfidence(float conf){cellConfidence = conf; }
			void setN(int N_){N = N_;}

	private:
	    Eigen::Matrix3d cov_;
		Eigen::Matrix3d covSum_;
	    Eigen::Matrix3d icov_;
	    Eigen::Matrix3d evecs_;
	    Eigen::Vector3d mean_;
		Eigen::Vector3d meanSum_;
	    Eigen::Vector3d evals_;
	    CellClass cl_;
	    static bool parametersSet_;													// ???
	    static double EVAL_ROUGH_THR;		// = 0.1;								// ???
	    static double EVEC_INCLINED_THR; 	// = cos(8*M_PI/18);//10 degree slope;	// ???
	    static double EVAL_FACTOR;													// ???
	    double d1_,d2_;
		unsigned int N; 	///Number of points used for Normal distribution estimation so far
		int emptyval;
		float R,G,B; 		///RGB values [0..1] - Special implementations for PointXYZRGB & PointXYZI
		float occ;   		///Occupancy value stored as "Log odds" (if you wish)
		float cellConfidence;
		TEventData edata;
	public:
	      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
};

#include<impl/ndt_cell.hpp>

#endif
