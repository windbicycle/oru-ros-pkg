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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <pcl/point_cloud.h>
#include <vector>
#include <cstdio>
#include <Eigen/Eigen>

namespace lslgeneric {

    /** \brief implements a normal distibution cell
        \details The base class for all NDT indeces, contains
	mean, covariance matrix as well as eigen decomposition of covariance
    */
    template<typename PointT>
    class NDTCell : public Cell<PointT> {
	public:
	    bool hasGaussian_;
	    std::vector<PointT> points_;
	    
	    enum CellClass {HORIZONTAL=0, VERTICAL, INCLINED, ROUGH, UNKNOWN};
	    
	    NDTCell() {
		hasGaussian_ = false;
		if(!parametersSet_) {
		    setParameters();
		}
	    }
	    virtual ~NDTCell() 
	    { 
		points_.clear(); 
	    }

	    NDTCell(PointT &center, double &xsize, double &ysize, double &zsize):
	        Cell<PointT>(center,xsize,ysize,zsize) 
	    { 
		hasGaussian_ = false;
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
	    }
	    
	    virtual Cell<PointT>* clone() const;
	    virtual Cell<PointT>* copy() const;

	    void computeGaussian();
	    //void updateObservation();
	    void classify();
	    
	    void writeToVRML(FILE *fout, Eigen::Vector3d col = Eigen::Vector3d(0,0,0));

	    inline CellClass getClass() const { return cl_; }
	    inline Eigen::Matrix3d getCov() const { return cov_; }
	    inline Eigen::Matrix3d getInverseCov() const { return icov_; }
	    inline Eigen::Vector3d getMean() const {return mean_; }
	    inline Eigen::Matrix3d getEvecs() const { return evecs_; }	    
	    inline Eigen::Vector3d getEvals() const { return evals_; }	    

	    void setCov(const Eigen::Matrix3d &cov);
	    inline void setMean(const Eigen::Vector3d &mean) { mean_ = mean; }
	    inline void setEvals(const Eigen::Vector3d &ev) { evals_ = ev; }		
	    
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

	private:
	    Eigen::Matrix3d cov_;
	    Eigen::Matrix3d icov_;
	    Eigen::Matrix3d evecs_;
	    Eigen::Vector3d mean_;
	    Eigen::Vector3d evals_;
	    CellClass cl_;
	    static bool parametersSet_;
	    static double EVAL_ROUGH_THR;// = 0.1;
	    static double EVEC_INCLINED_THR; // = cos(8*M_PI/18);//10 degree slope;
	    static double EVAL_FACTOR;
	    double d1_,d2_;

	public:
	      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
};

#include<impl/ndt_cell.hpp>

#endif
