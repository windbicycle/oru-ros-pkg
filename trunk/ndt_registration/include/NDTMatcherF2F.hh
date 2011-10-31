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
#ifndef NDT_MATCHER_F2F_HH
#define NDT_MATCHER_F2F_HH

#include "NDTMap.hh"
#include "pcl/point_cloud.h"
#include "Eigen/Core"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//#define DNUMERICAL

#ifdef DNUMERICAL
#include "stdafx.h"
#include "optimization.h"
#endif

namespace lslgeneric{

  /**
   * This class implements NDT registration for 3D point cloud scans.
   */
  class NDTMatcherF2F 
  {
  public:
     /**
      parametrized constructor. A default set is (false,false,true,empty_vector). parameters are:
     \param _bNumeric --- experimental numeric derivatives, unstable
     \param _isIrregularGrid --- experimental single pass through an irregular grid. also unstable
     \param useDefaultGridResolutions --- if set, the following parameter is set to a preset value
     \param _resolutions --- if previous bool is not set, these are the resolutions (in reverse order) that we will go through
    */
    NDTMatcherF2F(bool _bNumeric, bool _isIrregularGrid, 
	          bool useDefaultGridResolutions, std::vector<double> _resolutions) {
	this->init(_bNumeric,_isIrregularGrid,useDefaultGridResolutions,_resolutions);
    }
    NDTMatcherF2F() {
	this->init(false,false,true,std::vector<double>());
    }
    NDTMatcherF2F(const NDTMatcherF2F& other) {
	this->init(other.bNumeric,other.isIrregularGrid,false,other.resolutions);
    }

    /**
     * Register two point clouds. This method builds an NDT
     * representation of the "fixed" point cloud and uses that for
     * registering the "moving" point cloud.
     * \param  fixed
     *   Reference data. NDT structure is built for this point cloud.
     * \param  moving
     *   The output transformation registers this point cloud to \c fixed.
     * \param  T
     *   This is an input/output parameter. The initial value of \c T
     *   gives the initial pose estimate of \c moving. When the
     *   algorithm terminates, \c T holds the registration result.
     */
    bool match( pcl::PointCloud<pcl::PointXYZ>& fixed, 
	        pcl::PointCloud<pcl::PointXYZ>& moving,
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T);

    /**
     * Registers a point cloud to an NDT structure.
     * \param  fixed
     *   Reference data.
     * \param  moving
     *   The output transformation registers this point cloud to \c fixed.
     * \param  T
     *   This is an input/output parameter. The initial value of \c T
     *   gives the initial pose estimate of \c moving. When the
     *   algorithm terminates, \c T holds the registration result.
     */
    bool match( NDTMap& fixed, 
	        NDTMap& moving,
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T);
   
    /**
      * computes the covariance of the match between moving and fixed, at T.
      * note --- computes NDT distributions based on the resolution in res
      * result is returned in cov
      */
    bool covariance( pcl::PointCloud<pcl::PointXYZ>& fixed, 
	        pcl::PointCloud<pcl::PointXYZ>& moving,
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
		Eigen::Matrix<double,6,6> &cov
	    );

    /**
      * computes the covariance of the match between moving and fixed, at T.
      * result is returned in cov
      */
    bool covariance( NDTMap& fixed, 
	        NDTMap& moving,
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
		Eigen::Matrix<double,6,6> &cov
	    );

#ifdef DNUMERICAL
    //sets current fixed and current moving, sets-up optimization and runs it 
    bool matchLBFGS( NDTMap* fixed, 
	        NDTMap* moving,
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T );
    

    static void _evaluate(
	    const alglib::real_1d_array &x,
	    double &func,
	    void *ptr
	    )
    {
	reinterpret_cast<NDTMatcherF2F*>(ptr)->evaluate(x, func);
    }
    //transforms x into a pose, computes a new "moving" ndt and evaluates it
    void evaluate(
	    const alglib::real_1d_array &x,
	    double &func
	    ); 

    /////
    //evaluate with analytical gradient 
    static void _evaluateG(
	    const alglib::real_1d_array &x,
	    double &func,
	    alglib::real_1d_array &grad,
	    void *ptr
	    )
    {
	reinterpret_cast<NDTMatcherF2F*>(ptr)->evaluateG(x, func, grad);
    }
    //transforms x into a pose, computes a new "moving" ndt and evaluates it
    void evaluateG(
	    const alglib::real_1d_array &x,
	    double &func,
	    alglib::real_1d_array &grad
	    ); 


    static void _callback(
	    const alglib::real_1d_array &x,
	    double func,
	    void *ptr
	    )
    {
	reinterpret_cast<NDTMatcherF2F*>(ptr)->callback(x, func);
    }
    
    void callback(
	    const alglib::real_1d_array &x,
	    double func
	    );
#endif

    //compute the score of a point cloud to an NDT
    double scoreNDT(std::vector<NDTCell*> &moving, NDTMap &fixed);
	
    
    //compute the score gradient & hessian of a point cloud + transformation to an NDT
    // input: moving, fixed, tr, computeHessian
    //output: score_gradient, Hessian 
    void derivativesNDT( 
	    std::vector<NDTCell*> &moving,
	    NDTMap &fixed,
	    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
	    //Eigen::Vector3d &eulerAngles,
	    Eigen::Matrix<double,6,1> &score_gradient,
	    Eigen::Matrix<double,6,6> &Hessian,
	    bool computeHessian
	    );

    void generateScoreDebug(const char* out, pcl::PointCloud<pcl::PointXYZ>& fixed, 
			    pcl::PointCloud<pcl::PointXYZ>& moving);
    double finalscore;
  private:

    Eigen::Matrix<double,3,6> Jest;
    Eigen::Matrix<double,18,6> Hest;
    Eigen::Matrix<double,3,18> Zest;
    Eigen::Matrix<double,18,18> ZHest;
    
    Eigen::Matrix<double,3,3> dRdx, dRdy, dRdz;
    Eigen::Matrix<double,3,3> dRdxdx, dRdxdy, dRdxdz, dRdydy, dRdydz, dRdzdz;
    //lf = likelihood function d1 and d2 from the paper
    double lfd1,lfd2;
    //bool useSimpleDerivatives;
    double current_resolution;
    int iteration_counter_internal;

    bool isIrregularGrid;
    bool bNumeric;
    std::vector<double> resolutions;

#ifdef DNUMERIC
    //TSV: for numeric optimization
    NDTMap *current_fixed, *current_moving;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> current_T;
    alglib::real_1d_array px;
#endif

    //initializes stuff;
    void init(bool _bNumeric, bool _isIrregularGrid, 
	          bool useDefaultGridResolutions, std::vector<double> _resolutions);
    //pre-computes the multipliers of the derivatives for all points
    void precomputeAngleDerivatives(Eigen::Vector3d &eulerAngles);
    
    //iteratively update the score gradient and hessian
    bool update_gradient_hessian(
	    Eigen::Matrix<double,6,1> &score_gradient, 
	    Eigen::Matrix<double,6,6> &Hessian, 

	    Eigen::Vector3d &m1, 
	    Eigen::Matrix3d &C1); 

    //pre-computes the derivative matrices Jest, Hest, Zest, ZHest
    void computeDerivatives(pcl::PointXYZ &m1, Eigen::Matrix3d C1, Eigen::Matrix3d R);

    //perform line search to find the best descent rate (Mohre&Thuente)
    //adapted from NOX
    double lineSearchMT( Eigen::Matrix<double,6,1> &score_gradient_init,
	    Eigen::Matrix<double,6,1> &increment,
	    std::vector<NDTCell*> &moving,
	    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &globalT,
	    NDTMap &ndt) ;

    //auxiliary functions for MoreThuente line search
    struct MoreThuente {
	static double min(double a, double b);
	static double max(double a, double b);
	static double absmax(double a, double b, double c);
	static int cstep(double& stx, double& fx, double& dx,
		double& sty, double& fy, double& dy,
		double& stp, double& fp, double& dp,
		bool& brackt, double stmin, double stmax);
    }; //end MoreThuente

    //perform a subsampling depending on user choice
    int NUMBER_OF_POINTS;
    
    void gradient_numeric( 
	    std::vector<NDTCell*> &moving,
	    NDTMap &fixed,
	    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
	    //Eigen::Vector3d &eulerAngles,
	    Eigen::Matrix<double,6,1> &score_gradient
	    );

  private:
    //storage for pre-computed angular derivatives
    Eigen::Vector3d jest13, jest23, jest04, jest14, jest24, jest05, jest15, jest25;
    Eigen::Vector3d a2,a3, b2,b3, c2,c3, d1,d2,d3, e1,e2,e3, f1,f2,f3;
    int NUMBER_OF_ACTIVE_CELLS;
    double normalizeAngle(double a);
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // end namespace
 
#endif
