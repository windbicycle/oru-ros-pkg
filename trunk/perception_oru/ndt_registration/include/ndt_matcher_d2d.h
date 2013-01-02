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
#ifndef NDT_MATCHER_D2D_HH
#define NDT_MATCHER_D2D_HH

#include "ndt_map.h"
#include "pcl/point_cloud.h"
#include "Eigen/Core"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

namespace lslgeneric
{

/**
 * This class implements NDT registration for 3D point cloud scans.
 */
template <typename PointSource, typename PointTarget>
class NDTMatcherD2D
{
public:
    /**
     parametrized constructor. A default set is (false,false,true,empty_vector). parameters are:
    \param _isIrregularGrid --- experimental single pass through an irregular grid. also unstable
    \param useDefaultGridResolutions --- if set, the following parameter is set to a preset value
    \param _resolutions --- if previous bool is not set, these are the resolutions (in reverse order) that we will go through
    */
    NDTMatcherD2D(bool _isIrregularGrid,
                  bool useDefaultGridResolutions, std::vector<double> _resolutions)
    {
        this->init(_isIrregularGrid,useDefaultGridResolutions,_resolutions);
    }
    NDTMatcherD2D()
    {
        this->init(false,true,std::vector<double>());
    }
    NDTMatcherD2D(const NDTMatcherD2D& other)
    {
        this->init(other.isIrregularGrid,false,other.resolutions);
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
    bool match( pcl::PointCloud<PointTarget>& target,
                pcl::PointCloud<PointSource>& source,
                Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                bool useInitialGuess = false);
    /**
     * Register two point clouds. Use only 2D rotations This method builds an NDT
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
    bool match2D( pcl::PointCloud<PointTarget>& target,
                  pcl::PointCloud<PointSource>& source,
                  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                  bool useInitialGuess = false);

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
    bool match( NDTMap<PointTarget>& target,
                NDTMap<PointSource>& source,
                Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                bool useInitialGuess = false);

    /**
     * Registers a two ndt maps using only 2D rotation/translation.
     * \param  fixed
     *   Reference data.
     * \param  moving
     *   The output transformation registers this point cloud to \c fixed.
     * \param  T
     *   This is an input/output parameter. The initial value of \c T
     *   gives the initial pose estimate of \c moving. When the
     *   algorithm terminates, \c T holds the registration result.
     */
    bool match2D( NDTMap<PointTarget>& target,
                  NDTMap<PointSource>& source,
                  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                  bool useInitialGuess = false);


    /**
      * computes the covariance of the match between moving and fixed, at T.
      * note --- computes NDT distributions based on the resolution in res
      * result is returned in cov
      */
    bool covariance( pcl::PointCloud<PointTarget>& target,
                     pcl::PointCloud<PointSource>& source,
                     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                     Eigen::MatrixXd &cov
                   );

    /**
      * computes the covariance of the match between moving and fixed, at T.
      * result is returned in cov
      */
    bool covariance( NDTMap<PointTarget>& target,
                     NDTMap<PointSource>& source,
                     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                     Eigen::MatrixXd &cov
                   );

    //compute the score of a point cloud to an NDT //UNUSED
    virtual double scoreNDT(std::vector<NDTCell<PointSource>*> &source, NDTMap<PointTarget> &target);

    virtual double scoreNDT_OM(NDTMap<PointSource> &source, NDTMap<PointTarget> &target);


    virtual double scoreNDTPositive(std::vector<NDTCell<PointSource>*> &sourceNDT, NDTMap<PointTarget> &targetNDT,
                                    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T);

    //compute the score gradient & hessian of a point cloud + transformation to an NDT
    // input: moving, fixed, tr, computeHessian
    //output: score_gradient, Hessian, returns: score!
    virtual double derivativesNDT(
        const std::vector<NDTCell<PointSource>*> &source,
        const NDTMap<PointTarget> &target,
        Eigen::MatrixXd &score_gradient,
        Eigen::MatrixXd &Hessian,
        bool computeHessian
    );
    /*
        void generateScoreDebug(const char* out, pcl::PointCloud<pcl::PointXYZ>& fixed,
    			    pcl::PointCloud<pcl::PointXYZ>& moving);
    */
    double finalscore;
    double current_resolution;
    //max iterations, set in constructor
    int ITR_MAX;
protected:

    Eigen::Matrix<double,3,6> Jest;
    Eigen::Matrix<double,18,6> Hest;
    Eigen::Matrix<double,3,18> Zest;
    Eigen::Matrix<double,18,18> ZHest;

    Eigen::Matrix<double,3,3> dRdx, dRdy, dRdz;
    Eigen::Matrix<double,3,3> dRdxdx, dRdxdy, dRdxdz, dRdydy, dRdydz, dRdzdz;
    //lf = likelihood function d1 and d2 from the paper
    double lfd1,lfd2;
    //bool useSimpleDerivatives;
    int iteration_counter_internal;

    bool isIrregularGrid;
    std::vector<double> resolutions;

    //initializes stuff;
    void init(bool _isIrregularGrid,
              bool useDefaultGridResolutions, std::vector<double> _resolutions);

    //iteratively update the score gradient and hessian
    virtual bool update_gradient_hessian(
        Eigen::MatrixXd &score_gradient,
        Eigen::MatrixXd &Hessian,
        const Eigen::Vector3d &m1,
        const Eigen::Matrix3d &C1,
        const double &likelihood,
        bool computeHessian);

    //pre-computes the derivative matrices Jest, Hest, Zest, ZHest
    void computeDerivatives(Eigen::Vector3d &m1, Eigen::Matrix3d C1, bool computeHessian=true);

    //iteratively update the score gradient and hessian
    virtual bool update_gradient_hessian_local(
        Eigen::MatrixXd &score_gradient,
        Eigen::MatrixXd &Hessian,
        const Eigen::Vector3d &m1,
        const Eigen::Matrix3d &C1,
        const double &likelihood,
        const Eigen::Matrix<double,3,6> &_Jest,
        const Eigen::Matrix<double,18,6> &_Hest,
        const Eigen::Matrix<double,3,18> &_Zest,
        const Eigen::Matrix<double,18,18> &_ZHest,
        bool computeHessian);

    //pre-computes the derivative matrices Jest, Hest, Zest, ZHest
    void computeDerivativesLocal(Eigen::Vector3d &m1, Eigen::Matrix3d C1,
                                 Eigen::Matrix<double,3,6> &_Jest,
                                 Eigen::Matrix<double,18,6> &_Hest,
                                 Eigen::Matrix<double,3,18> &_Zest,
                                 Eigen::Matrix<double,18,18> &_ZHest,
                                 bool computeHessian);

    //perform line search to find the best descent rate (Mohre&Thuente)
    //adapted from NOX
    double lineSearchMT(
        Eigen::Matrix<double,6,1> &increment,
        std::vector<NDTCell<PointSource>*> &source,
        NDTMap<PointTarget> &target) ;

    //perform line search to find the best descent rate (Mohre&Thuente)
    //adapted from NOX
    double lineSearch2D(
        Eigen::Matrix<double,3,1> &increment,
        std::vector<NDTCell<PointSource>*> &source,
        NDTMap<PointTarget> &target) ;

    //auxiliary functions for MoreThuente line search
    struct MoreThuente
    {
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
    /*
    void gradient_numeric(
        std::vector<NDTCell*> &moving,
        NDTMap &fixed,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
        //Eigen::Vector3d &eulerAngles,
        Eigen::Matrix<double,6,1> &score_gradient
        );
    */

protected:
    //storage for pre-computed angular derivatives
    //Eigen::Vector3d jest13, jest23, jest04, jest14, jest24, jest05, jest15, jest25;
    //Eigen::Vector3d a2,a3, b2,b3, c2,c3, d1,d2,d3, e1,e2,e3, f1,f2,f3;
    int NUMBER_OF_ACTIVE_CELLS;
    double normalizeAngle(double a);

    //vars for gradient
    Eigen::Matrix<double,6,1> xtBJ, xtBZBx, Q;
    //vars for hessian
    Eigen::Matrix<double,6,6> JtBJ, xtBZBJ, xtBH, xtBZBZBx, xtBZhBx;
    Eigen::Matrix<double,1,3> TMP1, xtB;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace

#include <impl/ndt_matcher_d2d.hpp>
#endif
