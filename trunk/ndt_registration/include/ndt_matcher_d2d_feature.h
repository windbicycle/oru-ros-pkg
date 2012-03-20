#ifndef NDTMATCHERFEATUREF2F_HH
#define NDTMATCHERFEATUREF2F_HH

#include <ndt_matcher_d2d.h>
namespace lslgeneric
{
  /**
   * This class implements NDT / NDT registration with a priory known correspondances.
   */
    template <typename PointSource, typename PointTarget>
     class NDTMatcherFeatureD2D : public lslgeneric::NDTMatcherD2D<PointSource,PointTarget>
     {
     public:
	  NDTMatcherFeatureD2D(const std::vector<std::pair<int, int> > &corr) : _corr(corr) {}

	  /**
	   * Registers a point cloud to an NDT structure.
	   * \param  target
	   *   Reference data.
	   * \param  source
	   *   The output transformation registers this point cloud to \c target.
	   * \param  T
	   *   This is an input/output parameter. The initial value of \c T
	   *   gives the initial pose estimate of \c source. When the
	   *   algorithm terminates, \c T holds the registration result.
	  bool match( lslgeneric::NDTMap<PointTarget>& target, 
		      lslgeneric::NDTMap<PointSource>& source,
		      Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T);
	   */
	  /**
	   * computes the covariance of the match between moving and fixed, at T.
	   * result is returned in cov
	   */
	  bool covariance( lslgeneric::NDTMap<PointTarget>& target, 
			   lslgeneric::NDTMap<PointSource>& source,
			   Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
			   Eigen::Matrix<double,6,6> &cov
	       );
	  
	  //compute the score of a point cloud to an NDT
	  virtual double scoreNDT(std::vector<lslgeneric::NDTCell<PointSource>*> &source, 
			  lslgeneric::NDTMap<PointTarget> &target,
			  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T);
	
	  //compute the score gradient & hessian of a point cloud + transformation to an NDT
	  // input: moving, fixed, tr, computeHessian
	  //output: score_gradient, Hessian 
	  virtual void derivativesNDT( 
	       std::vector<lslgeneric::NDTCell<PointSource>*> &source,
	       lslgeneric::NDTMap<PointTarget> &target,
	       Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
	       Eigen::Matrix<double,6,1> &score_gradient,
	       Eigen::Matrix<double,6,6> &Hessian,
	       bool computeHessian
	       );

	  virtual bool update_gradient_hessian(
	       Eigen::Matrix<double,6,1> &score_gradient, 
	       Eigen::Matrix<double,6,6> &Hessian, 
	       
	       Eigen::Vector3d &m1, 
	       Eigen::Matrix3d &C1); 


	  using NDTMatcherD2D<PointSource,PointTarget>::Jest;
	  using NDTMatcherD2D<PointSource,PointTarget>::Hest;
	  using NDTMatcherD2D<PointSource,PointTarget>::Zest;
	  using NDTMatcherD2D<PointSource,PointTarget>::ZHest;
	  using NDTMatcherD2D<PointSource,PointTarget>::lfd1;
	  using NDTMatcherD2D<PointSource,PointTarget>::lfd2;
	  using NDTMatcherD2D<PointSource,PointTarget>::normalizeAngle;
	  using NDTMatcherD2D<PointSource,PointTarget>::NUMBER_OF_ACTIVE_CELLS;
     protected:
	  const std::vector<std::pair<int, int> > & _corr;
     };
} // namespace

#include <impl/ndt_matcher_d2d_feature.hpp>

#endif
