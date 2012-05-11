#ifndef ows_h___
#define ows_h___

#include "utilities.h"
#include "target_object.h"
#include "limit_surface.h"
#include <string>
#include <iostream>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Holds a list of ICR::WrenchCone pointers describing the Object Wrench Space of the parent object
 */
class OWS 
{
 private:

  WrenchConeList wrench_cones_;
  std::string parent_obj_name_;
  bool initialized_;
  bool div_by_lambda_;
  LimitSurface lim_surf_;
  double lambda_;
  uint num_wc_;
/*! 
 * Creates a new wrench cone by transforming the local wrench cone member of ICR::OWS#lim_surf_
 * according to "Murray, Li & Sastry - A Mathematical Introduction to Robotic Manipulation; pp. 218"
 * and pushes it onto ICR::OWS::wrench_cones_
 */
  void addWrenchCone(uint id,Eigen::Vector3d const* const cp_vtx, Eigen::Vector3d const* const cp_vtx_normal);
  void updateLambda(Eigen::Vector3d const* const cp_vtx);

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OWS();
  OWS(OWS const& src);
  OWS& operator=(OWS const& src);
  friend std::ostream& operator<<(std::ostream& stream, OWS const& ows);
  ~OWS();

  void init(TargetObject const& obj,LimitSurface const& lim_surf);
  uint getNumWrenchCones() const;
  WrenchCone const* getWrenchCone(uint id) const;
  LimitSurface const* getLimitSurface() const; 
  void scale(double scale);
  void divideByLambda();  
  std::string getParentObjectName()const;
};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
#endif
