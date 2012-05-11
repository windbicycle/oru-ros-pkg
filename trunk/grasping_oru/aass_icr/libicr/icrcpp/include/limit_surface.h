#ifndef limit_surface_h___
#define limit_surface_h___
#include "wrench_cone.h"
#include <string>
#include <iostream>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief A disc parametrized by the contact force magnitude, the friction coefficient and the
 *  torsional friction coefficient according to "Howe, R.D and Cutkosky, M - Practical Force-Motion
 *  Models for Sliding Manipulation; IJRR,1996"
 */
class LimitSurface
{
 private:

  double force_magnitude_;
/*! 
 *  Discretization of the corresponding friction cone; the total number of primitive wrenches associated with a contact point/wrench cone is
 *  disc_ + 1 (wrench produced by the contact normal force) + 2 (only if the Soft_Finger contact type is applied)
 */
  int disc_;
  double mu_0_;
  double mu_T_;
/*! 
 *  Local cone which is rotated and shifted to form the wrench cones building up ICR::OWS#wrench_cones_
 */
  WrenchCone local_cone_;
  ContactType contact_type_;
/*! 
 *  Selection matrix for the wrench cones according to "Murray, Li & Sastry - A Mathematical
 *  Introduction to Robotic Manipulation; pp. 219"
 */
  Eigen::Matrix<double,6,4> selection_matrix_;
  
  void initializeSelectionMatrix();
  void addFrictionlessWrench();
  void addHardFingerWrenches();
  void addSoftFingerWrenches();

  LimitSurface();
  friend class PointContactModel;
  friend class OWS;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LimitSurface(double force_magnitude);  
  LimitSurface(double force_magnitude, int disc, double mu_0);  
  LimitSurface(double force_magnitude, int disc, double mu_0, double mu_T);
  LimitSurface(LimitSurface const& src);
  LimitSurface& operator=(LimitSurface const& src);
  bool operator==(LimitSurface const& other)const;
  friend std::ostream& operator<<(std::ostream& stream, LimitSurface const& lim_surf);
  ~LimitSurface();

  double getMu0() const;
  double getMuT() const;
  double getForceMagnitude() const;
  int getDisc()const;
  int getNumPrimitiveWrenches() const;
  WrenchCone const* getLocalWrenchCone() const;
  ContactType getContactType() const;
};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
#endif
