#ifndef wrench_cone_h___
#define wrench_cone_h___

#include "utilities.h"
#include <iostream>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Holds a matrix of prmimitve wrenches; The wrench cone id is the same as the id of the corresponding contact point 
 *
 */
class WrenchCone
{
 private:

  uint num_primitive_wrenches_;
  Eigen::Matrix<double,6,Eigen::Dynamic> cone_;
  uint id_;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class IndependentContactRegions;

  WrenchCone();
  WrenchCone(uint id);
  WrenchCone(uint id,Eigen::Matrix<double,6,Eigen::Dynamic> const& cone);
  WrenchCone(WrenchCone const& src);
  WrenchCone& operator=(WrenchCone const& src);
  friend std::ostream& operator<<(std::ostream& stream,WrenchCone const& wrench_cone);
  ~WrenchCone();

  Eigen::Matrix<double,6,Eigen::Dynamic> const* getWrenches()const;
  Eigen::Matrix<double,6,Eigen::Dynamic>* getWrenches();
  uint getNumPrimitiveWrenches() const;
  void scaleWrenches(double scale);
  void scaleWrenchTorques(double scale);
  void addWrenches(Eigen::Matrix<double,6,Eigen::Dynamic> const& wrenches);
  void setWrenches(Eigen::Matrix<double,6,Eigen::Dynamic> const& wrenches);
  void setId(uint id);
  uint getId()const;
};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
#endif
