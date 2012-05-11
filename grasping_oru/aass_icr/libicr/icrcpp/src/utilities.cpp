#include "../include/utilities.h"
#include "../include/debug.h"
#include "assert.h"

namespace ICR
{
//--------------------------------------------------------------------
Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d vector)
{
  Eigen::Matrix3d skew_sym_m;

  skew_sym_m.row(0) << 0,          -vector(2), vector(1);
  skew_sym_m.row(1) << vector(2),  0,          -vector(0);
  skew_sym_m.row(2) << -vector(1), vector(0),  0;

  return skew_sym_m;
}
//--------------------------------------------------------------------
uint factorial(uint x)
{
  assert(x>=1);
  return (x == 1 ? x : x * factorial(x - 1));
}
//--------------------------------------------------------------------
uint dfactorial(uint x)	 
{
  assert(x>=1);
  return(x <= 2 ? x : x*dfactorial(x-2));
 
}
//--------------------------------------------------------------------
}//namespace ICR
