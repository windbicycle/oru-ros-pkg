#ifndef wrench_space_h___
#define wrench_space_h___

#include "utilities.h" 
#include <Eigen/Core>
#include <iostream>
#include "Qhull.h"

namespace ICR
{
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
/*! 
 *  \brief Base class from which ICR::SphericalWrenchSpace and ICR::DiscreteWrenchSpace are derived
 */
class WrenchSpace
{
 protected:
  
  WrenchSpaceType type_;
/*! 
 *  If true, the convex hull of the wrench space spans 6D-space
 */
  bool full_dim_;
  bool contains_origin_;
/*! 
 *  The radius of the largest origin-centered ball contained by the convex hull of the wrench space
 */
  double r_oc_insphere_;
  double volume_;
  double area_;
  uint dimension_;  

 public:

 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  friend class SearchZones;

  WrenchSpace();
  WrenchSpace(uint dimension);
  WrenchSpace(WrenchSpace const& src);
  WrenchSpace& operator=(WrenchSpace const& src);
  
  virtual ~WrenchSpace();
  
  WrenchSpaceType getWrenchSpaceType()const;
  bool isFullDimension()const;
  bool containsOrigin()const;
  double getOcInsphereRadius()const;
  double getVolume()const;
  double getArea()const;
  uint getDimension()const;
 
};
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
/*! 
 *  \brief A 6D-sphere used to describe a continous Task Wrench Space in ICR::SearchZones
 */
class SphericalWrenchSpace : public WrenchSpace
{
 private:

  double radius_;
  void computeArea();
  void computeVolume();
  
 public:

  friend class SearchZones;

  SphericalWrenchSpace();
  SphericalWrenchSpace(uint dimension,double radius);
  SphericalWrenchSpace(SphericalWrenchSpace const& src);
  SphericalWrenchSpace& operator=(SphericalWrenchSpace const& src);
  friend std::ostream& operator<<(std::ostream& stream, SphericalWrenchSpace const& s_wrench_space);
  virtual ~SphericalWrenchSpace();

  void setRadius(double const radius);
  double getRadius()const;
};
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
/*! 
 *  \brief A discrete 6D-wrench space used to describe the Grasp Wrench Space in ICR::Grasp
 */
class DiscreteWrenchSpace : public WrenchSpace
{

 private:

  bool ch_computed_;
  orgQhull::Qhull conv_hull_;
  uint num_wrenches_;
  uint num_vtx_;
  uint num_facets_;

 public:

 friend class SearchZones;

  DiscreteWrenchSpace();
  DiscreteWrenchSpace(DiscreteWrenchSpace const& src);
  DiscreteWrenchSpace& operator=(DiscreteWrenchSpace const& src);
  virtual ~DiscreteWrenchSpace();
  friend std::ostream& operator<<(std::ostream& stream,DiscreteWrenchSpace const& d_wrench_space);
/*! 
 *  Uses Qhull to compute the convex hull over wrenches
 */
  void computeConvexHull(double const* wrenches,uint num_wrenches);
  orgQhull::Qhull const* getConvexHull()const;
 bool convHullComputed()const;
 uint getNumWrenches()const;
 uint getNumVertices()const;
 uint getNumFacets()const;
};
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
}//namespace ICR
#endif
