#ifndef search_zones_h___
#define search_zones_h___

#include <iostream>
#include "utilities.h"
#include "Qhull.h"

namespace ICR
{
//-------------------------------------------------------------------
//-------------------------------------------------------------------
/*! 
 *  \brief Used in ICR::IndependentContactRegions#primitiveSearchZoneInclusionTest to determine
 *  whether a contact point associated to a wrench cone is eligible for inclusion in the independent regions
 */
struct PrimitiveSearchZone
{
 /*!
 * Idexes the hyperplanes in ICR::SearchZones#hyperplane_normals_ and ICR::SearchZones#hyperplane_offsets_
 */
  std::vector<uint> hyperplane_ids_;
  PrimitiveSearchZone();
 /*!
 * Helper for ICR::IndependentContactRegions#primitiveSearchZoneInclusionTest; stores the indices of already explored wrench cones;
 */
  VectorXui satisfied_wc_ids_;

  friend std::ostream& operator<<(std::ostream& stream, PrimitiveSearchZone const& psz);
};
//-------------------------------------------------------------------
//-------------------------------------------------------------------
/*! 
 *  \brief Holds a std::vector of ICR::SearchZone pointers, one for each finger of the associated prototype grasp.
 */
class SearchZones
{
 private:

  GraspPtr grasp_;
  WrenchSpace* tws_;
  std::vector<SearchZone*> search_zones_;
  uint num_search_zones_;
  bool search_zones_computed_;
  RowVectorXui map_vertex2finger_;
  Eigen::Matrix<double,Eigen::Dynamic,6> hyperplane_normals_;
  Eigen::VectorXd hyperplane_offsets_;  

  void computeShiftedHyperplanes(double alpha);
  void initializeSearchZones();
  void addShiftedPrimitiveSearchZone(uint finger_id,vertexT const* curr_vtx);
  void resetPrimitiveSearchZones(uint sz_id);    
  void clear();
  SearchZones();

 public:

  friend class IndependentContactRegions;

  SearchZones(const GraspPtr grasp);
  SearchZones(SearchZones const& src);
  SearchZones& operator=(SearchZones const& src);
  friend std::ostream& operator<<(std::ostream& stream,SearchZones const& sz);
  ~SearchZones();

/*! 
 *  \brief Creates ICR::SearchZones by shifting the hyperplanes described in
 *  ICR::SearchZones#hyperplane_normals_ and ICR::SearchZones#hyperplane_offsets_ until their are
 *  tangent to a contnous Task Wrench Space described by alpha
 */
  void computeShiftedSearchZones(double alpha);
  const GraspPtr getGrasp()const;
  SearchZone const* getSearchZone(uint finger_id)const;
  uint getNumSearchZones()const;
  bool searchZonesComputed()const;
/*! 
 *  \brief Empties the vectors ICR::PrimitiveSearchZone::satisfied_wc_ids_ of all primitive search
 *  zones; This is necessary when transferring search zones which were utilized to compute ICR for
 *  one object to a novel object; Note that its child function
 *  ICR::SearchZones#resetPrimitiveSearchZones is automatically called by
 *  ICR::IndependentContactRegions#computeContactRegion and thus doesn't need to be called
 *  explicitly;
 */
  void resetSearchZones();
  Eigen::Matrix<double,Eigen::Dynamic,6> const* getHyperplaneNormals()const;
  Eigen::VectorXd const*getHyperplaneOffsets()const;
  WrenchSpace const* getTWS()const;
};
//-------------------------------------------------------------------
//-------------------------------------------------------------------
}//namespace ICR
#endif
