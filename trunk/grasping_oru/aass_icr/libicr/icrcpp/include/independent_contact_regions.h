#ifndef independent_contact_regions_h___
#define independent_contact_regions_h___

#include <iostream>
#include "utilities.h"

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Holds shared pointers to the prototype ICR::Grasp and
 *  previously computed ICR::SearchZones; checks the contact points on
 *  the target object's surface for inclusion in the independent
 *  regions
 */
class IndependentContactRegions
{

 private:

  SearchZonesPtr search_zones_;
  GraspPtr grasp_;
  bool icr_computed_;
  std::vector<ContactRegion*> contact_regions_;
  uint num_contact_regions_;
/*! 
 *  Returns true if at least one of the primitive wrenches in wc is contained in the exterior
 *  half-space of each hyperplane defined by prim_sz; Here, an exterior half-space of a hyperplane
 *  is the half-space which does not contain the origin;
 */
  bool primitiveSearchZoneInclusionTest(PrimitiveSearchZone* prim_sz,WrenchCone const* wc)const;
  bool searchZoneInclusionTest(uint region_id,Patch const* patch)const;
  void computeContactRegion(uint region_id);

 public:
  IndependentContactRegions();
  IndependentContactRegions(const SearchZonesPtr search_zones,const GraspPtr grasp);
  /** \brief NOTE: Performes shallow copy of icr.  */
  IndependentContactRegions(IndependentContactRegions const& src);
  /** \brief NOTE: Performes shallow copy of icr.  */
  IndependentContactRegions& operator=(IndependentContactRegions const& src);
  friend std::ostream& operator<<(std::ostream& stream, IndependentContactRegions const& icr);
  ~IndependentContactRegions();

  void clear();
  void computeICR();
  bool icrComputed()const;
  ContactRegion const* getContactRegion(uint id)const;
  uint getNumContactRegions()const;
  const SearchZonesPtr getSearchZones()const;
  const GraspPtr getGrasp()const {return grasp_;}
  GraspPtr getGrasp() {return grasp_;}
  /** \brief True if exists pointer to an initialized grasp \ref ICR::Grasp::init()
   */
  bool hasInitializedGrasp(); 
  /** \brief Sets new search zones for this icr. Clears previously
   *   calculated regions.
   */
 void setSearchZones(SearchZonesPtr sz_in); 

  /** \brief Sets new grasp for this icr. Clears previously calculated
   *   regions.
   */
 void setGrasp(GraspPtr g_in); };
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//;namespace ICR
#endif
