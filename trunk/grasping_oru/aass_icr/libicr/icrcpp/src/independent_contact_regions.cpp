#include "../include/independent_contact_regions.h"
#include "../include/wrench_cone.h"
#include "../include/search_zones.h"
#include "../include/ows.h"
#include "../include/grasp.h"
#include "../include/target_object.h"
#include "../include/contact_point.h"
#include "../include/debug.h"
#include "../include/config.h"
#include "assert.h"
#include <thread>

namespace ICR
{

IndependentContactRegions::IndependentContactRegions() :  
  icr_computed_(false),
  num_contact_regions_(0) {contact_regions_.clear();}
//--------------------------------------------------------------------
IndependentContactRegions::IndependentContactRegions(const SearchZonesPtr search_zones,const GraspPtr grasp) : 
  search_zones_(search_zones), 
  grasp_(grasp),
  icr_computed_(false), 
  num_contact_regions_(0)
{
  contact_regions_.clear();
  assert((bool)search_zones_);
  assert((bool)grasp_);
}
//--------------------------------------------------------------------
IndependentContactRegions::IndependentContactRegions(IndependentContactRegions const& src) : 
  search_zones_(src.search_zones_), 
  grasp_(src.grasp_), 
  icr_computed_(src.icr_computed_),
  contact_regions_(src.contact_regions_),
  num_contact_regions_(src.num_contact_regions_){}
//--------------------------------------------------------------------
IndependentContactRegions& IndependentContactRegions::operator=(IndependentContactRegions const& src)
{
  if (this !=&src)
    {
      search_zones_=src.search_zones_;
      grasp_=src.grasp_;
      icr_computed_=src.icr_computed_;
      contact_regions_=src.contact_regions_;
      num_contact_regions_=src.num_contact_regions_;
    }

  return *this;
}

//--------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, IndependentContactRegions const& icr)
{
  stream <<'\n'<<"IndependentContactRegions: "<<'\n'
         <<"Number of contact regions: " << icr.num_contact_regions_<<'\n'
         <<"Contact regions computed: "<<std::boolalpha<<icr.icr_computed_<<'\n';
  if(icr.icr_computed_)
    {
      for(uint i=0;i<icr.num_contact_regions_;i++)
	{
	  stream<<"Centerpoint id's region "<<i<<": ";
	  for(uint j=0; j < icr.contact_regions_[i]->size();j++)
	    {
	      stream<<(*icr.contact_regions_[i])[j]->patch_ids_.front()<<" ";
	    }
	  stream<<'\n';
        }
    }
  stream<<'\n';
  return stream;
}
//--------------------------------------------------------------------
IndependentContactRegions::~IndependentContactRegions(){clear();}
//--------------------------------------------------------------------
void IndependentContactRegions::clear()
{
  for(uint i=0;i<contact_regions_.size();i++)
    {
      delete contact_regions_[i];
    }
  contact_regions_.clear();
  num_contact_regions_ = 0;
  icr_computed_ = false;
}
//--------------------------------------------------------------------
  bool IndependentContactRegions::primitiveSearchZoneInclusionTest(PrimitiveSearchZone* prim_sz,WrenchCone const* wc)const
  {
    if ((prim_sz->satisfied_wc_ids_.array() == wc->id_).any())
      return true;
   

    bool constraints_satisfied;
    for(uint pw_count=0; pw_count < wc->num_primitive_wrenches_;pw_count++)
      {
	constraints_satisfied=true;
	for(uint hp_count=0; hp_count< prim_sz->hyperplane_ids_.size(); hp_count++)
	  {
	    if(search_zones_->hyperplane_normals_.row(prim_sz->hyperplane_ids_[hp_count]) * wc->cone_.col(pw_count)  + search_zones_->hyperplane_offsets_(prim_sz->hyperplane_ids_[hp_count]) > 0)
	      {
		constraints_satisfied=false;
		break;
	      }
	  }
	if(constraints_satisfied)
	  {
	    prim_sz->satisfied_wc_ids_.conservativeResize(prim_sz->satisfied_wc_ids_.size()+1);
	    prim_sz->satisfied_wc_ids_(prim_sz->satisfied_wc_ids_.size()-1)=wc->id_;
	    return true;
	  }
      }
    return false;
  }
//--------------------------------------------------------------------
bool IndependentContactRegions::searchZoneInclusionTest(uint region_id,Patch const* patch)const
{

  bool psz_satisfied;
  for(uint psz_id=0; psz_id < search_zones_->search_zones_[region_id]->size();psz_id++)//iterate over all primitive search zones of the queried search zone
    {

      //check if a wrench cone associated with any point in the patch satisfies the primitiveSearchZoneInclusionTest
      psz_satisfied=false;
      for(ConstIndexListIterator patch_point=patch->patch_ids_.begin(); patch_point != patch->patch_ids_.end(); patch_point++)
	{
	if(primitiveSearchZoneInclusionTest((*search_zones_->search_zones_[region_id])[psz_id] ,grasp_->getFinger(region_id)->getOWS()->getWrenchCone(*patch_point)))
	  {
	    psz_satisfied=true;
	    break;
	  }
	}
      if(!psz_satisfied)
	return false;
    }     
  return true;
}
//--------------------------------------------------------------------
void IndependentContactRegions::computeContactRegion(uint region_id)
{
  search_zones_->resetPrimitiveSearchZones(region_id); //Make sure the member ICR::PrimitiveSearchZone::satisfied_wc_ids_ is empty
  std::list<Node> nodes; //List of nodes for the breadth-first-search on the target object's vertices
  VectorXui explored_cp=VectorXui::Zero(grasp_->getParentObj()->getNumCp(),1); //set all entries to NOT_EXPLORED
  uint init_centerpoint_id=grasp_->getFinger(region_id)->getCenterPointPatch()->patch_ids_.front();
  
  contact_regions_[region_id]->reserve(grasp_->getParentObj()->getNumCp());
  nodes.push_back(Node(const_cast<ContactPoint*>(grasp_->getParentObj()->getContactPoint(init_centerpoint_id)))); //push the node corresponding to the centerpoint of the patch associated with the initial prototype grasp contact
  contact_regions_[region_id]->push_back(const_cast<Patch*>(grasp_->getFinger(region_id)->getCenterPointPatch())); 
  explored_cp(init_centerpoint_id)=EXPLORED_QUALIFIED;
 
  while(nodes.size() > 0)
    {   
      for(ConstIndexListIterator neighbor=nodes.front().contact_point_->getNeighborItBegin(); neighbor != nodes.front().contact_point_->getNeighborItEnd(); neighbor++)      
	{
	  if(explored_cp(*neighbor)==NOT_EXPLORED)
	    {	     
 	      if(searchZoneInclusionTest(region_id,grasp_->getFinger(region_id)->getPatch(*neighbor)))
		{
		  nodes.push_back(Node(const_cast<ContactPoint*>(grasp_->getParentObj()->getContactPoint(*neighbor))));
                  contact_regions_[region_id]->push_back(const_cast<Patch*>(grasp_->getFinger(region_id)->getPatch(*neighbor))); 
		  explored_cp(*neighbor)=EXPLORED_QUALIFIED;
		}
	      else
	        explored_cp(*neighbor)=EXPLORED_UNQUALIFIED;
	    }
	}
      nodes.pop_front();
    }
}
//--------------------------------------------------------------------
void IndependentContactRegions::computeICR()
{
  assert((bool)search_zones_);
  assert((bool)grasp_);

  if(icr_computed_) //clean up possible previously computed contact regions
    clear();

  num_contact_regions_=search_zones_->num_search_zones_;
  contact_regions_.reserve(num_contact_regions_);
 
#ifdef MULTITHREAD_ICR_COMPUTATION
  std::vector<std::thread*> threads;
  threads.reserve(num_contact_regions_);
  for(uint region_id=0; region_id < num_contact_regions_; region_id++)
    {
      contact_regions_.push_back(new ContactRegion);
      threads.push_back(new std::thread(&IndependentContactRegions::computeContactRegion,this,region_id));
    }
  for(uint thread_id=0; thread_id < num_contact_regions_; thread_id++)
    {
      threads[thread_id]->join();
      delete threads[thread_id];
    }
#else
  for(uint region_id=0; region_id < num_contact_regions_; region_id++)
    {
      contact_regions_.push_back(new ContactRegion);
      computeContactRegion(region_id);
    }
#endif

  icr_computed_=true;
}
//--------------------------------------------------------------------
bool IndependentContactRegions::icrComputed()const{return icr_computed_;}
//--------------------------------------------------------------------
ContactRegion const* IndependentContactRegions::getContactRegion(uint id)const{return contact_regions_.at(id);}
//--------------------------------------------------------------------
uint IndependentContactRegions::getNumContactRegions()const{return num_contact_regions_;}
//--------------------------------------------------------------------
const SearchZonesPtr IndependentContactRegions::getSearchZones()const{return search_zones_;}
//--------------------------------------------------------------------
void IndependentContactRegions::setSearchZones(SearchZonesPtr sz_in)
{
  clear();
  search_zones_ = sz_in;
}
//--------------------------------------------------------------------
void IndependentContactRegions::setGrasp(GraspPtr g_in) 
{
  clear();
  grasp_ = g_in;
}
//--------------------------------------------------------------------
  bool IndependentContactRegions::hasInitializedGrasp() {
    return (grasp_ != NULL && grasp_->isInitialized());
  }

}//namespace ICR
