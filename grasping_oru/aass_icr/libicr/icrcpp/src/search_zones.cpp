#include "../include/search_zones.h"
#include "../include/debug.h"
#include "../include/grasp.h"
#include "assert.h"

namespace ICR
{
//-------------------------------------------------------------------
//-------------------------------------------------------------------
PrimitiveSearchZone::PrimitiveSearchZone(){hyperplane_ids_.clear();}
//-------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, PrimitiveSearchZone const& psz)
{
  stream <<'\n'<<"PRIMITIVE SEARCH ZONE: "<<'\n'
         <<"Hyperplane id's: ";
       
    for(uint i=0;i<psz.hyperplane_ids_.size();i++)
        stream<<psz.hyperplane_ids_[i]<<" ";

    stream<<'\n'<<'\n';

  return stream;
}
//-------------------------------------------------------------------
//-------------------------------------------------------------------
SearchZones::SearchZones() : tws_(NULL),num_search_zones_(0),search_zones_computed_(false)
{
  search_zones_.clear();
}
//-------------------------------------------------------------------
SearchZones::SearchZones(const GraspPtr grasp) : grasp_(grasp), tws_(NULL),num_search_zones_(0),search_zones_computed_(false)
{
  assert(grasp_->getGWS()->containsOrigin());
  search_zones_.clear();
}
//-------------------------------------------------------------------
SearchZones::SearchZones(SearchZones const& src) :  grasp_(src.grasp_), tws_(src.tws_), search_zones_(src.search_zones_),
						      num_search_zones_(src.num_search_zones_), search_zones_computed_(src.search_zones_computed_),map_vertex2finger_(src.map_vertex2finger_),
                                                      hyperplane_normals_(src.hyperplane_normals_),hyperplane_offsets_(src.hyperplane_offsets_){}
//-------------------------------------------------------------------
SearchZones& SearchZones::operator=(SearchZones const& src)
{
  if (this !=&src)
    {
      grasp_=src.grasp_;
      tws_=src.tws_;
      search_zones_=src.search_zones_;
      num_search_zones_=src.num_search_zones_;
      search_zones_computed_=src.search_zones_computed_;
      map_vertex2finger_=src.map_vertex2finger_;
      hyperplane_normals_=src.hyperplane_normals_;
      hyperplane_offsets_=src.hyperplane_offsets_;
    }
  return *this;
}
//-------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream,SearchZones const& sz)
{
  stream <<'\n'<<"SEARCH ZONES: "<<'\n'
         <<"Number of patch search zones: " << sz.num_search_zones_<<'\n'
         <<"Search zones computed: "<<std::boolalpha<<sz.search_zones_computed_<<'\n'
         <<"Vertex-finger map: "<<sz.map_vertex2finger_<<'\n';

  return stream;
}
//-------------------------------------------------------------------
SearchZones::~SearchZones(){clear();}
//-------------------------------------------------------------------
SearchZone const* SearchZones::getSearchZone(uint finger_id)const{return search_zones_.at(finger_id);}
//-------------------------------------------------------------------
uint SearchZones::getNumSearchZones()const{return num_search_zones_;}
//-------------------------------------------------------------------
bool SearchZones::searchZonesComputed()const{return search_zones_computed_;}
//-------------------------------------------------------------------
void SearchZones::computeShiftedHyperplanes(double alpha)
{
  double offset=grasp_->getGWS()->getOcInsphereRadius()*alpha;
  tws_=new SphericalWrenchSpace(6,offset);
  uint num_hyperplanes=grasp_->getGWS()->num_facets_;
  hyperplane_normals_.resize(num_hyperplanes,6);
  hyperplane_offsets_.resize(num_hyperplanes);
  facetT* curr_f=grasp_->getGWS()->conv_hull_.beginFacet().getFacetT();
 
  hyperplane_offsets_.fill(offset); 
     
  for(uint hp_id=0; hp_id < num_hyperplanes;hp_id++)
    {
      assert(offset <= (-curr_f->offset)); //Make sure that GWSinit contains the TWS and the hyperplanes are shifted inwards
      //The direction of the normal is switched to inward-pointing in order to be consistent with the positive offset
      hyperplane_normals_.row(hp_id)=(-Eigen::Map<Eigen::Matrix<double,1,6> >(curr_f->normal));
      curr_f=curr_f->next;
    }
}
//-------------------------------------------------------------------
void SearchZones::addShiftedPrimitiveSearchZone(uint finger_id,vertexT const* curr_vtx)
{
  setT *neighbor_facets=curr_vtx->neighbors;
  PrimitiveSearchZone* p_search_zone = new PrimitiveSearchZone();
  p_search_zone->hyperplane_ids_.reserve(qh_setsize(neighbor_facets));

  for(uint i=0; i < (uint)qh_setsize(neighbor_facets); i++)
    p_search_zone->hyperplane_ids_.push_back(((facetT*)neighbor_facets->e[i].p)->id);

  search_zones_[finger_id]->push_back(p_search_zone);
}
//-------------------------------------------------------------------
void SearchZones::clear()
{
  delete tws_;
  tws_=NULL;
  for(uint finger_id=0;finger_id < num_search_zones_;finger_id++)
    {
      for(uint prim_sz_id=0; prim_sz_id < search_zones_[finger_id]->size(); prim_sz_id++)
	delete (*search_zones_[finger_id])[prim_sz_id];            

      delete search_zones_[finger_id];
    }
  search_zones_.clear();
  num_search_zones_ = 0;
  search_zones_computed_ = false;

}
//-------------------------------------------------------------------
void SearchZones::initializeSearchZones()
{
  if(search_zones_computed_) //Clear up possible previously computed search zones
     clear();

  num_search_zones_=grasp_->getNumFingers();
  search_zones_.reserve(num_search_zones_);
  map_vertex2finger_.resize(1,num_search_zones_);

  for(uint finger_id=0; finger_id < num_search_zones_; finger_id++)
    {
      search_zones_.push_back(new SearchZone());
      //reserve for the maximum number of primitive search zones for each finger (i.e all wrenches are vertexes of the gws and span a primitive search zone)
      search_zones_[finger_id]->reserve(grasp_->getFinger(finger_id)->getCenterPointPatch()->patch_ids_.size()
                                       *grasp_->getFinger(finger_id)->getContactModel()->getLimitSurface()->getNumPrimitiveWrenches());

      //the elements of map_vertex2finger_ describe the relation between a vertex-index from the grasp wrench space to the finger this vertex belongs.
      //I.e. if the value of vertex id <= than map_vertex2finger_(0), then the vertex is a wrench belonging to the first finger, if vertex id <= than 
      //map_vertex2finger_(1) the vertex belongs to the second finger and so on. E.g, a 3-fingered grasp comprising patches with 2 contact points each and 10 
      //primitive wrenches in each contact point would have map_vertex2finger_=[19 39 59] 
      map_vertex2finger_(finger_id)=grasp_->getFinger(finger_id)->getCenterPointPatch()->patch_ids_.size()
                            *grasp_->getFinger(finger_id)->getContactModel()->getLimitSurface()->getNumPrimitiveWrenches()-1;
      if (finger_id > 0)
        map_vertex2finger_(finger_id)=map_vertex2finger_(finger_id)+map_vertex2finger_(finger_id-1)+1;
    }
}
//--------------------------------------------------------------------
void SearchZones::computeShiftedSearchZones(double alpha)
{
  assert(alpha > 0 && alpha <= 1);
  assert(grasp_->getGWS()->containsOrigin());
  initializeSearchZones();
  computeShiftedHyperplanes(alpha);

  //iterate through the vertices of the GWS and create the appropriate Primitive Search Zone for each 
  //vertex and push it back on the search zone of its corresponding finger
  orgQhull::QhullVertex curr_vtx=grasp_->getGWS()->conv_hull_.beginVertex();

  for (uint vtx_count=0; vtx_count < grasp_->getGWS()->num_vtx_;vtx_count++)
    {
      //determine to which finger the current vertex belongs and create an appropriate primitive search zone and push it on the list for the corresponding
      //finger
      for(uint finger_id=0; finger_id < num_search_zones_; finger_id++)
	{
	  if((uint)curr_vtx.point().id() <= map_vertex2finger_(finger_id))
	    {
	      addShiftedPrimitiveSearchZone(finger_id,curr_vtx.getVertexT());
	      break;
	    }
        }

      curr_vtx=curr_vtx.next();
    }
  search_zones_computed_=true;
}
//--------------------------------------------------------------------
Eigen::Matrix<double,Eigen::Dynamic,6> const* SearchZones::getHyperplaneNormals()const{return &hyperplane_normals_;}
//--------------------------------------------------------------------
Eigen::VectorXd const* SearchZones::getHyperplaneOffsets()const{return &hyperplane_offsets_;}
//--------------------------------------------------------------------
const GraspPtr SearchZones::getGrasp()const{return grasp_;}
//--------------------------------------------------------------------
WrenchSpace const* SearchZones::getTWS()const
{
  assert(search_zones_computed_);
  return tws_;
}
//--------------------------------------------------------------------
void SearchZones::resetPrimitiveSearchZones(uint sz_id)
{
  for(uint psz_id=0; psz_id < search_zones_[sz_id]->size(); psz_id++)
    (*search_zones_[sz_id])[psz_id]->satisfied_wc_ids_.resize(0);  
}
//--------------------------------------------------------------------
void SearchZones::resetSearchZones()
{
  for(uint sz_id=0;sz_id < num_search_zones_; sz_id++)
    resetPrimitiveSearchZones(sz_id);
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
