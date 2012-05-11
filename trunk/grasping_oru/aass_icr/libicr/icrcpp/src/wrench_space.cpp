#include "../include/wrench_space.h"
#include "../include/debug.h"
#include <iostream>
#include <math.h>
#include "assert.h"


namespace ICR
{
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
WrenchSpace::WrenchSpace() : type_(Undefined_WS),full_dim_(false),contains_origin_(false),
                             r_oc_insphere_(0),volume_(0),area_(0),dimension_(6){}
//--------------------------------------------------------------------------
WrenchSpace::WrenchSpace(uint dimension) : type_(Undefined_WS),full_dim_(false),contains_origin_(false),
                             r_oc_insphere_(0),volume_(0),area_(0),dimension_(dimension){}
//--------------------------------------------------------------------------
WrenchSpace::WrenchSpace(const WrenchSpace& src) : type_(src.type_),full_dim_(src.full_dim_),
                                                   contains_origin_(src.contains_origin_), r_oc_insphere_(src.r_oc_insphere_),
                                                   volume_(src.volume_), area_(src.area_),dimension_(src.dimension_){}
//--------------------------------------------------------------------------
WrenchSpace& WrenchSpace::operator=(const WrenchSpace& src)		  
{
  if (this !=&src)
    {
      type_=src.type_;
      full_dim_=src.full_dim_;
      contains_origin_=src.contains_origin_;
      r_oc_insphere_=src.r_oc_insphere_;
      volume_=src.volume_;
      area_=src.area_;
      dimension_=src.dimension_;
    }
  return *this;
}
//--------------------------------------------------------------------------
WrenchSpace::~WrenchSpace() {}
//--------------------------------------------------------------------------
WrenchSpaceType WrenchSpace::getWrenchSpaceType()const {return type_;}
//--------------------------------------------------------------------------
bool WrenchSpace::isFullDimension()const{return full_dim_;}
//--------------------------------------------------------------------------
bool WrenchSpace::containsOrigin()const{return contains_origin_;}
//--------------------------------------------------------------------------
double WrenchSpace::getOcInsphereRadius()const{return r_oc_insphere_;}
//--------------------------------------------------------------------------
double WrenchSpace::getVolume()const{return volume_;}
//--------------------------------------------------------------------------
double WrenchSpace::getArea()const{return area_;}
//--------------------------------------------------------------------------
uint WrenchSpace::getDimension()const{return dimension_;}
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
SphericalWrenchSpace::SphericalWrenchSpace() : WrenchSpace(), radius_(0)
{
  type_=Spherical;
}
//--------------------------------------------------------------------------
SphericalWrenchSpace::SphericalWrenchSpace(uint dimension,double radius) : WrenchSpace(dimension),radius_(radius)
{
  assert(radius>0);
  assert(dimension > 1);
  type_=Spherical;
  full_dim_=true;
  contains_origin_=true; 
  r_oc_insphere_=radius; 
  computeVolume();
  computeArea();
}
//--------------------------------------------------------------------------
SphericalWrenchSpace::SphericalWrenchSpace(SphericalWrenchSpace const& src) : WrenchSpace(src), radius_(src.radius_){}
//--------------------------------------------------------------------------
SphericalWrenchSpace& SphericalWrenchSpace::operator=(SphericalWrenchSpace const& src)
{
  if (this !=&src)
    {
      WrenchSpace::operator=(src); 
      radius_=src.radius_;   
    }
  return *this;
}  
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, SphericalWrenchSpace const& s_wrench_space)
{
  std::string wrench_space_type;
 if (s_wrench_space.type_==Spherical) wrench_space_type="Spherical";
  else wrench_space_type="Warning in SphericalWrenchSpace: Invalid rule type!";

  stream <<'\n'<<"SPHERICAL WRENCH SPACE: "<<'\n'
	 <<"Wrench space type: "<<wrench_space_type<<'\n'
         <<"Dimension: "<< s_wrench_space.dimension_<<'\n'
         <<"Contains origin: "<<s_wrench_space.contains_origin_<<'\n'
	 <<"Has full dimension: "<<s_wrench_space.full_dim_<<'\n'
	 <<"Radius: "<<s_wrench_space.radius_<<'\n'
	 <<"Volume: "<<s_wrench_space.volume_<<'\n'
	 <<"Area: "<<s_wrench_space.area_<<'\n'	
         <<"Origin-centered insphere radius: "<<s_wrench_space.r_oc_insphere_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------------
SphericalWrenchSpace::~SphericalWrenchSpace(){}
//--------------------------------------------------------------------------
void SphericalWrenchSpace::computeArea()
{
  double C_n;

  if ( dimension_ % 2== 0 )
    C_n=pow(PI,dimension_/2)/factorial(dimension_/2);
  else
    C_n=pow(2,(dimension_+1)/2)*pow(PI,(dimension_-1)/2)/dfactorial(dimension_);

  area_=dimension_*C_n*pow(radius_,dimension_-1);
}
//--------------------------------------------------------------------------
void SphericalWrenchSpace::computeVolume()
{
  double C_n;

  if ( dimension_ % 2== 0 )
    C_n=pow(PI,dimension_/2)/factorial(dimension_/2);
  else
    C_n=pow(2,(dimension_+1)/2)*pow(PI,(dimension_-1)/2)/dfactorial(dimension_);

  volume_=C_n*pow(radius_,dimension_);
}
//--------------------------------------------------------------------------
void SphericalWrenchSpace::setRadius(double const radius)
{
  assert(radius >0);
  radius_=radius;
  computeVolume();
  computeArea();
 

  r_oc_insphere_=radius;
  contains_origin_=true;
}
double SphericalWrenchSpace::getRadius()const{return radius_;}
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace() : ch_computed_(false),num_wrenches_(0),num_vtx_(0),num_facets_(0)
{
type_=Discrete;
}
//--------------------------------------------------------------------------
DiscreteWrenchSpace::DiscreteWrenchSpace(DiscreteWrenchSpace const& src) : WrenchSpace(src),ch_computed_(src.ch_computed_),
									   conv_hull_(src.conv_hull_),num_wrenches_(src.num_wrenches_),
									   num_vtx_(src.num_vtx_),num_facets_(src.num_facets_){}
//--------------------------------------------------------------------------
DiscreteWrenchSpace& DiscreteWrenchSpace::operator=(DiscreteWrenchSpace const& src)
{
  if (this !=&src)
    {
     WrenchSpace::operator=(src); 
     ch_computed_=src.ch_computed_;
     conv_hull_=src.conv_hull_;
     num_wrenches_=src.num_wrenches_;
     num_vtx_=src.num_vtx_;
     num_facets_=src.num_facets_;
    }
  return *this;
}
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, DiscreteWrenchSpace const& d_wrench_space)
{
  std::string wrench_space_type;
 if (d_wrench_space.type_==Discrete) wrench_space_type="Discrete";
  else wrench_space_type="Warning in DiscreteWrenchSpace: Invalid rule type!";

  stream <<'\n'<<"DISCRETE WRENCH SPACE: "<<'\n'
	 <<"Wrench space type: "<<wrench_space_type<<'\n'
         <<"Convex hull computed: "<<d_wrench_space.ch_computed_<<'\n'
         <<"Contains origin: "<<d_wrench_space.contains_origin_<<'\n'
	 <<"Has full dimension: "<<d_wrench_space.full_dim_<<'\n'
	 <<"Origin-centered insphere radius: "<<d_wrench_space.r_oc_insphere_<<'\n'
	 <<"Volume: "<<d_wrench_space.volume_<<'\n'
	 <<"Area: "<<d_wrench_space.area_<<'\n'
         <<"Number of input wrenches: "<<d_wrench_space.num_wrenches_<<'\n'
    	 <<"Number of vertices: "<<d_wrench_space.num_vtx_<<'\n' 
         <<"Number of facets: "<<d_wrench_space.num_facets_<<'\n'<<'\n';
    
  return stream;
}
//--------------------------------------------------------------------------
DiscreteWrenchSpace::~DiscreteWrenchSpace(){}
//--------------------------------------------------------------------------
bool DiscreteWrenchSpace::convHullComputed()const{return ch_computed_;}
//--------------------------------------------------------------------------
orgQhull::Qhull const* DiscreteWrenchSpace::getConvexHull()const
{
  assert(ch_computed_);
  return &conv_hull_;
}
//--------------------------------------------------------------------------
void DiscreteWrenchSpace::computeConvexHull(double const* wrenches,uint num_wrenches)
{
  assert(wrenches != NULL);
  assert(num_wrenches > dimension_);
  num_wrenches_=num_wrenches;

  try{
    conv_hull_.runQhull("", dimension_,num_wrenches,wrenches ,"Qx Qt");
  }
  catch(std::exception& exc)
    {
#ifdef DEBUG_QHULL
      std::cout<<exc.what()<<std::endl;
#endif
      contains_origin_=false;
      full_dim_=false;
      ch_computed_=true;
      return;
    }

  conv_hull_.defineVertexNeighborFacets();
  area_=conv_hull_.area();
  volume_=conv_hull_.volume();
  num_vtx_=conv_hull_.vertexCount();
  num_facets_=conv_hull_.facetCount();
 
  facetT* curr_f=conv_hull_.beginFacet().getFacetT();
  r_oc_insphere_=-(curr_f->offset);

#ifdef DEBUG_DISCRETEWRENCHSPACE // File for writing the hyperplanes to a file to compare in Matlab
  remove("../debug/hyperplanes.txt");
  FILE* hp=fopen ("../debug/hyperplanes.txt","a");
  if(!hp)
    std::cout<<"Warning in DiscreteWrenchSpace: Couldn't write to file."<<std::endl;
#endif

  for(uint i=0;i< num_facets_;i++)
    {
#ifdef DEBUG_DISCRETEWRENCHSPACE //Write hyperplanes to file
      fprintf(hp, "% f % f % f % f % f % f %f \n",(curr_f->normal)[0],(curr_f->normal)[1],(curr_f->normal)[2],(curr_f->normal)[3],(curr_f->normal)[4],(curr_f->normal)[5],curr_f->offset);
#endif
      curr_f->id=i; //Replaces the Qhull runtime indexing with indices 0 - num_facets_
      r_oc_insphere_ = (-(curr_f->offset) < r_oc_insphere_) ? -(curr_f->offset) : r_oc_insphere_;
      curr_f=curr_f->next;
    }
  
  contains_origin_=(r_oc_insphere_ > EPSILON_FORCE_CLOSURE) ? true : false;
  full_dim_=true;
  ch_computed_=true;
 
#ifdef DEBUG_DISCRETEWRENCHSPACE
  fclose (hp);
#endif
}
//--------------------------------------------------------------------------
uint DiscreteWrenchSpace::getNumWrenches()const{return num_wrenches_;}
//--------------------------------------------------------------------------
uint DiscreteWrenchSpace::getNumVertices()const{return num_vtx_;}
//--------------------------------------------------------------------------
uint DiscreteWrenchSpace::getNumFacets()const{return num_facets_;}
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
}//namespace ICR
