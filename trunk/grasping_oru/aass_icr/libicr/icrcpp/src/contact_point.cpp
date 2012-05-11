#include "../include/contact_point.h"
#include "../include/debug.h"

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
ContactPoint::ContactPoint() : id_(0){neighbors_.clear();}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(double const vertex[3]) : vertex_(Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex))),id_(0){neighbors_.clear();}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(Eigen::Vector3d const& vertex) : vertex_(vertex),id_(0) {neighbors_.clear();}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(double const vertex[3],uint id) : vertex_(Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex))),id_(id){neighbors_.clear();}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(Eigen::Vector3d const& vertex,uint id) : vertex_(vertex),id_(id) {neighbors_.clear();}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(double const vertex[3],double const vertex_normal[3],uint id) : vertex_(Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex))), vertex_normal_(Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex_normal))),id_(id)
{
  neighbors_.clear();
}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(Eigen::Vector3d const& vertex, Eigen::Vector3d const& vertex_normal,uint id) : vertex_(vertex), vertex_normal_(vertex_normal),id_(id)
{
  neighbors_.clear();
}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(double const vertex[3],double const vertex_normal[3], IndexList const& neighbors,uint id) : vertex_(Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex))), vertex_normal_(Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex_normal))),
                                                                                                 neighbors_(neighbors),id_(id) {}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(Eigen::Vector3d const& vertex,Eigen::Vector3d const& vertex_normal, IndexList const& neighbors,uint id) :  vertex_(vertex), vertex_normal_(vertex_normal), neighbors_(neighbors),id_(id) {}
//--------------------------------------------------------------------
ContactPoint::ContactPoint(ContactPoint const& src) : vertex_(src.vertex_), vertex_normal_(src.vertex_normal_),neighbors_(src.neighbors_), id_(src.id_) {}
//--------------------------------------------------------------------
ContactPoint& ContactPoint::operator=(ContactPoint const& src)		  
{
  if (this !=&src)
    {
      vertex_=src.vertex_;
      vertex_normal_=src.vertex_normal_;
      neighbors_=src.neighbors_;
      id_=src.id_;
    }

  return *this;
}
//--------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, ContactPoint const& cp)
{
  stream <<'\n'<<"CONTACT POINT: "<<'\n'
	 <<"Id: "<<cp.id_<<'\n'
         <<"Vertex: ";

  for(uint i=0; i < 3; i++)
    stream<<cp.vertex_(i)<<" ";

  stream<<'\n'<<"Vertex normal: ";

  for(uint i=0; i < 3; i++)
    stream<<cp.vertex_normal_(i)<<" ";

  stream<<'\n'<<"Neighbor ids: ";

  for(ConstIndexListIterator it = cp.neighbors_.begin(); it != cp.neighbors_.end();it++)
    stream<< *it<<" ";

  stream<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------
ContactPoint::~ContactPoint() {}
//--------------------------------------------------------------------
Eigen::Vector3d* ContactPoint::getVertex(){return &vertex_;}
//--------------------------------------------------------------------
Eigen::Vector3d const* ContactPoint::getVertex()const{return &vertex_;}
//--------------------------------------------------------------------
Eigen::Vector3d* ContactPoint::getVertexNormal(){return &vertex_normal_;}
//--------------------------------------------------------------------
Eigen::Vector3d const* ContactPoint::getVertexNormal()const{return &vertex_normal_;}
//--------------------------------------------------------------------
void ContactPoint::setVertex(Eigen::Vector3d const& vertex){vertex_=vertex;}
//--------------------------------------------------------------------
void ContactPoint::setVertex(double const vertex[3]){vertex_=Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex));}
//--------------------------------------------------------------------
void ContactPoint::setVertexNormal(Eigen::Vector3d const& vertex_normal){vertex_normal_=vertex_normal;}
//--------------------------------------------------------------------
void ContactPoint::setVertexNormal(double const vertex_normal[3]){vertex_normal_=Eigen::Map<Eigen::Vector3d >(const_cast<double*>(vertex_normal));}
//--------------------------------------------------------------------
void ContactPoint::addNeighbor(uint neighbor){neighbors_.push_back(neighbor);}
//--------------------------------------------------------------------
ConstIndexListIterator ContactPoint::getNeighborItBegin()const{return neighbors_.begin();}
//--------------------------------------------------------------------
ConstIndexListIterator ContactPoint::getNeighborItEnd()const{return neighbors_.end();}
//--------------------------------------------------------------------
IndexList* ContactPoint::getNeighbors(){return &neighbors_;}
//--------------------------------------------------------------------
IndexList const* ContactPoint::getNeighbors()const{return &neighbors_;}
//--------------------------------------------------------------------
void ContactPoint::filterDuplicateNeighbors()
{
  neighbors_.sort();
  neighbors_.unique();
}
//--------------------------------------------------------------------
uint ContactPoint::getId()const{return id_;}
//--------------------------------------------------------------------
void ContactPoint::setId(uint id){id_=id;}
//--------------------------------------------------------------------
uint ContactPoint::getNumNeighbors()const{return neighbors_.size();}
//---------------------------------------------------------------------------------
//---------------------------------------------------------------------------------
}//namespace ICR
