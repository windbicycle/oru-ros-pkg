#ifndef contact_point_h___
#define contact_point_h___

#include "utilities.h"
#include <iostream>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Contains the coordinates of a ICR::TargetObject's vertex and its corresponding vertex normal, as well as a
 *  unique id and a list of id's of neighboring points  
 *
 *  \details The id in ICR::ContactPoint#id_ corresponds to the index of the contact point in
 *  ICR::TargetObject#contact_points_. Neighboring points are defined as the ones connected to the
 *  considered point by an edge of the mesh of the target object.
 */
class ContactPoint
{
 private:

  Eigen::Vector3d vertex_;
  Eigen::Vector3d vertex_normal_;
  IndexList neighbors_;
  uint id_;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactPoint();
  ContactPoint(double const vertex[3]);
  ContactPoint(Eigen::Vector3d const& vertex);
  ContactPoint(double const vertex[3],uint id);
  ContactPoint(Eigen::Vector3d const& vertex,uint id);
  ContactPoint(double const vertex[3],double const vertex_normal[3],uint id);
  ContactPoint(Eigen::Vector3d const& vertex, Eigen::Vector3d const& vertex_normal,uint id);
  ContactPoint(double const vertex[3],double const vertex_normal[3], IndexList const& neighbors,uint id);
  ContactPoint(Eigen::Vector3d const& vertex,Eigen::Vector3d const& vertex_normal, IndexList const& neighbors,uint id);
  ContactPoint(ContactPoint const& src);
  ContactPoint& operator=(ContactPoint const& src);
  friend std::ostream& operator<<(std::ostream& stream, ContactPoint const& cp);
  ~ContactPoint();

  Eigen::Vector3d* getVertex();
  Eigen::Vector3d const* getVertex() const;
  Eigen::Vector3d* getVertexNormal();
  Eigen::Vector3d const* getVertexNormal() const;
  void setVertex(Eigen::Vector3d const& vertex);
  void setVertex(double const vertex[3]);
  void setVertexNormal(Eigen::Vector3d const& vertex_normal);
  void setVertexNormal(double const vertex_normal[3]);
  void addNeighbor(uint neighbor);
/*! 
 *  Removes duplicated id's from ICR::ContactPoint#neighbors_; Has no effect if there aren't any duplicates;
 */
  void filterDuplicateNeighbors();
  ConstIndexListIterator getNeighborItBegin()const;
  ConstIndexListIterator getNeighborItEnd()const;
  IndexList* getNeighbors();
  IndexList const* getNeighbors()const;
  uint getNumNeighbors()const;
  uint getId()const;
  void setId(uint id);
};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
#endif


