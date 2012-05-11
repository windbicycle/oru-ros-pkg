#include "../include/target_object.h"
#include "../include/debug.h"
#include <assert.h>
#include <math.h>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
TargetObject::TargetObject() : num_cp_(0) {contact_points_.clear();}
//--------------------------------------------------------------------
TargetObject::TargetObject(std::string const& name) : name_(name), num_cp_(0) {contact_points_.clear();}
//--------------------------------------------------------------------
TargetObject::TargetObject(TargetObject const& src) : name_(src.name_), num_cp_(src.num_cp_),
                                                      contact_points_(src.contact_points_) {}
//--------------------------------------------------------------------
TargetObject& TargetObject::operator=(TargetObject const& src)		  
{
  if (this !=&src)
    {
      name_=src.name_;
      num_cp_=src.num_cp_;
      contact_points_=src.contact_points_;
    }
  return *this;
}
//--------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, TargetObject const& obj)
{
  stream <<'\n'<<"TARGET OBJECT: "<<'\n'
         <<"Name: "<<obj.name_<<'\n'
         <<"Number of contact points: "<<obj.num_cp_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------
TargetObject::~TargetObject()
{
  for (uint i=0;i<contact_points_.size();i++)
    delete contact_points_[i];
}
//--------------------------------------------------------------------
std::string const TargetObject::getName() const {return name_;}
//--------------------------------------------------------------------
void TargetObject::setName(std::string const& name) {name_=name;}
//--------------------------------------------------------------------
void TargetObject::reserveCpList(uint num_cp){contact_points_.reserve(num_cp);}
//--------------------------------------------------------------------
// void TargetObject::findClosestIdx(double x, double y, double z) {
//   for (uint i=0; i<num_cp_ ; i++) {
    
    
//   }
// }
// //--------------------------------------------------------------------
// void TargetObject::findClosestIdx(Eigen::Vector3d& pt) {

// }
//--------------------------------------------------------------------
uint TargetObject::getNumCp() const {return num_cp_;}
//--------------------------------------------------------------------
void TargetObject::addContactPoint(ContactPoint const& point)
{
  //Ensure unit normal
  assert(fabs( (*point.getVertexNormal()).norm()-1) <= EPSILON_UNIT_NORMAL);

  contact_points_.push_back(new ContactPoint(point));
  num_cp_++;
}
//--------------------------------------------------------------------
void TargetObject::scaleObject(double scale)
{
  for (uint i=0; i < contact_points_.size(); i++)
    *(contact_points_[i]->getVertex())=*(contact_points_[i]->getVertex())*scale;
}
//--------------------------------------------------------------------
ContactPoint const* TargetObject::getContactPoint(uint id) const {return contact_points_.at(id);}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR

