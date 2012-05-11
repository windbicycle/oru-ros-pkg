#ifndef target_object_h___
#define target_object_h___

#include "../include/contact_point.h"
#include "../include/utilities.h"
#include <string>
#include <iostream>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Created by ICR::ObjectLoader; Holds the name of the loaded object and a
 *  ICR::ContactPointList describing the object
 *  \details This class has ICR::ObjectLoader as a
 *  friend in order to allow the object loader to have direct access to ICR::TargetObject#contact_points_ when
 *  building the object. This saves some computation time opposed to using the public interface of
 *  the target object.
 */
class TargetObject
{

 private:

  std::string name_;
  uint num_cp_;
  ContactPointList contact_points_;
  
  //  uint findClosestIdxCore(double x, double y, double z);

 public:

  friend class ObjectLoader;  

  TargetObject();
  TargetObject(std::string const& name);
  TargetObject(TargetObject const& src);
  TargetObject& operator=(TargetObject const& src);
  friend std::ostream& operator<<(std::ostream& stream, TargetObject const& obj);
  ~TargetObject();
 
  std::string const getName() const;
  void setName(std::string const& name);
  uint getNumCp() const;
  void reserveCpList(uint num_cp);
  //  uint findClosestIdx(double x, double y, double z);
  //  uint findClosestIdx(Eigen::Vector3d& pt);
  void addContactPoint(ContactPoint const& point);
  /*!  \brief Multiplies each ICR::ContactPoint#vertex_ of the contact points contained in
 *  ICR::TargetObject#contact_points_ with scale
 */
  void scaleObject(double scale);
  ContactPoint const* getContactPoint(uint id) const;
};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
#endif
