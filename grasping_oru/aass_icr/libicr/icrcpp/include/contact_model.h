#ifndef contact_model_h___
#define contact_model_h___

#include "utilities.h"
#include <iostream>
#include "limit_surface.h"
#include "finger.h"

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Base class for the ICR::MultiPointContactModel, holds a ICR::LimitSurface
 */
class PointContactModel
{
 private:
  
  uint id_;
  LimitSurface lim_surf_;  
  
 protected:

  ModelType model_type_;
  PointContactModel();

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointContactModel(FingerParameters const& param);  
  PointContactModel(FingerParameters param,uint id);  
  virtual bool operator==(PointContactModel const& other)const;
  PointContactModel(PointContactModel const& src);
  PointContactModel& operator=(PointContactModel const& src);
  friend std::ostream& operator<<(std::ostream& stream, PointContactModel const& pc_model);
  virtual ~PointContactModel();

  LimitSurface const* getLimitSurface() const;
  ModelType getModelType() const; 
  uint getId()const;
  void setId(uint const id);
};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Derived from ICR::PointContactModel, has an additional member ICR::InclusionRule
 */
class MultiPointContactModel : public PointContactModel
{
 private:

  InclusionRule inclusion_rule_;
  void inclusionRuleSphere(uint center_pt_id,TargetObject const& obj,IndexList& included_points) const;

 protected:

  MultiPointContactModel();
  
 public:

  MultiPointContactModel(FingerParameters const& param);  
  MultiPointContactModel(FingerParameters const& param,uint id);  

  virtual bool operator==(MultiPointContactModel const& other)const;
  MultiPointContactModel(MultiPointContactModel const& src);
  MultiPointContactModel& operator=(MultiPointContactModel const& src);
  virtual ~MultiPointContactModel();

  void computeInclusion(uint center_pt_id,TargetObject const& obj,IndexList& included_points) const;
  InclusionRule const* getInclusionRule()const;

};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
} //namespace ICR
#endif
