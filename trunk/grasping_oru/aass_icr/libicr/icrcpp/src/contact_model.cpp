#include "../include/contact_model.h"
#include "../include/debug.h"
#include "../include/target_object.h"
#include <iostream>
#include <cstdlib>
#include <string>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
PointContactModel::PointContactModel() : id_(0),model_type_(Single_Point) {}
//--------------------------------------------------------------------
PointContactModel::PointContactModel(FingerParameters const& param) : id_(0),model_type_(Single_Point)
{
      if (param.getContactType()==Frictionless)
	lim_surf_=LimitSurface(param.getForceMagnitude());
      else if (param.getContactType()==Frictional)
  	lim_surf_=LimitSurface(param.getForceMagnitude(),param.getDisc(),param.getMu0());
      else if (param.getContactType()==Soft_Finger)
        lim_surf_=LimitSurface(param.getForceMagnitude(),param.getDisc(),param.getMu0(),param.getMuT());
      else
  	{
  	  std::cout<<"Error in ContactModel: Invalid contact type. exiting..."<<std::endl;
  	  exit(1);
  	}
}
//--------------------------------------------------------------------
PointContactModel::PointContactModel(FingerParameters param,uint id) : id_(id),model_type_(Single_Point)
{
      if (param.getContactType()==Frictionless)
	lim_surf_=LimitSurface(param.getForceMagnitude());
      else if (param.getContactType()==Frictional)
  	lim_surf_=LimitSurface(param.getForceMagnitude(),param.getDisc(),param.getMu0());
      else if (param.getContactType()==Soft_Finger)
        lim_surf_=LimitSurface(param.getForceMagnitude(),param.getDisc(),param.getMu0(),param.getMuT());
      else
  	{
  	  std::cout<<"Error in ContactModel: Invalid contact type. exiting..."<<std::endl;
  	  exit(1);
  	}
}
//--------------------------------------------------------------------
PointContactModel::PointContactModel(PointContactModel const& src) : id_(src.id_), lim_surf_(src.lim_surf_), model_type_(src.model_type_) {}
//--------------------------------------------------------------------
PointContactModel& PointContactModel::operator=(PointContactModel const& src)
{
  if (this !=&src)
    {
      id_=src.id_;
      lim_surf_=src.lim_surf_;
      model_type_=src.model_type_;
    }
  return *this;
}
//--------------------------------------------------------------------
bool PointContactModel::operator==(PointContactModel const& other)const
{
  if((lim_surf_==other.lim_surf_) & (model_type_==other.model_type_))
    return true;
  else
    return false;
}
//--------------------------------------------------------------------
PointContactModel::~PointContactModel() {}
//--------------------------------------------------------------------
LimitSurface const* PointContactModel::getLimitSurface()const{return &lim_surf_;}
//--------------------------------------------------------------------
ModelType PointContactModel::getModelType()const {return model_type_;}
//--------------------------------------------------------------------
uint PointContactModel::getId()const{return id_;}
//--------------------------------------------------------------------
void PointContactModel::setId(uint const id){id_=id;}
//--------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, PointContactModel const& pc_model)
{
  std::string model_type;

  if (pc_model.model_type_==Single_Point) model_type="Single Point";
  else if (pc_model.model_type_==Multi_Point)  model_type="Multi Point";
  else if (pc_model.model_type_==Undefined_MT)  model_type="Undefined";
  else model_type="Warning in ContactModel: Invalid model type!";

  stream <<'\n'<<"CONTACT MODEL: "<<'\n'
         <<"Id: " << pc_model.id_ <<'\n'
         <<"Model type: "<<model_type<<'\n'<<'\n';

  return stream;
}
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
MultiPointContactModel::MultiPointContactModel(){model_type_=Multi_Point;}
//--------------------------------------------------------------------
MultiPointContactModel::MultiPointContactModel(FingerParameters const& param) : PointContactModel(param), inclusion_rule_(*param.getInclusionRule())
{
  assert(param.getModelType()==Multi_Point);
  model_type_=Multi_Point;
}
//--------------------------------------------------------------------
MultiPointContactModel::MultiPointContactModel(FingerParameters const& param,uint id) : PointContactModel(param,id), inclusion_rule_(*param.getInclusionRule())
{
  assert(param.getModelType()==Multi_Point);
  model_type_=Multi_Point;
} 
//--------------------------------------------------------------------
bool MultiPointContactModel::operator==(MultiPointContactModel const& other)const
{
  if((inclusion_rule_.rule_parameter_==other.inclusion_rule_.rule_parameter_) & 
     (PointContactModel::operator==(other)) & (inclusion_rule_.rule_type_==other.inclusion_rule_.rule_type_ ))
    return true;
  else
    return false;
}
//--------------------------------------------------------------------
MultiPointContactModel::MultiPointContactModel(MultiPointContactModel const& src) : PointContactModel(src), inclusion_rule_(src.inclusion_rule_) {}
//--------------------------------------------------------------------
MultiPointContactModel& MultiPointContactModel::operator=(MultiPointContactModel const& src)
{
  if (this !=&src)
    {
       PointContactModel::operator=(src);
       inclusion_rule_=src.inclusion_rule_;
    }
  return *this;
}
//--------------------------------------------------------------------
MultiPointContactModel::~MultiPointContactModel() {} 
//--------------------------------------------------------------------
InclusionRule const* MultiPointContactModel::getInclusionRule()const{return &inclusion_rule_;}
//--------------------------------------------------------------------
void MultiPointContactModel::inclusionRuleSphere(uint center_pt_id,TargetObject const& obj,IndexList& included_points)const
{
  //Fill the list included_points with according point indices from obj considering the patch center
  std::cout<<"Inclusion rule not implemented yet"<<std::endl;
  //Below are dummies, just do avoid unused variable compiling errors
  std::cout<<"Center point id: "<<center_pt_id<<std::endl;
  TargetObject obj2=obj;
  IndexList list2=included_points;
}
void MultiPointContactModel::computeInclusion(uint center_pt_id,TargetObject const& obj,IndexList& included_points) const
{

  if (inclusion_rule_.rule_type_==Sphere) 
    inclusionRuleSphere(center_pt_id,obj,included_points);
  else 
    {
      std::cout<<"Invalid inclusion rule: exiting"<<std::endl; 
      exit(1);
    }
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
