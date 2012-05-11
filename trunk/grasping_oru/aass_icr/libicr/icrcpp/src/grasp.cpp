#include "../include/grasp.h"
#include "../include/ows.h"
#include "../include/debug.h"
#include <iostream>
#include "assert.h"

namespace ICR
{
//------------------------------------------------------------------
//------------------------------------------------------------------
Grasp::Grasp() : initialized_(false) , gws_(NULL),num_fingers_(0),num_grasp_wrenches_(0){}
//------------------------------------------------------------------
Grasp::Grasp(Grasp const& src) :  fingers_(src.fingers_), initialized_(src.initialized_),
                                  obj_(src.obj_),gws_(src.gws_),num_fingers_(src.num_fingers_),num_grasp_wrenches_(src.num_grasp_wrenches_) {}
//------------------------------------------------------------------
Grasp& Grasp::operator=(Grasp const& src)
{
  if (this !=&src)
    {
      fingers_=src.fingers_;
      initialized_=src.initialized_;
      obj_=src.obj_;
      gws_=src.gws_;
      num_fingers_=src.num_fingers_;
      num_grasp_wrenches_=src.num_grasp_wrenches_;
    }
  return *this;
}
//------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, Grasp const& grasp)
{
  stream <<'\n'<<"GRASP: "<<'\n'
         <<"Number of fingers: " << grasp.num_fingers_ <<'\n'
         <<"Number of grasp wrenches: " << grasp.num_grasp_wrenches_ <<'\n'
         <<"Is initialized: "<<std::boolalpha<<grasp.initialized_<<'\n'<<'\n';

  return stream;
}
//------------------------------------------------------------------
Grasp::~Grasp(){clear();}
//------------------------------------------------------------------
void Grasp::clear()
{
 delete gws_;
  for(uint i=0; i < num_fingers_; i++)
    delete fingers_[i];

  fingers_.clear();
}
//------------------------------------------------------------------
void Grasp::computeGWS()
{
  double* wrenches=new double[num_grasp_wrenches_*gws_->getDimension()];
  double* wrench_it=wrenches;
  uint wrench_cone_cardinality=0;
  //ConstIndexListIterator curr_pt;

  for(uint i=0;i<num_fingers_;i++)
    {
      wrench_cone_cardinality=fingers_[i]->getOWS()->getLimitSurface()->getNumPrimitiveWrenches();
      // curr_pt=fingers_[i]->getCenterPointPatch()->patch_ids_.begin();
	// for(uint j=0; j < fingers_[i]->getCenterPointPatch()->patch_ids_.size(); j++)
      for(ConstIndexListIterator curr_pt = fingers_[i]->getCenterPointPatch()->patch_ids_.begin(); curr_pt != fingers_[i]->getCenterPointPatch()->patch_ids_.end();curr_pt++)
	{
	  Eigen::Map<Matrix6Xd >(wrench_it,6,wrench_cone_cardinality)=*(fingers_[i]->getOWS()->getWrenchCone(*curr_pt)->getWrenches());
          wrench_it+=gws_->getDimension()*wrench_cone_cardinality;
 	}
    }  
  
  gws_->computeConvexHull(wrenches,num_grasp_wrenches_);
  delete[] wrenches;
}
//------------------------------------------------------------------
PatchListPtr Grasp::computePatches(Finger* new_finger)
{
  PatchListPtr patches;
  if(new_finger->getContactModel()->getModelType()==Single_Point)
    { 
      if( fingers_.size() > 0)
	{
	      patches=fingers_[0]->getPatches();
	      return patches;
        }

      patches=PatchListPtr(new std::vector<Patch* >);
      patches->reserve(obj_->getNumCp());
      for(uint centerpoint_id=0; centerpoint_id < obj_->getNumCp();centerpoint_id++)
	patches->push_back(new Patch(centerpoint_id)); 
    }
  else if(new_finger->getContactModel()->getModelType()==Multi_Point)
    {
      for(uint id=0; id < fingers_.size(); id++)
	{
	  if(dynamic_cast<MultiPointContactModel*>(fingers_[id]->getContactModel())->getInclusionRule() == dynamic_cast<MultiPointContactModel*>(new_finger->getContactModel())->getInclusionRule())
	    {
	      patches=fingers_[id]->getPatches();
	      return patches;
	    }
	}
      patches=PatchListPtr(new std::vector<Patch* >);
      patches->reserve(obj_->getNumCp());
      for(uint centerpoint_id=0; centerpoint_id < obj_->getNumCp();centerpoint_id++)
	(*patches.get())[centerpoint_id]= new Patch(centerpoint_id,*obj_,*dynamic_cast<MultiPointContactModel*>(new_finger->getContactModel())->getInclusionRule());
    }
  else
    {
      std::cout<<"Error in Grasp::computePatches - Invalid contact model type. Exiting..."<<std::endl;
      exit(1);
    }
  return patches;
}
//------------------------------------------------------------------
OWSPtr Grasp::computeOWS(Finger const* new_finger)
{
  OWSPtr ows;
  for(uint id=0; id < fingers_.size(); id++)
    {
      if( *fingers_[id]->getContactModel()->getLimitSurface() == *new_finger->getContactModel()->getLimitSurface())
      	{
          ows=fingers_[id]->getOWS();
	  return ows;
        }
    }

  ows=OWSPtr(new OWS());
  ows->init(*obj_,*new_finger->getContactModel()->getLimitSurface());
  return ows;
}
//------------------------------------------------------------------
void Grasp::addFinger(FingerParameters const& param, uint centerpoint_id)
{
  Finger* new_finger=new Finger(param); 


  new_finger->setName(param.getName());
  new_finger->init(centerpoint_id,computePatches(new_finger),computeOWS(new_finger));
  num_grasp_wrenches_+=(new_finger->getCenterPointPatch()->patch_ids_.size())*(new_finger->getContactModel()->getLimitSurface()->getNumPrimitiveWrenches());
  fingers_.push_back(new_finger); 
}
//------------------------------------------------------------------
Finger const* Grasp::getFinger(uint id) const {return fingers_.at(id);}
//------------------------------------------------------------------
uint Grasp::getNumFingers()const{return num_fingers_;}
//------------------------------------------------------------------
void Grasp::init(FParamList const& f_param_list,const TargetObjectPtr obj,VectorXui const& centerpoint_ids)
{
  assert(f_param_list.size()>0);
  assert(f_param_list.size()==(uint)centerpoint_ids.size());
  assert((bool)obj);

  if(initialized_)//Clean up possible previous grasp
    clear();

  gws_=new DiscreteWrenchSpace();

  num_fingers_=f_param_list.size();
  for (uint i=0;i < num_fingers_;i++)
    assert(centerpoint_ids(i) < obj->getNumCp()); //make sure all centerpoint ids are valid

  obj_=obj;//const_cast<TargetObject*>(obj);
  for (uint i=0; i< num_fingers_;i++)
      addFinger(f_param_list[i],(uint)centerpoint_ids(i));

  computeGWS();
  initialized_=true;
}
//------------------------------------------------------------------
void Grasp::setCenterPointId(uint finger_id,uint centerpoint_id)
{
  //Not tested yet!! Changing the center contact point requires updating the number of grasp wrenches (the number of patch points might differ between old and new center point) and recomputing the GWS
 assert(initialized_);
 assert(centerpoint_id <= obj_->getNumCp()-1);
 num_grasp_wrenches_-=(fingers_.at(finger_id)->getCenterPointPatch()->patch_ids_.size())*(fingers_.at(finger_id)->getContactModel()->getLimitSurface()->getNumPrimitiveWrenches());
 fingers_.at(finger_id)->setCenterPointId(centerpoint_id);
 num_grasp_wrenches_+=(fingers_.at(finger_id)->getCenterPointPatch()->patch_ids_.size())*(fingers_.at(finger_id)->getContactModel()->getLimitSurface()->getNumPrimitiveWrenches());
 delete gws_; //old gws_ needs to be deleted, since DiscreteWrenchSpace::conv_hull_.runQhull() can only be executed once in the lifetime of conv_hull_
 gws_ = new DiscreteWrenchSpace();
 computeGWS();
}
//------------------------------------------------------------------
void Grasp::setCenterPointIds(VectorXui const& centerpoint_ids)
{
  //Not tested yet!!
  assert(initialized_);
  assert(centerpoint_ids.size()==(int)num_fingers_);

  for(uint finger_id=0; finger_id < num_fingers_; finger_id++)
    {
      assert(centerpoint_ids(finger_id) <= obj_->getNumCp()-1);
      num_grasp_wrenches_-=(fingers_.at(finger_id)->getCenterPointPatch()->patch_ids_.size())*(fingers_.at(finger_id)->getContactModel()->getLimitSurface()->getNumPrimitiveWrenches());
      fingers_.at(finger_id)->setCenterPointId(centerpoint_ids(finger_id));
      num_grasp_wrenches_+=(fingers_.at(finger_id)->getCenterPointPatch()->patch_ids_.size())*(fingers_.at(finger_id)->getContactModel()->getLimitSurface()->getNumPrimitiveWrenches());
    }
 delete gws_; //old gws_ needs to be deleted, since DiscreteWrenchSpace::conv_hull_.runQhull() can only be executed once
 gws_ = new DiscreteWrenchSpace();
  computeGWS();
}
//------------------------------------------------------------------
bool Grasp::isInitialized()const{return initialized_;}
//------------------------------------------------------------------
const TargetObjectPtr Grasp::getParentObj()const{return obj_;}
//------------------------------------------------------------------
DiscreteWrenchSpace const* Grasp::getGWS()const{return gws_;}
//------------------------------------------------------------------
//------------------------------------------------------------------
}//namespace ICR
