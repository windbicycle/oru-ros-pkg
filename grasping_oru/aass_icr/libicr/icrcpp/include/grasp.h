#ifndef grasp_h___
#define grasp_h___

#include "target_object.h" 
#include "utilities.h" 
#include "contact_model.h" 
#include "finger.h" 
#include "wrench_space.h"
#include <iostream>

namespace ICR
{
//------------------------------------------------------------------
//------------------------------------------------------------------
/*! 
 *  \brief The central class of the icrcpp library; Holds a list of ICR::Finger and a
 *  ICR::DiscreteWrenchSpace describing the Grasp Wrench Space. 
 *
 */
class Grasp
{
 private:

  std::vector<Finger*> fingers_;
  bool initialized_;
  TargetObjectPtr obj_;  
  DiscreteWrenchSpace* gws_;
  uint num_fingers_;
  uint num_grasp_wrenches_;

  OWSPtr computeOWS(Finger const* new_finger);
  PatchListPtr computePatches(Finger* new_finger);
  void addFinger(FingerParameters const& param, uint centerpoint_id);
  void computeGWS();
  void clear();

 public:

  Grasp();
  Grasp(Grasp const& src);
  Grasp& operator=(Grasp const& src);
  friend std::ostream& operator<<(std::ostream& stream,Grasp const& grasp);
  ~Grasp();
/*! 
 *   Cretes a list of Finger-pointers. A Finger holds a pointer to an Object Wrench Space and a
 * pointer to a list of patches. OWS and patches are precomputed when the Finger's parent grasp is
 * initialized. A separate OWS is computed for each finger with a different contact model, a
 * separate list of patches is computed for each finger with a different inclusion rule. Fingers
 * with the same contact model/inclusion rule hold pointers to the same OWS/patch list. The
 * inclusion rule determines which vertices of the target object's mesh are eligible for inclusion
 * in a patch centered around a given center-point. In case of the single-point contact model, each
 * patch only contains its respective center-point. Also the Grasp Wrench Space is computed
 * utilizing Qhull.
 */
  void init(FParamList const& f_param_list,const TargetObjectPtr obj,VectorXui const& centerpoint_ids);  
  void setCenterPointId(uint finger_id,uint centerpoint_id);
  void setCenterPointIds(VectorXui const& centerpoint_ids);
  bool isInitialized()const;
  Finger const* getFinger(uint id) const;
  uint getNumFingers()const;
  const TargetObjectPtr getParentObj()const;
  DiscreteWrenchSpace const* getGWS()const;
};
//------------------------------------------------------------------
//------------------------------------------------------------------
}//namespace ICR
#endif
