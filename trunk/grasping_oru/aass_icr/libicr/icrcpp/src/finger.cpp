#include "../include/finger.h"
#include "../include/contact_model.h"
#include "../include/ows.h"
#include "../include/debug.h"
#include "../include/config.h"
#include "assert.h"

namespace ICR
{
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
InclusionRule::InclusionRule() : 
  rule_parameter_(DEFAULT_INCLUSION_RULE_PARAMETER),
  rule_type_(DEFAULT_INCLUSION_RULE_TYPE),
  filter_inside_points_(DEFAULT_INCLUSION_RULE_FILTER_INSIDE_POINTS) {}
//------------------------------------------------------
  InclusionRule::InclusionRule(double rp_in, RuleType rt_in, bool filter_in) : 
    filter_inside_points_(filter_in) 
  {
    if (rp_in < 0) {
      rule_parameter_ = DEFAULT_INCLUSION_RULE_PARAMETER;
    } else {
      rule_parameter_ = rp_in;
    }

    if (rt_in != Undefined_RT || rt_in != Sphere) {
      rule_type_ = DEFAULT_INCLUSION_RULE_TYPE;
    } else {
      rule_type_ = rt_in;
    }
  }
//------------------------------------------------------
bool InclusionRule::inclusionTest(ContactPoint const* center_point, ContactPoint const* test_point)const
{
  if (rule_type_==Sphere)
    {
     if( ((*center_point->getVertex())-(*test_point->getVertex())).norm() <= rule_parameter_ )
	return true;
    }
  else
    {
      std::cout<<"Error in InclusionRule::inclusionTest: Invalid rule type. Exiting..."<<std::endl;
      exit(1);
    }

  return false;
}
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream &stream,InclusionRule const& inclusion_rule)
{
  std::string rule_type;

 if (inclusion_rule.rule_type_==Sphere) rule_type="Sphere";
  else if (inclusion_rule.rule_type_==Undefined_RT)  rule_type="Undefined";
  else rule_type="Warning in InclusionRule: Invalid rule type!";

  stream <<'\n'<<"INCLUSION RULE: "<<'\n'
         <<"Inclusion rule type: "<<rule_type<<'\n'
         <<"Inclusion rule parameter: "<<inclusion_rule.rule_parameter_<<'\n'
	 <<"Points inside a patch are filtered out: "<<std::boolalpha<<inclusion_rule.filter_inside_points_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
Node::Node() : contact_point_(NULL), inside_patch_(true){}
//--------------------------------------------------------------------------
Node::Node(ContactPoint* contact_point) : contact_point_(contact_point), inside_patch_(true){}
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, Node const& node)
{
  stream <<'\n'<<"NODE: "<<'\n';

  if(node.contact_point_ != NULL)
    {
      stream <<"Associated contact point id: "<<node.contact_point_->getId()<<'\n'
	     <<"Is inside patch: "<<std::boolalpha<<node.inside_patch_<<'\n';
    }
  else
    stream <<"Contact point pointer not set"<<'\n';

  stream<<'\n';   

  return stream;
}
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
Patch::Patch(){}
//--------------------------------------------------------------------------
Patch::Patch(uint centerpoint_id){patch_ids_.push_back(centerpoint_id);}
//--------------------------------------------------------------------------
Patch::Patch(uint centerpoint_id,TargetObject const& obj, InclusionRule const& rule)
{
  //Breadth first exploration of the mesh with the centerpoint of the patch as root node. The criterion to qualify a
  //point for inclusion in the patch is that the InclusionRule::inclusionTest is satisfied. The vector 'nodes' acts as a FIFO
  //list for the search; explored_cp(i) states if contact point i was already explored during the search and whether it qualified 
  //for inclusion in the patch or not. If the filter_inside_points_ flag of the LimitSuface is set, the patch is filtered so that it
  //contains only centerpoint and border points, not the points inside the patch. The criterion for a point to be on the border of the 
  //patch is, that not all of its neighbors qualified for patch inclusion.

  std::list<Node> nodes;
  VectorXui explored_cp=VectorXui::Zero(obj.getNumCp(),1); //set all entries to NOT_EXPLORED

  nodes.push_back(Node(const_cast<ContactPoint*>(obj.getContactPoint(centerpoint_id)))); //push the node corresponding to the centerpoint of the patch
  explored_cp(centerpoint_id)=EXPLORED_QUALIFIED;
 
  while(nodes.size() > 0)
    {   
      for(ConstIndexListIterator neighbor=nodes.front().contact_point_->getNeighborItBegin(); neighbor != nodes.front().contact_point_->getNeighborItEnd(); neighbor++)      
	{
	  if(explored_cp(*neighbor)==NOT_EXPLORED)
	    {	      
	      if(rule.inclusionTest(obj.getContactPoint(centerpoint_id) , obj.getContactPoint(*neighbor)))
		{
		  nodes.push_back(Node(const_cast<ContactPoint*>(obj.getContactPoint(*neighbor))));
		  explored_cp(*neighbor)=EXPLORED_QUALIFIED;
		}
	      else
		{
                 //If not all neighbors of the investigated node qualify for patch inclusion, the contact point associated with the node
                 //is a border point of the patch.
		  nodes.front().inside_patch_=false; 
		  explored_cp(*neighbor)=EXPLORED_UNQUALIFIED;
		}
	    }
          //The same as above, not all neighbors of the current node qualify for inclusion, thus the contact point is on the border
          else if(explored_cp(*neighbor)==EXPLORED_UNQUALIFIED)
	    nodes.front().inside_patch_=false;
	}

      if(rule.filter_inside_points_)//Filtering of points inside the patch, if the according flag is set
	{
	  if((!nodes.front().inside_patch_) | (nodes.front().contact_point_->getId()==centerpoint_id))//The centerpoint always qualifies for patch inclusion
	    patch_ids_.push_back(nodes.front().contact_point_->getId());
	}
      else
	  patch_ids_.push_back(nodes.front().contact_point_->getId());

      nodes.pop_front();
    }
}
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream,Patch const& patch)
{
  stream <<'\n'<<"PATCH: "<<'\n'
	 <<"Number of contact points in the patch: "<< patch.patch_ids_.size() <<'\n'
	 <<"Patch id's: ";

  for(ConstIndexListIterator it=patch.patch_ids_.begin(); it != patch.patch_ids_.end(); it++)
    stream<< *it <<" ";

  stream<<'\n'<<'\n';

  return stream;
}
//---------------------------------------------------------------
//---------------------------------------------------------------
FingerParameters::FingerParameters() :
  name_("Default finger"),
  force_magnitude_(DEFAULT_FORCE_MAGNITUDE), 
  disc_(DEFAULT_DISC), 
  mu_0_(DEFAULT_MU_0),  
  mu_T_(DEFAULT_MU_T),
  contact_type_(DEFAULT_CONTACT_TYPE),
  model_type_(DEFAULT_CONTACT_MODEL_TYPE),
  inclusion_rule_(InclusionRule(DEFAULT_INCLUSION_RULE_PARAMETER)) {}
//--------------------------------------------------------------------------
FingerParameters::FingerParameters(FingerParameters const& src) : 
  name_(src.name_),
  force_magnitude_(src.force_magnitude_), 
  disc_(src.disc_),
  mu_0_(src.mu_0_), 
  mu_T_(src.mu_T_), 
  contact_type_(src.contact_type_),
  model_type_(src.model_type_), 
  inclusion_rule_(src.inclusion_rule_) {}
//--------------------------------------------------------------------------
FingerParameters::FingerParameters(string name, 
				   double force_magnitude_in,
				   uint disc_in,
				   double mu_0_in, 
				   double mu_T_in, 
				   ContactType contact_type_in, 
				   ModelType model_type_in, 
				   double radius_in) {
  name_ = name;
  if (force_magnitude_in <= 0) {
    force_magnitude_ = DEFAULT_FORCE_MAGNITUDE;
  } else {
      force_magnitude_ = force_magnitude_in;
  }
  
  if (disc_in < 4) {
    disc_ = DEFAULT_DISC;
  } else {
    disc_ = disc_in;
  }
  if(mu_0_in <= 0) {
    mu_0_ = DEFAULT_MU_0;
  } else {
    mu_0_ = mu_0_in;
  }
  if(mu_T_in <= 0) {
    mu_T_ = DEFAULT_MU_T;
  } else {
    mu_T_ = mu_T_in;
  }
  if(contact_type_in != Undefined_CT || contact_type_in != Frictional ||
     contact_type_in != Frictional || contact_type_in != Soft_Finger) {
    contact_type_ = DEFAULT_CONTACT_TYPE;
  } else {
    contact_type_ = contact_type_in;
  }
  if(model_type_in != Undefined_MT || model_type_in != Single_Point || model_type_in != Multi_Point) {
    model_type_ = DEFAULT_CONTACT_MODEL_TYPE;
  } else {
    model_type_ = model_type_in;
  }
  if (radius_in < 0) {
    inclusion_rule_ = InclusionRule(DEFAULT_INCLUSION_RULE_PARAMETER);
  } else {
    inclusion_rule_ = InclusionRule(radius_in);
  }
}
//--------------------------------------------------------------------------
FingerParameters& FingerParameters::operator=(FingerParameters const& src)
{
  if (this !=&src)
    {
      name_ = src.name_;
      force_magnitude_=src.force_magnitude_;
      disc_=src.disc_;
      mu_0_=src.mu_0_;
      mu_T_=src.mu_T_;
      contact_type_=src.contact_type_;
      model_type_=src.model_type_;
      inclusion_rule_=src.inclusion_rule_;
    }
  return *this;
}
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream &stream,FingerParameters const& param)
{
  std::string contact_type;
  std::string model_type;
  std::string rule_type;

    if (param.contact_type_==Frictionless) contact_type="Frictionless";
  else if (param.contact_type_==Frictional)   contact_type="Frictional";
  else if (param.contact_type_==Soft_Finger)  contact_type="Soft Finger";
  else if (param.contact_type_==Undefined_CT)  contact_type="Undefined";
  else contact_type="Warning in FingerParameters: Invalid contact type!";

     if (param.model_type_==Single_Point) model_type="Single Point";
  else if (param.model_type_==Multi_Point)   model_type="Multi Point";
  else if (param.model_type_==Undefined_MT)  model_type="Undefined";
  else model_type="Warning in FingerParameters: Invalid model type!";

 if (param.inclusion_rule_.rule_type_==Sphere) rule_type="Sphere";
  else if (param.inclusion_rule_.rule_type_==Undefined_RT)  rule_type="Undefined";
  else rule_type="Warning in FingerParameters: Invalid rule type!";

  stream <<'\n'<<"FINGER PARAMETERS: "<<'\n'
	 <<"Name: " << param.name_ << '\n'
         <<"Force magnitude: " << param.force_magnitude_ <<'\n'
         <<"Discretization: "<<param.disc_<<'\n'
         <<"Mu_0: "<<param.mu_0_<<'\n'
         <<"Mu_T: "<<param.mu_T_<<'\n'
         <<"Contact type: "<<contact_type<<'\n'
         <<"Model type: "<<model_type<<'\n'
         <<"Inclusion rule type: "<<rule_type<<'\n'
         <<"Inclusion rule parameter: "<<param.inclusion_rule_.rule_parameter_<<'\n'<< std::endl;

  return stream;
}        
//--------------------------------------------------------------------------                   
FingerParameters::~FingerParameters(){}
//--------------------------------------------------------------------------
void FingerParameters::setFrictionlessContact(double force_magnitude)
{
  assert(force_magnitude_ > 0);
  force_magnitude_=force_magnitude;
  contact_type_=Frictionless;
}
//--------------------------------------------------------------------------
void FingerParameters::setFrictionalContact(double force_magnitude,uint disc,double mu_0)
{
  assert(force_magnitude_ > 0);
  assert(disc > 0);
  assert(mu_0 > 0);
  force_magnitude_=force_magnitude;
  disc_=disc;
  mu_0_=mu_0;
  contact_type_=Frictional;
}
//--------------------------------------------------------------------------
void FingerParameters::setSoftFingerContact(double force_magnitude,uint disc,double mu_0,double mu_T)
{
  assert(force_magnitude_ > 0);
  assert(disc > 0);
  assert(mu_0 > 0);
  assert(mu_T > 0);
  force_magnitude_=force_magnitude;
  disc_=disc;
  mu_0_=mu_0;
  mu_T_=mu_T;
  contact_type_=Soft_Finger;
}

void FingerParameters::setContactType(ContactType contact_type_in) {
  contact_type_ = contact_type_in;
}

void FingerParameters::setContactType(string &contact_type_in) {
  if (contact_type_in.compare("Frictionless") == 0) {
    contact_type_ = Frictionless;
  } else if (contact_type_in.compare("Frictional") == 0) {
    contact_type_ = Frictional;
  } else if (contact_type_in.compare("Soft_Finger") == 0) {
    contact_type_ = Soft_Finger;
  } else {
    contact_type_ = Undefined_CT;
  } 
}

//--------------------------------------------------------------------------
void FingerParameters::setContactModelType(ModelType model_type){model_type_=model_type;}

void FingerParameters::setContactModelType(std::string &model_type_in){
  if (model_type_in.compare("Single_Point") == 0) {
    model_type_=Single_Point;
  } else if (model_type_in.compare("Multi_Point") == 0) {
    model_type_=Single_Point;
  } else {
    model_type_=Undefined_MT;
  }
}
//--------------------------------------------------------------------------
void FingerParameters::setInclusionRule(InclusionRule const& inclusion_rule){inclusion_rule_=inclusion_rule;}
//--------------------------------------------------------------------------
void FingerParameters::setInclusionRuleType(RuleType rule_type){inclusion_rule_.rule_type_=rule_type;}

void FingerParameters::setInclusionRuleType(std::string &rule_type_in) {
  if (rule_type_in.compare("Sphere") == 0) {
    inclusion_rule_.rule_type_=Sphere;
  } else {
    inclusion_rule_.rule_type_=Undefined_RT;
  }
}

//--------------------------------------------------------------------------
void FingerParameters::setInclusionRuleParameter(double rule_parameter){inclusion_rule_.rule_parameter_=rule_parameter;}
//--------------------------------------------------------------------------
void FingerParameters::setInclusionRuleFilterPatch(bool filter_inside_points){inclusion_rule_.filter_inside_points_=filter_inside_points;}
//--------------------------------------------------------------------------
double FingerParameters::getForceMagnitude()const{return force_magnitude_;}
//--------------------------------------------------------------------------
uint FingerParameters::getDisc()const{return disc_;}
//--------------------------------------------------------------------------
std::string FingerParameters::getName()const{return name_;}
//--------------------------------------------------------------------------
double FingerParameters::getMu0()const{return mu_0_;}
//--------------------------------------------------------------------------
double FingerParameters::getMuT()const{return mu_T_;}
//-------------------------------------------------------------------
ContactType FingerParameters::getContactType()const{return contact_type_;}
//-------------------------------------------------------------------
ModelType FingerParameters::getModelType()const{return model_type_;}
//-------------------------------------------------------------------
InclusionRule const* FingerParameters::getInclusionRule()const{return &inclusion_rule_;}
//-------------------------------------------------------------------
RuleType FingerParameters::getInclusionRuleType()const{return inclusion_rule_.rule_type_;}
//-------------------------------------------------------------------
uint FingerParameters::getInclusionRuleParameter()const{return inclusion_rule_.rule_parameter_;}
//-------------------------------------------------------------------
bool FingerParameters:: getInclusionRuleFilterPatch()const{return inclusion_rule_.filter_inside_points_;}
//-------------------------------------------------------------------
//-------------------------------------------------------------------
Finger::Finger() : c_model_(NULL), centerpoint_id_(0), initialized_(false), name_("default")
{
  FingerParameters default_param;

  if (default_param.getModelType()==Single_Point)
    c_model_=new PointContactModel(default_param); 
  else if (default_param.getModelType()==Multi_Point)
    c_model_=new MultiPointContactModel(default_param); 
  else
    {
      std::cout<<"Error in Finger: Invalid contact model type. exiting..."<<std::endl;
      exit(1);
    }
}
//--------------------------------------------------------------------------
Finger::Finger(FingerParameters const& param) : 
  c_model_(NULL), 
  centerpoint_id_(0), 
  initialized_(false)
{
  if (param.getModelType()==Single_Point)
    c_model_=new PointContactModel(param); 
  else if (param.getModelType()==Multi_Point)
    c_model_=new MultiPointContactModel(param); 
  else
    {
      std::cout<<"Error in Finger: Invalid contact model type. exiting..."<<std::endl;
      exit(1);
    }
}
//--------------------------------------------------------------------------
Finger::Finger(Finger const& src) : c_model_(src.c_model_), ows_(src.ows_), patches_(src.patches_),
				    centerpoint_id_(src.centerpoint_id_), initialized_(src.initialized_),name_("default"){}
//--------------------------------------------------------------------------
Finger& Finger::operator=(const Finger& src)		  
{
  if (this !=&src)
    {
      c_model_=src.c_model_;
      ows_=src.ows_;
      patches_=src.patches_;
      centerpoint_id_=src.centerpoint_id_;
      initialized_=src.initialized_;
      name_=src.name_;
    }
  return *this;
}
//--------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream,Finger const& finger)
{
  stream <<'\n'<<"FINGER: "<<'\n'
	 <<"Is initialized: "<<std::boolalpha<<finger.initialized_<<'\n'
         <<"Centerpoint id: "<<finger.centerpoint_id_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------------
Finger::~Finger()
{
  delete c_model_;
  if (patches_.use_count()==1) //The last shared pointer to a patch list has to clean up the allocated patches
     for(uint i=0; i < patches_->size();i++)
       delete (*patches_.get())[i];
}
//--------------------------------------------------------------------------
Patch const* Finger::getCenterPointPatch()const{return (*patches_.get())[centerpoint_id_];}
//--------------------------------------------------------------------------
Patch const* Finger::getPatch(uint id)const{return (*patches_.get())[id];}
//--------------------------------------------------------------------------
const OWSPtr Finger::getOWS()const{return ows_;}
//--------------------------------------------------------------------------
string Finger::getName()const{return name_;}
//--------------------------------------------------------------------------
void Finger::setName(std::string const & name){name_=name;}
//--------------------------------------------------------------------------
PointContactModel const* Finger::getContactModel()const{return c_model_;}
//--------------------------------------------------------------------------
PointContactModel* Finger::getContactModel(){return c_model_;}
//--------------------------------------------------------------------------
const PatchListPtr Finger::getPatches()const{return patches_;}
//--------------------------------------------------------------------------
uint Finger::getCenterPointId()const{return centerpoint_id_;}
//--------------------------------------------------------------------------
bool Finger::isInitialized()const{return initialized_;}
//--------------------------------------------------------------------------
void Finger::setCenterPointId(uint centerpoint_id)
{
  assert(initialized_);
  centerpoint_id_=centerpoint_id;
}
//--------------------------------------------------------------------------
void Finger::init(uint centerpoint_id, const PatchListPtr patches, const OWSPtr ows)
{
  assert((bool)patches);
  assert((bool)ows);
  centerpoint_id_=centerpoint_id;
  patches_=patches;
  ows_=ows;
  initialized_=true;
}
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
}//namespace ICR

