#include "../include/limit_surface.h"
#include "../include/debug.h"
#include <iostream>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
LimitSurface::LimitSurface() : force_magnitude_(0), disc_(0), mu_0_(0) , mu_T_(0) {initializeSelectionMatrix();}
//--------------------------------------------------------------------
LimitSurface::LimitSurface(double force_magnitude) : force_magnitude_(force_magnitude), disc_(0), 
                                                    mu_0_(0) , mu_T_(0), contact_type_(Frictionless)
{
  assert(force_magnitude >0);
  initializeSelectionMatrix();

  addFrictionlessWrench();
}
//--------------------------------------------------------------------
LimitSurface::LimitSurface(double force_magnitude, int disc,double mu_0) : force_magnitude_(force_magnitude), disc_(disc), 
									  mu_0_(mu_0), mu_T_(0), contact_type_(Frictional)
{
  assert(force_magnitude >0);
  assert(mu_0_ > 0); 
  assert(disc_ > 0);

  initializeSelectionMatrix();

  addFrictionlessWrench();
  addHardFingerWrenches();
}
//--------------------------------------------------------------------
LimitSurface::LimitSurface(double force_magnitude, int disc, double mu_0,double mu_T) : force_magnitude_(force_magnitude), disc_(disc), 
                                                                                         mu_0_(mu_0), mu_T_(mu_T), contact_type_(Soft_Finger)
{
  assert(mu_0_ > 0); 
  assert(mu_T_ > 0);
  assert(disc_ > 0);
  assert(force_magnitude > 0);

  initializeSelectionMatrix();

  addFrictionlessWrench();
  addHardFingerWrenches();
  addSoftFingerWrenches();
}
//--------------------------------------------------------------------
LimitSurface::LimitSurface(LimitSurface const& src) : force_magnitude_(src.force_magnitude_), disc_(src.disc_), 
                                                      mu_0_(src.mu_0_) , mu_T_(src.mu_T_), 
                                                      local_cone_(src.local_cone_), contact_type_(src.contact_type_), selection_matrix_(src.selection_matrix_) {}
//--------------------------------------------------------------------
LimitSurface& LimitSurface::operator=(LimitSurface const& src)
{
  if (this !=&src)
    {
      force_magnitude_=src.force_magnitude_;
      mu_0_=src.mu_0_;
      mu_T_=src.mu_T_;
      disc_=src.disc_;
      local_cone_=src.local_cone_;
      contact_type_=src.contact_type_;
      selection_matrix_=src.selection_matrix_;
    }
  return *this;
}
//--------------------------------------------------------------------
bool LimitSurface::operator==(LimitSurface const& other)const
{
  if ((force_magnitude_==other.force_magnitude_) & (disc_==other.disc_) & (mu_0_==other.mu_0_)
      & (mu_T_==other.mu_T_) & (contact_type_==other.contact_type_) & (local_cone_.getNumPrimitiveWrenches()==other.local_cone_.getNumPrimitiveWrenches()))
    {
      if(*(local_cone_.getWrenches())==*(other.local_cone_.getWrenches()))
	return true;
      else
	return false;
    }
    else
      return false;
}
//--------------------------------------------------------------------
LimitSurface::~LimitSurface(){}
//--------------------------------------------------------------------
void LimitSurface::initializeSelectionMatrix()
{
  //Build the selection matrix for the wrenches according to "Murray, Li & Sastry - A Mathematical Introduction
  //to Robotic Manipulation; pp. 219"
  selection_matrix_.setZero();
  selection_matrix_.topLeftCorner(3,3) =Eigen::Matrix<double,3,3>::Identity(3,3);
  selection_matrix_(5,3)=1;
}
//--------------------------------------------------------------------
void LimitSurface::addFrictionlessWrench()
{
  Eigen::Vector4d gen_forces;
  gen_forces<< 0, 0,force_magnitude_, 0;
  local_cone_.addWrenches(selection_matrix_*gen_forces);
}
//--------------------------------------------------------------------
void LimitSurface::addHardFingerWrenches()
{
  Eigen::Matrix4Xd gen_forces=Eigen::Matrix4Xd::Zero(4,disc_);
 
  Eigen::RowVectorXd lin=Eigen::RowVectorXd::LinSpaced(Eigen::Sequential,disc_+1,-PI,PI);
  lin.conservativeResize(disc_);
  double alpha=atan(mu_0_);
  double c_radius =sin(alpha)*force_magnitude_;
  
  gen_forces.row(0)=lin.array().cos()*c_radius;
  gen_forces.row(1)=lin.array().sin()*c_radius;
  gen_forces.row(2).fill(force_magnitude_*cos(alpha));
  local_cone_.addWrenches(selection_matrix_*gen_forces);
}
//--------------------------------------------------------------------
void LimitSurface::addSoftFingerWrenches()
{
  Eigen::Matrix<double,4,2> gen_forces=Eigen::Matrix<double,4,2>::Zero(4,2);
  
  gen_forces(2,0)=force_magnitude_;
  gen_forces(2,1)=force_magnitude_;
  gen_forces(3,0)=mu_T_*force_magnitude_;
  gen_forces(3,1)=-mu_T_*force_magnitude_;

  local_cone_.addWrenches(selection_matrix_*gen_forces);
}
//--------------------------------------------------------------------
int LimitSurface::getNumPrimitiveWrenches() const {return local_cone_.getNumPrimitiveWrenches();}
//--------------------------------------------------------------------
double LimitSurface::getMu0() const {return mu_0_;}
//--------------------------------------------------------------------
double LimitSurface::getMuT() const {return mu_T_;}
//--------------------------------------------------------------------
int LimitSurface::getDisc() const {return disc_;}
//--------------------------------------------------------------------
WrenchCone const* LimitSurface::getLocalWrenchCone() const {return &local_cone_;}
//--------------------------------------------------------------------
ContactType LimitSurface::getContactType() const {return contact_type_;}
//--------------------------------------------------------------------
double LimitSurface::getForceMagnitude() const {return force_magnitude_;}
//--------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, LimitSurface const& lim_surf)
{
  std::string contact_type;

    if (lim_surf.contact_type_==Frictionless) contact_type="Frictionless";
  else if (lim_surf.contact_type_==Frictional)   contact_type="Frictional";
  else if (lim_surf.contact_type_==Soft_Finger)  contact_type="Soft Finger";
  else if (lim_surf.contact_type_==Undefined_CT)  contact_type="Undefined";
  else contact_type="Warning in LimitSurface: Invalid contact type!";

  stream <<'\n'<<"LIMIT SURFACE: "<<'\n'
         <<"Force magnitude: " << lim_surf.force_magnitude_ <<'\n'
         <<"Discretization: "<<lim_surf.disc_<<'\n'
         <<"Mu_0: "<<lim_surf.mu_0_<<'\n'
         <<"Mu_T: "<<lim_surf.mu_T_<<'\n'
         <<"Contact type: "<<contact_type<<'\n'<<'\n';
   
  return stream;
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR

