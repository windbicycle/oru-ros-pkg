#include "../include/ows.h"
#include "../include/debug.h"
#include "../include/config.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <Eigen/Geometry>

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
OWS::OWS() : initialized_(false), div_by_lambda_(false), lambda_(0),num_wc_(0){wrench_cones_.clear();} 
//--------------------------------------------------------------------
OWS::OWS(OWS const& src) :  wrench_cones_(src.wrench_cones_), parent_obj_name_(src.parent_obj_name_), initialized_(src.initialized_), 
			    div_by_lambda_(src.div_by_lambda_), lim_surf_(src.lim_surf_), lambda_(src.lambda_),num_wc_(src.num_wc_) {}
//--------------------------------------------------------------------
OWS& OWS::operator=(OWS const& src)
{
  if (this !=&src)
    {
      wrench_cones_=src.wrench_cones_;
      parent_obj_name_=src.parent_obj_name_;
      initialized_=src.initialized_;
      div_by_lambda_=src.div_by_lambda_;
      lim_surf_=src.lim_surf_;
      lambda_=src.lambda_;
      num_wc_=src.num_wc_;
    }
  return *this;
}
//--------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream, OWS const& ows)
{
  stream <<'\n'<<"OWS: "<<'\n'
         <<"Parent object name: "<<ows.parent_obj_name_<<'\n'
         <<"Is initialized: "<<ows.initialized_<<'\n'
         <<"Number of wrench cones: "<<ows.num_wc_<<'\n'
         <<"Lambda: "<<ows.lambda_<<'\n'
         <<"Wrenches divided by lambda: "<<ows.div_by_lambda_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------
OWS::~OWS()
{
  for(uint i=0; i < num_wc_; i++)
    delete wrench_cones_[i];
}
//--------------------------------------------------------------------
void OWS::updateLambda(Eigen::Vector3d const* const cp_vtx)
{  
  double norm=(*cp_vtx).norm();
  lambda_ = (norm > lambda_) ? norm : lambda_;
}
//--------------------------------------------------------------------
void OWS::addWrenchCone(uint id,Eigen::Vector3d const* const cp_vtx, Eigen::Vector3d const* const cp_vtx_normal)
{

  //Build the transformation matrix for the wrenches according to "Murray, Li & Sastry - A Mathematical Introduction
  //to Robotic Manipulation; pp. 218"
  Eigen::Vector3d z_ax(0,0,1);
  Eigen::Vector3d axis=(z_ax.cross(*cp_vtx_normal));
  axis.normalize();
  double angle=acos(z_ax.dot(*cp_vtx_normal));
  assert(angle==angle); //assure that the angle is not NaN
  Eigen::Matrix3d Rot_ci=Eigen::Matrix3d::Zero();

  if (fabs(angle-PI) < EPSILON_WRENCH_CONE_ROTATION) //rotation angle is pi    
    {Rot_ci(0,0)=1;  Rot_ci(1,1)=-1; Rot_ci(2,2)=-1;}
  else if (fabs(angle) < EPSILON_WRENCH_CONE_ROTATION) // rotation angle is 0
    Rot_ci.setIdentity();
  else
    Rot_ci=Eigen::AngleAxisd(angle,axis);

  Eigen::Matrix<double,6,6> trans_mat=Eigen::Matrix<double,6,6>::Zero();
  trans_mat.bottomLeftCorner<3,3>()=skewSymmetricMatrix(*cp_vtx)*Rot_ci;
  trans_mat.topLeftCorner<3,3>()=Rot_ci;
  trans_mat.bottomRightCorner<3,3>()=Rot_ci;
 
  //Multiply the local wrench cone with the transformation matrix and create a new Wrench cone associated
  //with the current contact point
  wrench_cones_.push_back(new WrenchCone(id,trans_mat*(*lim_surf_.getLocalWrenchCone()->getWrenches())));
}
//--------------------------------------------------------------------
void OWS::init(TargetObject const& obj, LimitSurface const& lim_surf)
{
  //would need to clean up the wrench_cones_ before recomputing the OWS
  assert(!initialized_);

  num_wc_=obj.getNumCp();
  wrench_cones_.reserve(num_wc_);
  parent_obj_name_=obj.getName();
  lim_surf_=lim_surf;
    
  for(uint id=0; id < num_wc_;id++)
    {
      addWrenchCone(id,obj.getContactPoint(id)->getVertex(),obj.getContactPoint(id)->getVertexNormal());
      updateLambda(obj.getContactPoint(id)->getVertex());
    } 

  initialized_=true;

#ifdef DIVIDE_OWS_BY_LAMBDA
    this->divideByLambda();
#endif

#ifdef DEBUG_OWS // File for writing the wrenches
  remove("../debug/wrenches.txt");
  FILE* wn=fopen ("../debug/wrenches.txt","a");
  if(!wn)
    {
      std::cout<<"Error in OWS: Couldn't write to file. Exiting..."<<std::endl;
      exit(1);
    }

  Eigen::Matrix<double,6,Eigen::Dynamic> pw;
  for(uint id=0; id < num_wc_;id++)
    {
      pw=*wrench_cones_[id]->getWrenches();
 
      for (int i=0; i<pw.cols();i++)
        fprintf(wn, "% f % f % f % f % f % f \n", pw.col(i)(0),pw.col(i)(1) ,pw.col(i)(2),pw.col(i)(3),pw.col(i)(4) ,pw.col(i)(5));    
    }

fclose (wn);
#endif
}
//--------------------------------------------------------------------
uint OWS::getNumWrenchCones()const{return num_wc_;}
//--------------------------------------------------------------------
WrenchCone const* OWS::getWrenchCone(uint id)const{return wrench_cones_.at(id);}
//--------------------------------------------------------------------
LimitSurface const* OWS::getLimitSurface()const{return &lim_surf_;}
//--------------------------------------------------------------------
void OWS::scale(double scale)
{
  for(uint id=0; id < num_wc_; id++)
    wrench_cones_[id]->scaleWrenches(scale);

}
void OWS::divideByLambda()
{
  assert(initialized_);

    if(!div_by_lambda_)
      for(uint id=0; id < num_wc_; id++)
	wrench_cones_[id]->scaleWrenchTorques(1/lambda_);

  div_by_lambda_=true;
}
//--------------------------------------------------------------------
std::string OWS::getParentObjectName()const{return parent_obj_name_;}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
