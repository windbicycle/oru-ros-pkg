#include "../include/wrench_cone.h"
#include "../include/debug.h"

namespace ICR
{
//--------------------------------------------------------------------
//--------------------------------------------------------------------
WrenchCone::WrenchCone() : num_primitive_wrenches_(0),id_(0) {}
//--------------------------------------------------------------------
WrenchCone::WrenchCone(uint id) :num_primitive_wrenches_(0), id_(id){}
//--------------------------------------------------------------------
WrenchCone::WrenchCone(uint id, Eigen::Matrix<double,6,Eigen::Dynamic> const& cone) : num_primitive_wrenches_(cone.cols()), cone_(cone),id_(id) {}
//--------------------------------------------------------------------
WrenchCone::WrenchCone(WrenchCone const& src) : num_primitive_wrenches_(src.num_primitive_wrenches_), cone_(src.cone_) ,id_(src.id_){}
//--------------------------------------------------------------------
WrenchCone& WrenchCone::operator=(WrenchCone const& src)
{
  if (this !=&src)
    {
      num_primitive_wrenches_=src.num_primitive_wrenches_;
      cone_=src.cone_;
      id_=src.id_;
    }
  return *this;
}
//--------------------------------------------------------------------
std::ostream& operator<<(std::ostream& stream,WrenchCone const&  wrench_cone)
{

  stream <<'\n'<<"WRENCH CONE: "<<'\n'
         <<"Id: "<<wrench_cone.id_<<'\n'
	 <<"Number of primitive wrenches: " << wrench_cone.num_primitive_wrenches_<<'\n'
         <<"Primitive wrenches: " <<'\n'<< wrench_cone.cone_<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------
WrenchCone::~WrenchCone() {}
//--------------------------------------------------------------------
Eigen::Matrix<double,6,Eigen::Dynamic> const* WrenchCone::getWrenches()const {return &cone_;}
//--------------------------------------------------------------------
Eigen::Matrix<double,6,Eigen::Dynamic>* WrenchCone::getWrenches(){return &cone_;}
//--------------------------------------------------------------------
uint WrenchCone::getNumPrimitiveWrenches() const {return num_primitive_wrenches_;}
//--------------------------------------------------------------------
void WrenchCone::scaleWrenches(double scale)
{
  cone_=cone_*scale;
}
//--------------------------------------------------------------------
void WrenchCone::scaleWrenchTorques(double scale)
{
  cone_.block(3, 0, 3, cone_.cols())=cone_.block(3, 0, 3, cone_.cols())*scale;
}
//--------------------------------------------------------------------
void WrenchCone::addWrenches(Eigen::Matrix<double,6,Eigen::Dynamic> const& wrenches)
{
  cone_.conservativeResize(6,cone_.cols()+wrenches.cols());
  cone_.block(0,cone_.cols()-wrenches.cols(), 6,wrenches.cols())=wrenches;
  num_primitive_wrenches_=cone_.cols();
}
//--------------------------------------------------------------------
void WrenchCone::setWrenches(Eigen::Matrix<double,6,Eigen::Dynamic> const& wrenches)
{
 cone_=wrenches;
 num_primitive_wrenches_=cone_.cols();
}
//--------------------------------------------------------------------
void WrenchCone::setId(uint id){id_=id;}
//--------------------------------------------------------------------
uint WrenchCone::getId()const{return id_;}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
