#include "../include/phalange.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>

namespace ICR
{

Phalange::Phalange(tf::Transform const & L_T_Cref,std::string const & phl_name,std::string const & phl_frame_id, std::string const & sensor_topic) : nh_private_("~"),L_T_Cref_(L_T_Cref),C_T_O_(new icr_msgs::StampedContactPose),
																		      phl_name_(phl_name),phl_frame_id_(phl_frame_id),obj_frame_id_("/fixed")
{
  std::string searched_param;
  nh_private_.searchParam("tf_prefix",searched_param);
  nh_private_.getParam(searched_param, tf_prefix_);

  L_T_Cref_.setRotation(L_T_Cref_.getRotation().normalize());//ensure normalized orientation

  contacts_subs_=nh_.subscribe<icr_msgs::ContactState>(sensor_topic, 1, &Phalange::listenContacts, this);
  ct_pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>(phl_name_ + "/contact_pose",1);
  set_pose_srv_ = nh_.advertiseService(phl_name_ + "/set_ref_contact_pose",&Phalange::setPose,this);

  //Initialize the filter
  tf_filter_ = new tf::MessageFilter<icr_msgs::StampedContactPose>(tf_list_, obj_frame_id_, 10);
  tf_filter_->registerCallback(boost::bind(&Phalange::transformContactPose, this, _1) );
}
//---------------------------------------------------------------------
Phalange::~Phalange(){delete tf_filter_;}
//---------------------------------------------------------------------
void Phalange::transformContactPose(const boost::shared_ptr<const icr_msgs::StampedContactPose>& C_T_L)
{
  geometry_msgs::PoseStamped P_in, P_out;
  P_in.header=C_T_L->header;
  P_in.pose=C_T_L->contact_pose.pose;

  //Compute the current contact pose w.r.t. the object frame 
  tf_list_.transformPose(obj_frame_id_,P_in,P_out); 

  lock_.lock();
  C_T_O_->contact_pose.phalange=phl_name_;
  C_T_O_->header.stamp=C_T_L->header.stamp;
  C_T_O_->header.frame_id=obj_frame_id_;
  C_T_O_->contact_pose.pose=P_out.pose;
  C_T_O_->contact_pose.touching=C_T_L->contact_pose.touching;
  lock_.unlock();
}
//---------------------------------------------------------------------
void Phalange::listenContacts(const icr_msgs::ContactState::ConstPtr& cts_st)
{
  icr_msgs::StampedContactPose::Ptr C_T_L(new icr_msgs::StampedContactPose);
  C_T_L->header.stamp=cts_st->header.stamp;
  C_T_L->header.frame_id=tf_prefix_ + phl_frame_id_;

  lock_.lock();   

  if(!strcmp(cts_st->info.c_str(),"touching"))
    {
        C_T_L->contact_pose.pose.position.x=cts_st->contact_position.x;
        C_T_L->contact_pose.pose.position.y=cts_st->contact_position.y;
        C_T_L->contact_pose.pose.position.z=cts_st->contact_position.z;

        //Contact orientation is generated via projecting the pose of the link frame on the nullspace of
        //the normal specified in the message
        tf::quaternionTFToMsg(projectPose(tf::Vector3(cts_st->contact_normal.x,cts_st->contact_normal.y,cts_st->contact_normal.z)),C_T_L->contact_pose.pose.orientation);

        C_T_L->contact_pose.touching=true;  //Phalange geometry and target object geometry are in contact
        lock_.unlock();

        tf_filter_->add(C_T_L);//Push the current pose on the filter queue

	//Publish the contact pose 
	geometry_msgs::PoseStamped P;
        P.header=C_T_L->header;
	P.pose=C_T_L->contact_pose.pose;
        ct_pose_pub_.publish(P);

        return;
    }

  lock_.unlock();

  //Use the reference contact pose if no contact is made
  tf::pointTFToMsg(L_T_Cref_.getOrigin(),C_T_L->contact_pose.pose.position);
  tf::quaternionTFToMsg(L_T_Cref_.getRotation(),C_T_L->contact_pose.pose.orientation);

  C_T_L->contact_pose.touching=false;
  tf_filter_->add(C_T_L);

  //Publish the contact pose 
  geometry_msgs::PoseStamped P;
  P.header=C_T_L->header;
  P.pose=C_T_L->contact_pose.pose;
  ct_pose_pub_.publish(P);
}
//---------------------------------------------------------------------
void Phalange::setTargetObjFrameId(std::string const & obj_frame_id)
{

  lock_.lock();
  obj_frame_id_=obj_frame_id;
  tf_filter_->setTargetFrame(obj_frame_id_);//Set the filter's target frame to the current target object
  lock_.unlock();
}
//---------------------------------------------------------------------
tf::Quaternion Phalange::projectPose(tf::Vector3 const & z)
{
  tf::Quaternion ori;
  Eigen::Vector3d z_Cref;  

  //The transposed rotation matrix containing the basis of the new contact frame expressed in the
  //reference contact frame. It is transposed since btMatrix3x3 only allows row-wise access
  btMatrix3x3 Cref_R_C;

  //Express z (given in the link frame) in the contact reference frame
  tf::VectorTFToEigen(L_T_Cref_.inverse()*z,z_Cref); 
  z_Cref.normalize();//Assert unit normal

  //Generate the matrix I-n*n^T for projecting on the nullspace of the new z-axis. The nullspace
  //is perpendicular to z.
  Eigen::Matrix3d P = (Eigen::Matrix3d()).setIdentity()-z_Cref*z_Cref.transpose();
  
  //Project the x & y axis of the contact reference frame - this is unstable if one of these axis is
  //(nearly) orthogonal to the nullspace - there should be some check on the norm of the projections ...
  tf::VectorEigenToTF((P*Eigen::Vector3d(1,0,0)).normalized(),Cref_R_C[0]);
  tf::VectorEigenToTF((P*Eigen::Vector3d(0,1,0)).normalized(),Cref_R_C[1]);
  tf::VectorEigenToTF(z_Cref,Cref_R_C[2]);
    
  //Transformation from the new contact frame to the reference frame
  tf::Transform Cref_T_C(Cref_R_C.transpose(),tf::Vector3(0,0,0));

  //Return the orientation (the position is computed in the listenContacts callback by simply
  //averaging the positions contained in the gazebo_msgs/ContactsState msg)
  return (L_T_Cref_*Cref_T_C.getRotation()).normalize();
}
//---------------------------------------------------------------------
bool Phalange::setPose(icr_msgs::SetPose::Request  &req, icr_msgs::SetPose::Response &res)
{
  //Sets the pose of the contact reference w.r.t the phalange link frame
  res.success=false;
  lock_.lock();

  L_T_Cref_.setOrigin(tf::Vector3(req.origin.x,req.origin.y,req.origin.z));
  L_T_Cref_.setRotation(tf::createQuaternionFromRPY(req.rpy.x,req.rpy.y,req.rpy.z));

  lock_.unlock();
  res.success=true;
  return res.success;
}
//---------------------------------------------------------------------
icr_msgs::StampedContactPose::ConstPtr Phalange::getStampedContactPose()
{
  //return a copy for thread safety
  lock_.lock();
  icr_msgs::StampedContactPose::ConstPtr C_T_O(new icr_msgs::StampedContactPose(*C_T_O_.get()));
  lock_.unlock();

  return C_T_O;
}
//---------------------------------------------------------------------
std::string Phalange::getPhalangeName()
{
  std::string phl_name;

  lock_.lock();
  phl_name=phl_name_;
  lock_.unlock();

  return phl_name;
}
//---------------------------------------------------------------------
std::string Phalange::getPhalangeFrameId()
{
  std::string phl_frame_id;

  lock_.lock();
  phl_frame_id=phl_frame_id_;
  lock_.unlock();

  return phl_frame_id;
}
//---------------------------------------------------------------------
}//end namespace
