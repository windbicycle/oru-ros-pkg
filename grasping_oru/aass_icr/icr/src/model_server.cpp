#include "../include/model_server.h"
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <urdf_interface/link.h>
#include <urdf/model.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <pcl/ros/conversions.h>
#include <Eigen/Core>
#include <algorithm>
#include <icr_msgs/SetObject.h>

namespace ICR
{
  ModelServer::ModelServer() : nh_private_("~"),pose_brc_(new PoseBroadcaster),obj_(new icr_msgs::Object),scale_(1.0),obj_loaded_(false)
{
  std::string searched_param;
  std::string gazebo_prefix;

  nh_private_.searchParam("pose_source", searched_param);
  nh_private_.getParam(searched_param, pose_source_);

  nh_private_.searchParam("model_directory", searched_param);
  nh_private_.getParam(searched_param, model_dir_);

  nh_private_.searchParam("gazebo_prefix", searched_param);
  nh_private_.getParam(searched_param, gazebo_prefix);

  nh_private_.searchParam("object_scale", searched_param);
  nh_private_.getParam(searched_param, scale_);

  //read the client topics to be called from the parameter server
  XmlRpc::XmlRpcValue obj_client_topics;
  if (nh_private_.searchParam("obj_client_topics", searched_param))
    {
      nh_.getParam(searched_param,obj_client_topics);
      ROS_ASSERT(obj_client_topics.getType() == XmlRpc::XmlRpcValue::TypeArray); 
    
       for (int32_t i = 0; i < obj_client_topics.size(); ++i) 
       	{    
       	  ROS_ASSERT(obj_client_topics[i].getType() == XmlRpc::XmlRpcValue::TypeString);
          
	  //create a new client for each connected server
	  boost::shared_ptr<ros::ServiceClient> obj_client(new ros::ServiceClient( nh_.serviceClient<icr_msgs::SetObject>((std::string)obj_client_topics[i])));
	  set_obj_clts_.push_back(obj_client);
         }
    }

  gazebo_spawn_clt_ = nh_.serviceClient<gazebo_msgs::SpawnModel>(gazebo_prefix + "/spawn_urdf_model");
  gazebo_delete_clt_ = nh_.serviceClient<gazebo_msgs::DeleteModel>(gazebo_prefix + "/delete_model");
  gazebo_get_wp_clt_ = nh_.serviceClient<gazebo_msgs::GetWorldProperties>(gazebo_prefix + "/get_world_properties");
  gazebo_pause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/pause_physics");
  gazebo_unpause_clt_ = nh_.serviceClient<std_srvs::Empty>(gazebo_prefix + "/unpause_physics");

  //If Gazebo is the intended pose source, wait for Gazebo services to be available 
  if(!strcmp(pose_source_.c_str(),"gazebo"))
    {
      gazebo_delete_clt_.waitForExistence(); 
      gazebo_get_wp_clt_.waitForExistence();  
      gazebo_pause_clt_.waitForExistence();  
      gazebo_unpause_clt_.waitForExistence(); 
      gazebo_spawn_clt_.waitForExistence();
    }
  //Wait for the connected set_object services to be available
  for(unsigned int i=0;i<set_obj_clts_.size();i++)
    set_obj_clts_[i]->waitForExistence();
  
 load_obj_srv_ = nh_.advertiseService("load_object",&ModelServer::loadObject,this);
}
  //--------------------------------------------------------------------------------
  bool ModelServer::loadObject(icr_msgs::LoadObject::Request  &req, icr_msgs::LoadObject::Response &res)
  {
    res.success=false;

    lock_.lock();
    obj_loaded_=false;

    //delete the previous model in Gazebo if Gazebo is the intended pose source
    if(!strcmp(pose_source_.c_str(),"gazebo"))  
      if(!gazeboDeleteModel(obj_name_))
	{
	  lock_.unlock();
	  return res.success;
	}

    //if a model is currently spawned in gazebo whichs name conforms to the file name in the
    //request, also delete it (this might be the case if there already was a model spawned in Gazebo
    //prior to the start of the model_server)
    if(!strcmp(pose_source_.c_str(),"gazebo"))  
      if(!gazeboDeleteModel(req.file))
	{
	  lock_.unlock();
	  return res.success;
	}

    //load the urdf model, extract obj_name_ and obj_frame_id_ from it and push it on the parameter server
    std::string serialized_model;
    if(!loadURDF(model_dir_+"/urdf/"+req.file+".urdf",serialized_model))
      {
	lock_.unlock();
	return res.success;
      }

    //Spawn the new model in Gazebo if Gazebo is the intended pose source
    if(!strcmp(pose_source_.c_str(),"gazebo"))  
      if(!gazeboSpawnModel(serialized_model,req.initial_pose))
	{
	  lock_.unlock();
	  return res.success;
	}

    //Load the according Wavefront .obj file
    pcl::PointCloud<pcl::PointNormal> obj_cloud;
    std::vector<std::vector<unsigned int> >  neighbors;
    if(!loadWavefrontObj(model_dir_+"/obj/"+req.file+".obj",obj_cloud,neighbors))
      {
	lock_.unlock();
	return res.success;
      }

    //Set the object in the Broadcaster
    pose_brc_->setObject(obj_cloud,obj_name_);

    //if pose should be taken from uc3m_objtracker then call it once here 
    if(pose_brc_->isPoseSourceUC3M() == true) 
      {
	if (!pose_brc_->getPoseUC3MObjtrack()) {
	  ROS_WARN("uc3m_objtracker failed.");
	}
      }

    //Update the Object message
    obj_->name=obj_name_;
    pcl::toROSMsg(obj_cloud,obj_->points);
    icr_msgs::Neighbors ngb;

    obj_->neighbors.clear();
    for(unsigned int i=0;i<neighbors.size();i++)  
      {    
	ngb.indices.resize(neighbors[i].size());
	std::copy(neighbors[i].begin(),neighbors[i].end(),ngb.indices.begin());     
	obj_->neighbors.push_back(ngb);
      }

    obj_loaded_=true;
    res.success=true;

    //Send the object to the connected servers
    icr_msgs::SetObject set_obj;
    set_obj.request.object=(*obj_);
    for(unsigned int i=0; i<set_obj_clts_.size();i++)
      if(!set_obj_clts_[i]->call(set_obj))
	{
	  ROS_ERROR("Could not send object to server number %d",i+1);
	  res.success=false;
        }
 
    lock_.unlock();

    
    return res.success;
  }
//--------------------------------------------------------------------------
void ModelServer::spin()
{
  //Broadcast the object's pose to tf
  if(obj_loaded_)
    pose_brc_->broadcastPose();

  ros::spinOnce();
}
//--------------------------------------------------------------------------
 bool ModelServer::loadURDF(std::string const & path,std::string & serialized_model)
  {

    serialized_model.clear();
  std::string line;
  std::ifstream file(path.c_str());

  if(!file.is_open()) 
    {
      ROS_ERROR("Could not open file %s",path.c_str());
      return false;
    }
  while(!file.eof()) // Parse the contents of the given urdf in a string
    {
      std::getline(file,line);
      serialized_model+=line;
    }
  file.close();


  //Parse the urdf in order to get the robot name and geometry
  urdf::Model urdf_model;
  if(!urdf_model.initString(serialized_model))
    {  
      ROS_ERROR("Could not parse urdf model %s",path.c_str());
      return false;
    }

  obj_name_=urdf_model.getName();
  obj_frame_id_="/"+urdf_model.getRoot()->name;//set the frame id to the base link name

  //Push the loaded file on the parameter server 
  nh_.setParam("icr_object",serialized_model);

  return true;
  }
//--------------------------------------------------------------------------
bool ModelServer::gazeboSpawnModel(std::string const & serialized_model,geometry_msgs::Pose const & initial_pose)
  {

  gazebo_msgs::SpawnModel spawn_model;
  std_srvs::Empty empty;
 
  spawn_model.request.model_name=obj_name_;
  spawn_model.request.model_xml=serialized_model;
  spawn_model.request.initial_pose=initial_pose;
  spawn_model.request.reference_frame="world"; //spawn the object in Gazebo's world frame

  gazebo_pause_clt_.call(empty);
  if (gazebo_spawn_clt_.call(spawn_model))
    ROS_INFO("Successfully spawned model %s in Gazebo",obj_name_.c_str());
  else
    {
      ROS_ERROR("Failed to spawn model %s in Gazebo",obj_name_.c_str());
      return false;
    }

  gazebo_unpause_clt_.call(empty);

  return true;
}
//--------------------------------------------------------------------------
  bool ModelServer::gazeboDeleteModel(std::string const & name)
  {

    gazebo_msgs::GetWorldProperties get_wp;
    gazebo_msgs::DeleteModel delete_model;
    std_srvs::Empty empty;

    if(!gazebo_get_wp_clt_.call(get_wp))
      {
	ROS_ERROR("Could not get world properties from Gazebo");
	return false;
      }

    delete_model.request.model_name=name;

    //see if a model with the obj_name_ is currently spawned, if yes delete it
    for(unsigned int i=0; i < get_wp.response.model_names.size();i++ )
      {
        if(!strcmp(get_wp.response.model_names[i].c_str(),name.c_str()))  
	  {
	    gazebo_pause_clt_.call(empty);
	    if(!gazebo_delete_clt_.call(delete_model))
	      {
		ROS_WARN("Could not delete model %s",name.c_str());
	      }
	    gazebo_unpause_clt_.call(empty);
            ROS_INFO("Deleted model %s in Gazebo",name.c_str());
	    break;
	  }
      }

    return true;
  }
//--------------------------------------------------------------------------
  bool ModelServer::loadWavefrontObj(std::string const & path, pcl::PointCloud<pcl::PointNormal> & cloud, std::vector<std::vector<unsigned int> > & neighbors )
{
  ObjectLoader obj_loader; 

  obj_loader.loadObject(path.c_str() ,obj_name_);

  if(!((obj_loader.objectLoaded()) && (obj_loader.getObject()->getNumCp() > 0)))
    {
      ROS_INFO("Invalid Wavefront obj file: %s",path.c_str());
      return false;
    }
  obj_loader.getObject()->scaleObject(scale_);
  double xmax=0;
  pcl::PointNormal p;
  neighbors.resize(obj_loader.getObject()->getNumCp());
  for(unsigned int i=0; i<obj_loader.getObject()->getNumCp();i++)
    {
      Eigen::Vector3d const * v=obj_loader.getObject()->getContactPoint(i)->getVertex();
      Eigen::Vector3d const * vn=obj_loader.getObject()->getContactPoint(i)->getVertexNormal();
      neighbors[i].resize(obj_loader.getObject()->getContactPoint(i)->getNeighbors()->size());
      std::copy(obj_loader.getObject()->getContactPoint(i)->getNeighbors()->begin(),obj_loader.getObject()->getContactPoint(i)->getNeighbors()->end(),neighbors[i].begin()); 
      p.x=v->x(); p.y=v->y(); p.z=v->z(); 
      p.normal[0]=vn->x();  p.normal[1]=vn->y();  p.normal[2]=vn->z(); 
      p.curvature=0; //not estimated
      cloud.push_back(p);
      if(p.x > xmax)
        xmax=p.x;


    }
  cloud.header.frame_id=obj_frame_id_;
  cloud.header.stamp=ros::Time::now();
  cloud.width = cloud.size();
  cloud.height = 1;
  cloud.is_dense=true;
  std::cout<<"XMAX: "<<xmax<<std::endl;
  return true;
}
//--------------------------------------------------------------------------
}//end namespace
