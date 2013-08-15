/**
* 2D NDT-MCL Node. 
* This application runs the ndt-mcl localization based on a map and laser scanner and odometry. 
* 
* The initialization is based now on ground truth pose information (should be replaced with manual input). 
* 
* The visualization depends on mrpt-gui 
* 
* More details about the algorithm:
* Jari Saarinen, Henrik Andreasson, Todor Stoyanov and Achim J. Lilienthal, Normal Distributions Transform Monte-Carlo Localization (NDT-MCL)
* IEEE/RSJ International Conference on Intelligent Robots and Systems November 3-8, 2013, Tokyo Big Sight, Japan
* 
* @author Jari Saarinen (jari.p.saarinen@gmail.com)
* 
*	@TODO Initialization from GUI
* @TODO Global initialization possibility 
* Known issues: in launch file (or hard coded parameters) you have to set the same resolution for NDT map as is saved -- otherwise it wont work
*/

#define USE_VISUALIZATION_DEBUG ///< Enable / Disable visualization


 
#include <mrpt/utils/CTicTac.h>

#ifdef USE_VISUALIZATION_DEBUG
	#include <mrpt/gui.h>	
	#include <mrpt/base.h>
	#include <mrpt/opengl.h>
	#include <GL/gl.h>
	#include "CMyEllipsoid.h"
#endif

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ndt_map.h>
#include <tf/transform_broadcaster.h>

#include "ndt_mcl.hpp"



/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Visualization stuff
/////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_VISUALIZATION_DEBUG
#include "mcl_visualization.hpp" ///< here is a punch of visualization code based on the MRPT's GUI components

#endif


/**
 * Convert x,y,yaw to Eigen::Affine3d 
 */ 
Eigen::Affine3d getAsAffine(float x, float y, float yaw ){
	Eigen::Matrix3d m;
	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x,y,0);
	Eigen::Affine3d T = v*m;
	
	return T;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Update measurement
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
NDTMCL<pcl::PointXYZ> *ndtmcl;
///Laser sensor offset
float offx = 0;
float offy = 0;
float offa = 0;
static bool has_sensor_offset_set = false;
static bool isFirstLoad=true;
Eigen::Affine3d Told,Todo;

ros::Publisher mcl_pub; ///< The output of MCL is published with this!
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
bool sendROSOdoMessage(Eigen::Vector3d mean,Eigen::Matrix3d cov, ros::Time ts){
	nav_msgs::Odometry O;
	static int seq = 0;
	O.header.stamp = ts;
	O.header.seq = seq;
	O.header.frame_id = "/world";
	O.child_frame_id = "/mcl_pose";
	
	O.pose.pose.position.x = mean[0];
	O.pose.pose.position.y = mean[1];
	tf::Quaternion q;
	q.setRPY(0,0,mean[2]);
	O.pose.pose.orientation.x = q.getX();
	O.pose.pose.orientation.y = q.getY();
	O.pose.pose.orientation.z = q.getZ();
	O.pose.pose.orientation.w = q.getW();
	

	/**
	O.pose.covariance = { cov(0,0), cov(0,1), 0 , 0 , 0 , 0,
											 cov(1,0), cov(1,1), 0 , 0 , 0 , 0,
											 0       ,     0   , 0 , 0 , 0 , 0,
											 0       ,     0   , 0 , 0 , 0 , 0,
											 0       ,     0   , 0 , 0 , 0 , 0,
											 0       ,     0   , 0 , 0 , 0 , cov(2,2)};*/
	
	O.pose.covariance[0] = cov(0,0);
	O.pose.covariance[1] = cov(0,1);
	O.pose.covariance[2] = 0;
	O.pose.covariance[3] = 0;
	O.pose.covariance[4] = 0;
	O.pose.covariance[5] = 0;
	
	O.pose.covariance[6] = cov(1,0);
	O.pose.covariance[7] = cov(1,1);
	O.pose.covariance[8] = 0;
	O.pose.covariance[9] = 0;
	O.pose.covariance[10] = 0;
	O.pose.covariance[11] = 0;
	
	O.pose.covariance[12] = 0;
	O.pose.covariance[13] = 0;
	O.pose.covariance[14] = 0;
	O.pose.covariance[15] = 0;
	O.pose.covariance[16] = 0;
	O.pose.covariance[17] = 0;
	
	O.pose.covariance[18] = 0;
	O.pose.covariance[19] = 0;
	O.pose.covariance[20] = 0;
	O.pose.covariance[21] = 0;
	O.pose.covariance[22] = 0;
	O.pose.covariance[23] = 0;
	
	O.pose.covariance[24] = 0;
	O.pose.covariance[25] = 0;
	O.pose.covariance[26] = 0;
	O.pose.covariance[27] = 0;
	O.pose.covariance[28] = 0;
	O.pose.covariance[29] = 0;
	
	O.pose.covariance[30] = 0;
	O.pose.covariance[31] = 0;
	O.pose.covariance[32] = 0;
	O.pose.covariance[33] = 0;
	O.pose.covariance[34] = 0;
	O.pose.covariance[35] = cov(2,2);
	
	seq++;
	mcl_pub.publish(O);
	
	static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(mean[0],mean[1], 0.0) );
  
	transform.setRotation( q );
  br.sendTransform(tf::StampedTransform(transform, ts, "world", "mcl_pose"));
	
	return true;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
mrpt::utils::CTicTac	TT;

std::string tf_odo_topic =   "odom_base_link";
std::string tf_state_topic = "base_link";
std::string tf_laser_link =  "base_laser_link";

/**
 * Callback for laser scan messages 
 */
void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	static int counter = 0;
	counter++;
	
	static tf::TransformListener tf_listener;
	double looptime = TT.Tac();
	TT.Tic();
	fprintf(stderr,"Lt( %.1lfms %.1lfHz seq:%d) -",looptime*1000,1.0/looptime,scan->header.seq);
	
	if(has_sensor_offset_set == false) return;
	double gx,gy,gyaw,x,y,yaw;
	
	///Get state information
	tf::StampedTransform transform;
	tf_listener.waitForTransform("world", tf_state_topic, scan->header.stamp,ros::Duration(1.0));
	///Ground truth --- Not generally available so should be changed to the manual initialization
	try{
		tf_listener.lookupTransform("world", tf_state_topic, scan->header.stamp, transform);
		gyaw = tf::getYaw(transform.getRotation());  
	  gx = transform.getOrigin().x();
	  gy = transform.getOrigin().y();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
	///Odometry 
	try{
		tf_listener.lookupTransform("world", tf_odo_topic, scan->header.stamp, transform);
		yaw = tf::getYaw(transform.getRotation());  
	  x = transform.getOrigin().x();
	  y = transform.getOrigin().y();
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	
		
	mrpt::utils::CTicTac	tictac;
	tictac.Tic();
	
	///Number of scans
	int N =(scan->angle_max - scan->angle_min)/scan->angle_increment;
	/////
	/// Pose conversions
	////
	
	Eigen::Affine3d T = getAsAffine(x,y,yaw);
	Eigen::Affine3d Tgt = getAsAffine(gx,gy,gyaw);
	
	
	if(isFirstLoad){
		fprintf(stderr,"Initializing to (%lf, %lf, %lf)\n",gx,gy,gyaw);
		/// Initialize the particle filter
		ndtmcl->initializeFilter(gx, gy,gyaw,0.2, 0.2, 2.0*M_PI/180.0, 150);
		Told = T;
		Todo = Tgt;
	}
	
	///Calculate the differential motion from the last frame
	Eigen::Affine3d Tmotion = Told.inverse() * T;
	Todo = Todo*Tmotion; ///< just integrates odometry for the visualization
	
	if(isFirstLoad==false){
		if( (Tmotion.translation().norm()<0.005 && fabs(Tmotion.rotation().eulerAngles(0,1,2)[2])<(0.2*M_PI/180.0))){
			Eigen::Vector3d dm = ndtmcl->getMean();
			Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances();
			sendROSOdoMessage(dm,cov, scan->header.stamp);
			double Time = tictac.Tac();
			fprintf(stderr,"Time elapsed %.1lfms\n",Time*1000);
			return;
		}
	}
	
	Told = T;
	
	///Calculate the laser pose with respect to the base
	float dy =offy;
	float dx = offx;
	float alpha = atan2(dy,dx);
	float L = sqrt(dx*dx+dy*dy);
	
	///Laser pose in base frame
	float lpx = L * cos(alpha);
	float lpy = L * sin(alpha);
	float lpa = offa;
	
	///Laser scan to PointCloud expressed in the base frame
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);	
	for (int j=0;j<N;j++){
		double r  = scan->ranges[j];
		if(r>=scan->range_min && r<scan->range_max && r>0.3 && r<20.0){
			double a  = scan->angle_min + j*scan->angle_increment;
			pcl::PointXYZ pt;
			pt.x = r*cos(a+lpa)+lpx;
			pt.y = r*sin(a+lpa)+lpy;
			pt.z = 0.1+0.02 * (double)rand()/(double)RAND_MAX;
			cloud->push_back(pt);
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/// Now we have the sensor origin and pointcloud -- Lets do MCL
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	ndtmcl->updateAndPredict(Tmotion, *cloud); ///<predicts, updates and resamples if necessary (ndt_mcl.hpp)
	
	Eigen::Vector3d dm = ndtmcl->getMean(); ///Maximum aposteriori pose
	Eigen::Matrix3d cov = ndtmcl->pf.getDistributionVariances(); ///Pose covariance
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double Time = tictac.Tac();
	fprintf(stderr,"Time elapsed %.1lfms (%lf %lf %lf) \n",Time*1000,dm[0],dm[1],dm[2]);
	isFirstLoad = false;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	sendROSOdoMessage(dm,cov, scan->header.stamp); ///Spit out the pose estimate
	
	///This is all for visualization
#ifdef USE_VISUALIZATION_DEBUG
	Eigen::Vector3d origin(dm[0] + L * cos(dm[2]+alpha),dm[1] + L * sin(dm[2]+alpha),0.1);
	Eigen::Affine3d ppos = getAsAffine(dm[0],dm[1],dm[2]);
	
	//lslgeneric::transformPointCloudInPlace(Tgt, *cloud);
	lslgeneric::transformPointCloudInPlace(ppos, *cloud);
	mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();	
	win3D.setCameraPointingToPoint(gx,gy,1);
	if(counter%2000==0) gl_points->clear();
	scene->clear();
	scene->insert(plane);
	
	addMap2Scene(ndtmcl->map, origin, scene);
	addPoseCovariance(dm[0],dm[1],cov,scene);
	addScanToScene(scene, origin, cloud);
	addParticlesToWorld(ndtmcl->pf,Tgt.translation(),dm, Todo.translation());
	scene->insert(gl_points);
	scene->insert(gl_particles);
	win3D.unlockAccess3DScene();
	win3D.repaint();

	if (win3D.keyHit())
	{
		mrpt::gui::mrptKeyModifier kmods;
		int key = win3D.getPushedKey(&kmods);
	}
#endif
}


int main(int argc, char **argv){
	ros::init(argc, argv, "NDT-MCL");
	double resolution=0.4;
	
#ifdef USE_VISUALIZATION_DEBUG	
	initializeScene();
#endif

	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle paramHandle ("~");
	TT.Tic();
	
	std::string tf_base_link,input_laser_topic; 
	
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Parameters for the mapper
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	bool loadMap = false; ///< flag to indicate that we want to load a map
	std::string mapName("basement.ndmap"); ///<name and the path to the map
	
	bool saveMap = true;						///< indicates if we want to save the map in a regular intervals
	std::string output_map_name = std::string("ndt_mapper_output.ndmap");
	
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Set the values from a config file
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	
	paramHandle.param<std::string>("input_laser_topic", input_laser_topic, std::string("/base_scan"));
	paramHandle.param<std::string>("tf_base_link", tf_state_topic, std::string("/base_link"));
	paramHandle.param<std::string>("tf_laser_link", tf_laser_link, std::string("/hokuyo1_link"));
	
	bool use_sensor_pose;
	paramHandle.param<bool>("use_sensor_pose", use_sensor_pose, false);
	double sensor_pose_x, sensor_pose_y, sensor_pose_th;
	paramHandle.param<double>("sensor_pose_x", sensor_pose_x, 0.);
	paramHandle.param<double>("sensor_pose_y", sensor_pose_y, 0.);
	paramHandle.param<double>("sensor_pose_th", sensor_pose_th, 0.);
	
	paramHandle.param<bool>("load_map_from_file", loadMap, false);
	paramHandle.param<std::string>("map_file_name", mapName, std::string("basement.ndmap"));
	
	paramHandle.param<bool>("save_output_map", saveMap, true);
	paramHandle.param<std::string>("output_map_file_name", output_map_name, std::string("ndt_mapper_output.ndmap"));
	
	paramHandle.param<double>("map_resolution", resolution , 0.2);
	bool forceSIR=false;
	paramHandle.param<bool>("forceSIR", forceSIR, false);
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Prepare the map
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	fprintf(stderr,"USING RESOLUTION %lf\n",resolution);
	lslgeneric::NDTMap<pcl::PointXYZ> ndmap(new lslgeneric::LazyGrid<pcl::PointXYZ>(resolution));

	ndmap.setMapSize(80.0, 80.0, 1.0);
	
	if(loadMap){
		fprintf(stderr,"Loading Map from '%s'\n",mapName.c_str());
		ndmap.loadFromJFF(mapName.c_str());
	}
	
	ndtmcl = new NDTMCL<pcl::PointXYZ>(resolution,ndmap,-0.5);
	if(forceSIR) ndtmcl->forceSIR=true;

	fprintf(stderr,"*** FORCE SIR = %d****",forceSIR);
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	///Set up our output
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	mcl_pub = nh.advertise<nav_msgs::Odometry>("sauna_mcl",10);
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	/// Set the subscribers and setup callbacks and message filters
	//////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////
	ros::Subscriber scansub=nh.subscribe(input_laser_topic, 1, callback);


	offa = sensor_pose_th;
	offx = sensor_pose_x;
	offy = sensor_pose_y;
	
	
	has_sensor_offset_set = true;
	
	fprintf(stderr,"Sensor Pose = (%lf %lf %lf)\n",offx, offy, offa);	
	
	ros::spin();

	
	return 0;
}
