//#ifndef DO_DEBUG_PROC
//#define DO_DEBUG_PROC
//#endif

#include <ndt_matcher_p2d.h>
#include <ndt_matcher_d2d.h>
//#include <ndt_matcher_d2d.hh>
#include <ndt_map.h>
#include <oc_tree.h>
#include <pointcloud_utils.h>

#include "ros/ros.h"
#include "pcl/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"
#include "pcl/features/feature.h"
#include "pcl/registration/icp.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/filters/passthrough.h>

#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>

using namespace std;


/*
static int ctr = 0;
bool matchICP(pcl::PointCloud<pcl::PointXYZ> &fixed,  pcl::PointCloud<pcl::PointXYZ> &moving,
	      Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tout) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr f (new pcl::PointCloud<pcl::PointXYZ>(fixed) );
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr m (new pcl::PointCloud<pcl::PointXYZ>(moving) );

    pcl::VoxelGrid<pcl::PointXYZ> gr1,gr2;
    gr1.setLeafSize(0.1,0.1,0.1);
    gr2.setLeafSize(0.1,0.1,0.1);

    gr1.setInputCloud(m);
    gr2.setInputCloud(f);

    cloud_in->height = 1;
    cloud_in->width = cloud_in->points.size();
    cloud_out->height = 1;
    cloud_out->width = cloud_out->points.size();
    cloud_in->is_dense = false;
    cloud_out->is_dense = false;

    gr1.filter(*cloud_in);
    gr2.filter(*cloud_out);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setMaximumIterations(10000);
    cout<<"max itr are "<<icp.getMaximumIterations()<<endl;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);

    icp.setRANSACOutlierRejectionThreshold (2);
    icp.setMaxCorrespondenceDistance(10);
    icp.setTransformationEpsilon(0.00001);
//    cout<<"ransac outlier thersh   : "<<icp.getRANSACOutlierRejectionThreshold ()<<endl;
//    cout<<"correspondance max dist : "<<icp.getMaxCorrespondenceDistance() << endl;
//    cout<<"epsilon : "<<icp.getTransformationEpsilon() << endl;
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);


//    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
//	icp.getFitnessScore() << std::endl;
//    std::cout << icp.getFinalTransformation() << std::endl;

    //Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> tTemp;
    Tout = (icp.getFinalTransformation()).cast<double>();

    char fname[50];
    snprintf(fname,49,"/home/tsv/ndt_tmp/c2_offset.wrl");
    FILE *fout = fopen(fname,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    lslgeneric::writeToVRML(fout,*cloud_out,Eigen::Vector3d(0,1,0));
    lslgeneric::writeToVRML(fout,Final,Eigen::Vector3d(1,0,0));
    lslgeneric::writeToVRML(fout,*cloud_in,Eigen::Vector3d(1,1,1));
    fclose(fout);

    return icp.hasConverged();

}
*/

int
main (int argc, char** argv)
{

  //  double roll,pitch,yaw,xoffset,yoffset,zoffset;

    pcl::PointCloud<pcl::PointXYZ> cloud, cloud_offset, cloud_OFF;
    pcl::PCDReader reader;
    pcl::PCDWriter writer;

    double __res[] = {0.5, 1, 2, 4};
    std::vector<double> resolutions (__res, __res+sizeof(__res)/sizeof(double));
    lslgeneric::NDTMatcherD2D<pcl::PointXYZ,pcl::PointXYZ> matcherD2D(false, false, resolutions);
    lslgeneric::NDTMatcherP2D<pcl::PointXYZ,pcl::PointXYZ> matcherP2D(resolutions);

    struct timeval tv_start,tv_end,tv_reg_start,tv_reg_end;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tin, Tout, Tout2;
    Tout.setIdentity();
    if(argc == 3) {
	
	gettimeofday(&tv_start,NULL);
	//we do a single scan to scan registration
	//reader.read(argv[1],cloud);
	//reader.read(argv[2],cloud_offset);
	cloud = lslgeneric::readVRML<pcl::PointXYZ>(argv[1]);
	cloud_offset = lslgeneric::readVRML<pcl::PointXYZ>(argv[2]);

/*	pcl::PointCloud<pcl::PointXYZ> cloud1, cloud_offset1;
	for(int i=0; i<cloud.points.size(); i++) {
	    double d = sqrt(pow(cloud.points[i].x,2) + pow(cloud.points[i].y,2) +pow(cloud.points[i].z,2));
	    if(d<3 && d>1) {
		cloud1.points.push_back(cloud.points[i]);
	    }
	}
	for(int i=0; i<cloud_offset.points.size(); i++) {
	    double d = sqrt(pow(cloud_offset.points[i].x,2) + pow(cloud_offset.points[i].y,2) +pow(cloud_offset.points[i].z,2));
	    if(d<3 && d>1) {
		cloud_offset1.points.push_back(cloud_offset.points[i]);
	    }
	}
*/
//	bool ret = matcherP2D.match(cloud,cloud_offset,Tout2);
	
	gettimeofday(&tv_reg_start,NULL);
	bool ret = matcherD2D.match(cloud,cloud_offset,Tout);
	gettimeofday(&tv_reg_end,NULL);

	lslgeneric::transformPointCloudInPlace<pcl::PointXYZ>(Tout,cloud_offset);
	cout<<"translation "<<Tout.translation().transpose()<<endl;
	cout<<"euler: "<<Tout.rotation().eulerAngles(0,1,2).transpose()<<endl;

	// cloud_offset += cloud;
	char fname[50];
	snprintf(fname,49,"c_offset.pcd");
	// writer.write(fname, cloud_offset);

	
	snprintf(fname,49,"c_offset.wrl");
	FILE *fout = fopen(fname,"w");
	fprintf(fout,"#VRML V2.0 utf8\n");
	lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud,Eigen::Vector3d(0,1,0));
	lslgeneric::writeToVRML<pcl::PointXYZ>(fout,cloud_offset,Eigen::Vector3d(1,0,0));
	fclose(fout);
	
	cout<<"initial registration done, saved into file c_offset.pcd\n";
	cout<<"make sure to check that initial registration was successful!\n";

	gettimeofday(&tv_end,NULL);
	double tim = (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;
	double tim2 = (tv_reg_end.tv_sec-tv_reg_start.tv_sec)*1000.+(tv_reg_end.tv_usec-tv_reg_start.tv_usec)/1000.;

	cout<<"OVERALL TIME: "<<tim<<" REG TIME "<<tim2<<endl;
	return 0;
    }
#if 0
    if(!(argc == 2 || succ)) {
	cout<<"usage: ./test_ndt point_cloud1 [point_cloud2]\n";
	return -1;
    }

    cloud = lslgeneric::readVRML(argv[1]);

    char fname[50];
    FILE *fout;
    if(!succ) {
	cloud_offset = cloud;
    } else {
	cloud_offset = cloud_OFF;
    }

    //matcherF2F.generateScoreDebug("/home/tsv/ndt_tmp/scoresF2F.m",cloud,cloud_offset);
    //matcherP2F.generateScoreDebug("/home/tsv/ndt_tmp/scoresP2F.m",cloud,cloud_offset);
    //cout<<"Generated scores\n";
    //return 0;

    std::vector<double> times;
    std::vector<int> success;
    std::vector<double> et;
    std::vector<double> er;

    roll = 0; pitch =0; yaw = 40*M_PI/180;
    xoffset = -2; yoffset = -1.; zoffset =0;
    //for(roll = -2*M_PI/5;roll<M_PI/2;roll+=M_PI/5) {
    //for(pitch = -2*M_PI/5;pitch<M_PI/2;pitch+=M_PI/5) {
    for(yaw = -30*M_PI/180;yaw<=35*M_PI/180;yaw+=10*M_PI/180) {
    logger2<<"%";
    for(xoffset=-1.5;xoffset<=1.5;xoffset+=0.5) {
    for(yoffset=-1.5;yoffset<=1.5;yoffset+=0.5) {
    /*
    for(yaw = -5*M_PI/180;yaw<=15*M_PI/180;yaw+=10*M_PI/180) {
    for(xoffset=-0.1;xoffset<=0.5;xoffset+=0.5) {
    for(yoffset=0;yoffset<=0.5;yoffset+=0.5) {
    */
    //for(zoffset = -1;zoffset<1;zoffset+=0.5) {


    Tin =  Eigen::Translation<double,3>(xoffset,yoffset,zoffset)*
	Eigen::AngleAxis<double>(roll,Eigen::Vector3d::UnitX()) *
	Eigen::AngleAxis<double>(pitch,Eigen::Vector3d::UnitY()) *
	Eigen::AngleAxis<double>(yaw,Eigen::Vector3d::UnitZ()) ;

    //cout<<"translation \n"<<Tin.translation().transpose()<<endl;
    //cout<<"rotation \n"<<Tin.rotation()<<endl;

    cout<<roll<<" "<<pitch<<" "<<yaw<<" "<<xoffset<<" "<<yoffset<<" "<<zoffset<<endl;
    logger<<roll<<" "<<pitch<<" "<<yaw<<" "<<xoffset<<" "<<yoffset<<" "<<zoffset<<endl;

    Eigen::Vector3d out = Tin.rotation().eulerAngles(0,1,2);
    //cout<<"euler: "<<out<<endl;

    if(!succ) {
	cloud_offset = lslgeneric::transformPointCloud(Tin,cloud);
    } else {
	cloud_offset = lslgeneric::transformPointCloud(Tin,cloud_OFF);
    }

    /*snprintf(fname,49,"/home/tsv/ndt_tmp/cloud_init%05d.wrl",ctr);
    fout = fopen(fname,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    lslgeneric::writeToVRML(fout,cloud_offset,Eigen::Vector3d(1,0,0));
    lslgeneric::writeToVRML(fout,cloud,Eigen::Vector3d(0,1,0));
    fclose(fout);
*/
    gettimeofday(&tv_start,NULL);
    //bool ret = matcherP2F.match(cloud,cloud_offset,Tout);
    bool ret = matcherF2F.match(cloud,cloud_offset,Tout);
    //bool ret = matchICP(cloud,cloud_offset,Tout);
    gettimeofday(&tv_end,NULL);

    double tim = (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.;
    logger<<"TIME: "<<tim<<endl;
    C_TIME+=tim;

    if(!ret) {
	logger<<">>>>>>> NO SOLUTION <<<<<<<<<\n";
	cout<<"NO SOLUTION\n";
	logger2<<"N ";
	N_FAIL++;
	times.push_back(tim);
	et.push_back(Tout.translation().norm());
	er.push_back(Tout.rotation().eulerAngles(0,1,2).norm());
    } else {
	/*logger<<"Input Transform: "<<endl;
	logger<<"translation "<<Tin.translation()<<endl;
	logger<<"rotation "<<Tin.rotation()<<endl;
	logger<<"translation "<<Tout.translation()<<endl;
	logger<<"rotation "<<Tout.rotation()<<endl;
	*/
	//lslgeneric::transformPointCloudInPlace(Tout,cloud_offset);
	Eigen::Vector3d out = Tout.rotation().eulerAngles(0,1,2);
	logger<<"OUT: "<<out.transpose()<<endl;
	//cout<<"OUT: "<<out<<endl;
	logger<<"translation "<<Tout.translation().transpose()<<endl;
	//cout<<"translation "<<Tout.translation()<<endl;
	snprintf(fname,49,"/home/tsv/ndt_tmp/cloud_offset%05d.wrl",ctr);
	lslgeneric::transformPointCloudInPlace(Tout,cloud_offset);

	/*fout = fopen(fname,"w");
	fprintf(fout,"#VRML V2.0 utf8\n");
	lslgeneric::writeToVRML(fout,cloud_offset,Eigen::Vector3d(1,0,0));
	lslgeneric::writeToVRML(fout,cloud,Eigen::Vector3d(0,1,0));
	fclose(fout);
	*/
	ctr++;

	Tout = Tin*Tout;
	logger<<"E translation "<<Tout.translation().transpose()
	      <<" (norm) "<<Tout.translation().norm()<<endl;
	logger<<"E rotation "<<Tout.rotation().eulerAngles(0,1,2).transpose()
	      <<" (norm) "<<Tout.rotation().eulerAngles(0,1,2).norm()<<endl;

	times.push_back(tim);
	et.push_back(Tout.translation().norm());
	er.push_back(Tout.rotation().eulerAngles(0,1,2).norm());

	if(Tout.translation().norm() < 0.2 && Tout.rotation().eulerAngles(0,1,2).norm() < 0.087) {
	    logger<<"OK\n";
	    logger2<<"O ";
	    cout<<"OK\n";
	    N_SUCC++;
	    success.push_back(0);
	}
	else if(Tout.translation().norm() < 1 && Tout.rotation().eulerAngles(0,1,2).norm() < 0.15) {
	    logger<<"ACK\n";
	    logger2<<"A ";
	    cout<<"ACK\n";
	    success.push_back(1);
	}
	else {
	    logger<<"FAIL\n";
	    logger2<<"F ";
	    cout<<"FAIL\n";
	    N_FAIL++;
	    success.push_back(2);
	}

	return 0;
	//cout<<"E translation "<<Tout.translation()<<endl;
	//cout<<"E rotation "<<Tout.rotation()<<endl;
    }
    N_TRIALS++;
    logger <<"-------------------------------------------\n";
    }
    }
    logger2<<"\n";
    logger2.flush();


    }
    //}}}}}

    cout<<"Trials:  "<<N_TRIALS<<endl;
    cout<<"Success: "<<N_SUCC<<"  Rate = "<<(double)N_SUCC/(double)N_TRIALS<<endl;
    cout<<"Success + Acc : "<<N_TRIALS-N_FAIL<<"  Rate = "<<1-(double)N_FAIL/(double)N_TRIALS<<endl;
    cout<<"Fail:    "<<N_FAIL<<endl;
    cout<<"Runtime  "<<C_TIME/N_TRIALS<<endl;

    logger2<<"Trials:  "<<N_TRIALS<<endl;
    logger2<<"Success: "<<N_SUCC<<"  Rate = "<<(double)N_SUCC/(double)N_TRIALS<<endl;
    logger2<<"Success + Acc : "<<N_TRIALS-N_FAIL<<"  Rate = "<<1-(double)N_FAIL/(double)N_TRIALS<<endl;
    logger2<<"Fail:    "<<N_FAIL<<endl;
    logger2<<"Runtime  "<<C_TIME/N_TRIALS<<endl;

    logger2<<"Times = [";
    for(int i=0; i<times.size(); i++) {
	logger2<<times[i]<<" ";
    }
    logger2<<"];\n";

    logger2<<"ER = [";
    for(int i=0; i<er.size(); i++) {
	logger2<<er[i]<<" ";
    }
    logger2<<"];\n";

    logger2<<"ET = [";
    for(int i=0; i<et.size(); i++) {
	logger2<<et[i]<<" ";
    }
    logger2<<"];\n";

    logger2<<"Succ = [";
    for(int i=0; i<success.size(); i++) {
	logger2<<success[i]<<" ";
    }
    logger2<<"];\n";

    return (0);
#endif
}



