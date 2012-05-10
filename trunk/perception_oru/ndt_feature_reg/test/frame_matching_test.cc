#include <iostream>
#include <boost/program_options.hpp>
#include <cv.h>
#include <highgui.h>
#include "pcl/io/pcd_io.h"
#include <pcl/filters/passthrough.h>

#include <ndt_feature_reg/Frame.hh>
#include <NDTMatcherF2F.hh>
#include <NDTMatcherFeatureF2F.hh>
#include <PointCloudUtils.hh>

using namespace std;
using namespace ndt_feature_reg;
using namespace lslgeneric;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
     cout << "--------------------------------------------------" << endl;
     cout << "Small test program of the frame matching between 2" << endl;
     cout << "pair of depth + std (RGB) images. Each image pair " << endl;
     cout << "is assumed to have 1-1 correspondance, meaning    " << endl;
     cout << "that for each pixel in the std image we have the  " << endl;
     cout << "corresponding depth / disparity pixel at the same " << endl;
     cout << "location in the depth image.                      " << endl;
     cout << "--------------------------------------------------" << endl;

     string std1_name, depth1_name, std2_name, depth2_name, pc1_name, pc2_name;
     double scale, max_inldist_xy, max_inldist_z;
     Eigen::Matrix<double,6,1> pose_increment_v;
     int nb_ransac;
     
     po::options_description desc("Allowed options");
     desc.add_options()
	  ("help", "produce help message")
	  ("debug", "print debug output")
	  ("visualize", "visualize the output")
	  ("depth1", po::value<string>(&depth1_name), "first depth image")
	  ("depth2", po::value<string>(&depth2_name), "second depth image")
	  ("std1", po::value<string>(&std1_name), "first standard image")
	  ("std2", po::value<string>(&std2_name), "second standard image")
	  ("pc1", po::value<string>(&pc1_name), "first pointcloud")
	  ("pc2", po::value<string>(&pc2_name), "second pointcloud")
	  ("scale", po::value<double>(&scale)->default_value(0.0002), "depth scale (depth = scale * pixel value)")
	  ("max_inldist_xy", po::value<double>(&max_inldist_xy)->default_value(0.1), "max inlier distance in xy related to the camera plane (in meters)")
	  ("max_inldist_z", po::value<double>(&max_inldist_z)->default_value(0.1), "max inlier distance in z - depth (in meters)")
	  ("nb_ransac", po::value<int>(&nb_ransac)->default_value(100), "max number of RANSAC iterations")
	  ;

     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     po::notify(vm);    
     
     if (vm.count("help")) {
	  cout << desc << "\n";
	  return 1;
     }
     if ((vm.count("pc1") && vm.count("depth1")) || (vm.count("pc2") && vm.count("depth2"))) 
     {
	  cout << "ERROR: both a poincloud and a depth image are provided as input  - quitting\n";
	  return 1;
     }
     bool debug = vm.count("debug");
     bool visualize = vm.count("visualize");
     if (debug)
	  cout << "scale : " << scale << endl;

     // TODO - this are the values from the Freiburg 1 camera.
     double fx = 517.3;
     double fy = 516.5;
     double cx = 318.6;
     double cy = 255.3;
     double d0 = 0.2624;
     double d1 = -0.9531;
     double d2 = -0.0054;
     double d3 = 0.0026;
     double d4 = 1.1633; 
     double ds = 1.035; // Depth scaling factor.
    
     cv::Mat camera_mat = getCameraMatrix(fx, fy, cx, cy);
     cv::Mat dist_vec = getDistVector(d0, d1, d2, d3, d4);
     
     cv::Mat std1,std2,depth1,depth2, lookup_table;
     // Load the files.
     pcl::PCDReader reader;
     double t1,t2,t3;

     if (debug)
	  cout << "loading imgs : " << std1_name << " and " << std2_name << endl;
     NDTFrame frame1, frame2;
     frame1.img = cv::imread(std1_name, 0);
     frame2.img = cv::imread(std2_name, 0);
     
     if (vm.count("pc1") && vm.count("pc2"))
     {
	  if (debug)
	       cout << "loading pc : " << pc1_name << " and " << pc2_name << endl;
	  reader.read<pcl::PointXYZ>(pc1_name, frame1.pc);
	  reader.read<pcl::PointXYZ>(pc2_name, frame2.pc);
     }
     else if (vm.count("depth1") && vm.count("depth2"))
     {
	  if (debug)
	       cout << "loading depth img : " << depth1_name << " and " << depth2_name << endl;
	  frame1.depth_img = cv::imread(depth1_name, CV_LOAD_IMAGE_ANYDEPTH); // CV_LOAD_IMAGE_ANYDEPTH is important to load the 16bits image
	  frame2.depth_img = cv::imread(depth2_name, CV_LOAD_IMAGE_ANYDEPTH);

	  if (debug)
	  {
	       cout << "using camera matrix : \n" << camera_mat << endl;
	       cout << "using distortion vec: \n" << dist_vec << endl;
	       cout << "depth1.depth() : " << frame1.depth_img.depth() << endl;
	       cout << "depth1.channels() : " << frame1.depth_img.channels() << endl;
	  }
	  if (visualize)
	  {
//	       cv::imshow("frame1.depth_img", frame1.depth_img);
//	       cv::waitKey(0);
	  }
	  
	  t1 = getDoubleTime();
	  lookup_table = getDepthPointCloudLookUpTable(frame1.depth_img.size(), camera_mat, dist_vec, ds*scale);
	  t2 = getDoubleTime();
	  if (debug)
	  {
	       cout << "lookup: " << t2-t1 << endl;
	       t1 = getDoubleTime();
	       convertDepthImageToPointCloud(frame1.depth_img, lookup_table, frame1.pc);
	       t2 = getDoubleTime();
	       convertDepthImageToPointCloud(frame1.depth_img, lookup_table, frame1.pc);
	       t3 = getDoubleTime();
	       cout << "convert: " << t2-t1 << endl;
	       cout << "convert2: " << t3-t2 << endl;
	       convertDepthImageToPointCloud(frame2.depth_img, lookup_table, frame2.pc);
	       pcl::io::savePCDFileASCII ("frame1_depth.pcd", frame1.pc);
	       pcl::io::savePCDFileASCII ("frame2_depth.pcd", frame2.pc);
	  }
     }     
     else
     {
	  cout << "Check depth data input - quitting\n";
	  return 1;
     }
     
     cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create("SURF");
     cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create("SURF");
     t1 = getDoubleTime();
     detector->detect(frame1.img, frame1.kpts);
     t2 = getDoubleTime();
     detector->detect(frame2.img, frame2.kpts);
     
     cv::Mat img_draw1, img_draw2;
     cv::drawKeypoints(frame1.img, frame1.kpts, img_draw1, cv::Scalar(0,255,0));
     cv::drawKeypoints(frame2.img, frame2.kpts, img_draw2, cv::Scalar(0,255,0));
     
     if (visualize)
     {
//	  cv::imshow("frame1.img", img_draw1);
//	  cv::imshow("frame2.img", img_draw2);
//	  cv::waitKey(0);
     }
     if (debug)
     	  cout << "Computing NDT ..." << endl;
     t1 = getDoubleTime();
     frame1.computeNDT();
     t2 = getDoubleTime();
     cout << "compute NDT : " << t2 - t1 << endl;
     frame2.computeNDT();
     if (debug)
     	  cout << " - done." << endl;

     std::cout << "detect() : " << t2 - t1 << std::endl;
     extractor->compute(frame1.img, frame1.kpts, frame1.dtors);
     extractor->compute(frame2.img, frame2.kpts, frame2.dtors);
  
     if (debug)
     {
	  pcl::PointCloud<pcl::PointXYZRGB> pc_color;
	  cout << "create colored point cloud" << endl;
	  cout << "frame2.img.width : " << frame2.img.cols << endl;
	  cout << "frame2.img.height : " << frame2.img.rows << endl;
	  cout << "pc.width : " << frame2.pc.width << endl;
	  cout << "pc.height : " << frame2.pc.height << endl;
	  createColoredPointCloud(frame2.img, frame2.pc, pc_color);
	  colorKeyPointsInPointCloud(pc_color, frame2.kpts_pc_indices);
	  cout << "create colored point cloud - done" << endl;
	  pcl::io::savePCDFileASCII ("frame2_color.pcd", pc_color);
     }

     // Do matching
     PoseEstimator pe(nb_ransac, max_inldist_xy, max_inldist_z);
     frame1.assignPts();
     frame2.assignPts();
	  
     pe.estimate(frame1, frame2);

     if (visualize)
     {
	  cv::Mat display;
	  drawMatches(frame1.img, frame1.kpts, frame2.img, frame2.kpts, pe.inliers, display);
	  const std::string window_name = "matches";
	  cv::namedWindow(window_name,0);
	  cv::imshow(window_name, display);
	  cv::waitKey(0);
     }
     
     

     frame1.ndt_map.writeToVRML("/home/han/tmp/ndt1.wrl");
     frame2.ndt_map.writeToVRML("/home/han/tmp/ndt2.wrl");
     
     if (debug)
	  cout << "Creating key point cloud ..." << endl;
     frame1.createKeyPointCloud();
     frame2.createKeyPointCloud();
     if (debug)
	  cout << "Creating key point cloud - done." << endl;

     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> transform, transform_p2p, transform_ndt2ndt, transform_ndt2ndt_corr, transform_p2p_corr, transform_p2p_corr_rot, transform_p2p_corr_transl;
     transform_p2p.setIdentity();
     transform_ndt2ndt.setIdentity();
     transform_ndt2ndt_corr.setIdentity();
     transform_p2p_corr.setIdentity();
//     transform_p2p_corr.translate(pe.trans).rotate(pe.quat);
//     transform_p2p_corr.rotate(pe.quat).translate(pe.trans);
     transform_p2p_corr.rotate(pe.rot);
     transform_p2p_corr.translate(pe.trans);
     
#if 0
     transform = Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
	  Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
	  Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
	  Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;
#endif     
     NDTMatcherF2F matcher;

     transform.setIdentity();
     // Not possible due to that these pc's are dense and contains NaN's.
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(frame1.pc.makeShared());
     pass.filter(frame1.pc);
     pass.setInputCloud(frame2.pc.makeShared());
     pass.filter(frame2.pc);

     if (debug)
	  cout << "Matching ..." << endl;
     t1 = getDoubleTime();
     matcher.match(frame1.pc, frame2.pc, transform);
     transform_p2p = transform;
     t2 = getDoubleTime();
     std::cout << "p2p matching : " << t2 - t1 << std::endl;
     if (debug)
	  cout << "Matching - done." << endl;
     std::cout<<"est translation (points) "<<transform.translation().transpose()
	      <<" (norm) "<<transform.translation().norm()<<std::endl;
     std::cout<<"est rotation (points) "<<transform.rotation().eulerAngles(0,1,2).transpose()
	      <<" (norm) "<<transform.rotation().eulerAngles(0,1,2).norm()<<std::endl;


     t1 = getDoubleTime();
     matcher.match(frame1.ndt_map, frame2.ndt_map, transform);
     transform_ndt2ndt = transform;
     t2 = getDoubleTime();
     std::cout << "ndt2ndt matching : " << t2 - t1 << std::endl;
     
     std::cout<<"est translation (ndt - no corr) "<<transform.translation().transpose()
	      <<" (norm) "<<transform.translation().norm()<<std::endl;
     std::cout<<"est rotation (ndt - no corr) "<<transform.rotation().eulerAngles(0,1,2).transpose()
	      <<" (norm) "<<transform.rotation().eulerAngles(0,1,2).norm()<<std::endl;


     std::vector<std::pair<int,int> > corr;
     for (size_t i = 0; i < pe.inliers.size(); i++)
     {
	  corr.push_back(std::pair<int,int>(pe.inliers[i].queryIdx, pe.inliers[i].trainIdx));
//	  corr.push_back(std::pair<int,int>(pe.inliers[i].trainIdx, pe.inliers[i].queryIdx));
	  assert(pe.inliers[i].queryIdx < frame1.ndt_map.getMyIndex()->size() && pe.inliers[i].queryIdx >= 0);
	  assert(pe.inliers[i].trainIdx < frame2.ndt_map.getMyIndex()->size() && pe.inliers[i].trainIdx >= 0);
    }
    cout << "frame1.ndt_map.getMyIndex()->size() : " << frame1.ndt_map.getMyIndex()->size() << endl;
     cout << "frame2.ndt_map.getMyIndex()->size() : " << frame2.ndt_map.getMyIndex()->size() << endl;
     NDTMatcherFeatureF2F matcher_feat(corr);
     cout << "Matching ..." << endl;
     t1 = getDoubleTime();
     matcher_feat.match(frame1.ndt_map, frame2.ndt_map, transform);
     transform_ndt2ndt_corr = transform;
     t2 = getDoubleTime();
     cout << "Matching ... - done" << endl;
     std::cout << "ndt2ndt matching : " << t2 - t1 << std::endl;
     
     std::cout<<"est translation (ndt - no corr) "<<transform.translation().transpose()
	      <<" (norm) "<<transform.translation().norm()<<std::endl;
     std::cout<<"est rotation (ndt - no corr) "<<transform.rotation().eulerAngles(0,1,2).transpose()
	      <<" (norm) "<<transform.rotation().eulerAngles(0,1,2).norm()<<std::endl;



     {
	  transform = transform_p2p;
	  char fname[50];
	  snprintf(fname,49,"/home/han/tmp/output_p2p.wrl");
	  FILE *fout = fopen(fname,"w");
	  fprintf(fout,"#VRML V2.0 utf8\n");
	  std::vector<NDTCell*> nextNDT = frame2.ndt_map.pseudoTransformNDT(transform.inverse());
	  
	  for(unsigned int i=0; i<nextNDT.size(); i++) {
	       nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
	       if(nextNDT[i]!=NULL) delete nextNDT[i];
	  }
	  
	  frame1.ndt_map.writeToVRML(fout, false);
	  writeToVRML(fout, frame1.pc);
	  transformPointCloudInPlace(transform, frame2.pc);
	  writeToVRML(fout, frame2.pc);
	  fclose(fout);
     }

     {
	  transform = transform_ndt2ndt;
	  char fname[50];
	  snprintf(fname,49,"/home/han/tmp/output_ndt2ndt.wrl");
	  FILE *fout = fopen(fname,"w");
	  fprintf(fout,"#VRML V2.0 utf8\n");
	  std::vector<NDTCell*> nextNDT = frame2.ndt_map.pseudoTransformNDT(transform.inverse());
	  
	  for(unsigned int i=0; i<nextNDT.size(); i++) {
	       nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
	       if(nextNDT[i]!=NULL) delete nextNDT[i];
	  }
	  
	  frame1.ndt_map.writeToVRML(fout, false);
	  writeToVRML(fout, frame1.pc);
	  transformPointCloudInPlace(transform, frame2.pc);
	  writeToVRML(fout, frame2.pc);
	  fclose(fout);
     }

     {
	  transform = transform_ndt2ndt_corr;
	  char fname[50];
	  snprintf(fname,49,"/home/han/tmp/output_ndt2ndt_corr.wrl");
	  FILE *fout = fopen(fname,"w");
	  fprintf(fout,"#VRML V2.0 utf8\n");
	  std::vector<NDTCell*> nextNDT = frame2.ndt_map.pseudoTransformNDT(transform.inverse());
	  
	  for(unsigned int i=0; i<nextNDT.size(); i++) {
	       nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
	       if(nextNDT[i]!=NULL) delete nextNDT[i];
	  }
	  
	  frame1.ndt_map.writeToVRML(fout, false);
	  writeToVRML(fout, frame1.pc);
	  transformPointCloudInPlace(transform, frame2.pc);
	  writeToVRML(fout, frame2.pc);
	  fclose(fout);
     }

     {
	  transform = transform_p2p_corr;
	  char fname[50];
	  snprintf(fname,49,"/home/han/tmp/output_p2p_corr.wrl");
	  FILE *fout = fopen(fname,"w");
	  fprintf(fout,"#VRML V2.0 utf8\n");
	  std::vector<NDTCell*> nextNDT = frame2.ndt_map.pseudoTransformNDT(transform.inverse());
	  
	  for(unsigned int i=0; i<nextNDT.size(); i++) {
//	       nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
	       if(nextNDT[i]!=NULL) delete nextNDT[i];
	  }
	  
//	  frame1.ndt_map.writeToVRML(fout, false);
	  
	  writeToVRML(fout, frame1.pc);
	  transformPointCloudInPlace(transform, frame2.pc);
	  writeToVRML(fout, frame2.pc, Eigen::Vector3d(0,1,0));
	  fclose(fout);
     }


}
