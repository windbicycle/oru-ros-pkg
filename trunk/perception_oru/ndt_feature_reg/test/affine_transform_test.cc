// Visualizer of feature f2f registration.
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transform.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_types.h>
#include <Eigen/Core>


using namespace pcl;
using namespace std;
    
namespace po = boost::program_options;

int main(int argc, char** argv)
{
     cout << "--------------------------------------------------" << endl;
     cout << "Small test program of affine transforms" << endl;
     cout << "--------------------------------------------------" << endl;

     string pc_name;
     float tx,ty,tz,rx;
     po::options_description desc("Allowed options");
     desc.add_options()
	  ("help", "produce help message")
	  ("debug", "print debug output")
	  ("pc", po::value<string>(&pc_name), "pointcloud .pcd file")
	  ("tx", po::value<float>(&tx), "transl x")
	  ("ty", po::value<float>(&ty), "transl y")
	  ("tz", po::value<float>(&tz), "transl z")
	  ("rx", po::value<float>(&rx), "rotation x-axis")
	  ;
     
     po::variables_map vm;
     po::store(po::parse_command_line(argc, argv, desc), vm);
     po::notify(vm);    
     
     if (vm.count("help")) {
	  cout << desc << "\n";
	  return 1;
     }
     if (!vm.count("pc"))
     {
	  cout << "ERROR: need a point cloud file (.pcd)\n";
	  return 1;
     }
     bool debug = vm.count("debug");
     
     pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PCDReader reader;
     reader.read<pcl::PointXYZ>(pc_name, *pc);

     pcl::visualization::PCLVisualizer viewer("3D Viewer");
     viewer.addPointCloud(pc);

     pcl::PointCloud<pcl::PointXYZ>::Ptr pc_moved(new pcl::PointCloud<pcl::PointXYZ>);
     


     viewer.addCoordinateSystem();
     while (!viewer.wasStopped())
     {
	  Eigen::Affine3f transl_transform = (Eigen::Affine3f)Eigen::Translation3f(tx, ty, tz);
	  Eigen::Affine3f rot_transform = (Eigen::Affine3f)Eigen::AngleAxisf(rx,Eigen::Vector3f::UnitX());
	  pcl::transformPointCloud<pcl::PointXYZ> (*pc, *pc_moved, rot_transform*transl_transform);   
	  viewer.addPointCloud(pc_moved, "pc_moved");
	  
	  viewer.spinOnce(100);
	  viewer.removePointCloud("pc_moved");

	  rx += 0.01;
     }

}
