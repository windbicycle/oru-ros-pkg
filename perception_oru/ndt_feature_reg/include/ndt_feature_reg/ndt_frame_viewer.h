#ifndef NDTFRAMEVIEWER_HH
#define NDTFRAMEVIEWER_HH

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_feature_reg/ndt_frame_proc.h>
#include "pcl/visualization/pcl_visualizer.h"


namespace ndt_feature_reg
{
         template <typename PointT>
	      inline void viewKeypointMatches(NDTFrameProc<PointT> *proc, int delay)
     {
	  if (proc->frames.size() < 2)
	       return;
	  cv::Mat display;
	  int i = proc->frames.size()-1;
	  cv::drawMatches(proc->frames[i-1]->img, proc->frames[i-1]->kpts, proc->frames[i]->img, proc->frames[i]->kpts, proc->pe.inliers, display);
	  const std::string window_name = "matches";
	  cv::namedWindow(window_name,0);
	  cv::imshow(window_name, display);
	  cv::waitKey(delay);
     }

    template <typename PointT>
     class NDTFrameViewer
     {
     public:
	  NDTFrameViewer(NDTFrameProc<PointT> *proc);
	  //NDTFrameViewer(NDTFrame<PointT> *f0, NDTFrame<PointT> *f1, NDTFrameProc<PointT> &proc);
	  void showPC();
	  void showFeaturePC();
	  void showNDT();
	  void showMatches(const std::vector<cv::DMatch> &matches);
	  void showMatches(const std::vector<std::pair<int,int> > &matches);
	  boost::shared_ptr<pcl::visualization::PCLVisualizer>& getViewerPtr() { return _viewer; }
	  bool wasStopped();
	  void spinOnce();
     private:
	  void initViewer();	
	  //std::vector<NDTFrame<PointT>*> _frames;
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
	  NDTFrameProc<PointT> *_proc;
     
	  inline pcl::PointXYZRGB getPCLColor(int r, int g, int b)
	  {
	      pcl::PointXYZRGB ret;
	      ret.r = r;
	      ret.g = g;
	      ret.b = b;
	      return ret;
	  }

	  inline pcl::PointXYZRGB getPCLColor(size_t i)
	  {
	      pcl::PointXYZRGB ret;

	      switch (i)
	      {
		  case 0:
		      ret.r = 255; ret.g = 0; ret.b = 0;
		      return ret;
		  case 1:
		      ret.r = 0; ret.g = 255; ret.b = 0;
		      return ret;
		  case 2:
		      ret.r = 0; ret.g = 0; ret.b = 255;
		      return ret;
		  default:
		      ret.r = 0; ret.g = 0; ret.b = 0;
	      }
	      return ret;
	  }

     };
}
#include <ndt_feature_reg/impl/ndt_frame_viewer.hpp>

#endif
