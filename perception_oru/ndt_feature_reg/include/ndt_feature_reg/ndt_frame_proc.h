#ifndef NDTFRAMEPROC_HH
#define NDTFRAMEPROC_HH

#include <ndt_feature_reg/ndt_frame.h>
#include <ndt_matcher_d2d_feature.h>

namespace ndt_feature_reg
{
    template <typename PointT>
     class NDTFrameProc
     {
     public:
//	  bool loadImg(NDTFrame<PointT> &f, const std::string &fileName) const;
//	  void loadPC(NDTFrame<PointT> &f, const std::string &fileName) const;
	  
//	  bool loadPCfromDepth(NDTFrame<PointT> &f, const std::string &fileName) const;
	  
//	  bool setupDepthToPC(const std::string &fileName, double fx, double fy, double cx, double cy, const std::vector<double> &dist, double ds);
//	  void setupDepthToPC(const cv::Mat &depthImg, double fx, double fy, double cx, double cy, const std::vector<double> &dist, double ds);

//	  void convertDepthToPC(const cv::Mat &depthImg, pcl::PointCloud<PointT> &pc) const;

	  void addFrame (NDTFrame<PointT> *f) 
	  {
	      frames.push_back(f);
	  }
	  
	  void addFrameIncremental (NDTFrame<PointT> *f, bool skipMatching, bool ndtEstimateDI = false, 
				    bool match_full = false, bool match_no_association = false);
	  void trimNbFrames (size_t maxNbFrames);

	  void processFrames (bool skipMatching, bool ndtEstimateDI = false, 
				    bool match_full = false, bool match_no_association = false);
	  
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr createColoredFeaturePC(NDTFrame<PointT> &f, pcl::PointXYZRGB color);
	  
	  NDTFrameProc(int nb_ransac, double max_inldist_xy, double max_inldist_z): pe(nb_ransac, max_inldist_xy, max_inldist_z)
	  {
	       detector = cv::FeatureDetector::create("SURF");
	       extractor = cv::DescriptorExtractor::create("SURF");
	       img_scale = 1.;
	       trim_factor = 1.;
	       non_mean = false;
	  }
	  
	  PoseEstimator<PointT,PointT> pe;
	  typedef Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> EigenTransform;
	  typename std::vector< NDTFrame<PointT>*,Eigen::aligned_allocator<NDTFrame<PointT> > > frames;
	  typename std::vector<EigenTransform, Eigen::aligned_allocator<EigenTransform> > transformVector;

	  inline void convertMatches(const std::vector<cv::DMatch> &in, std::vector<std::pair<int,int> > &out)
	  {  
	      out.resize(in.size());
	      for (size_t i = 0; i < in.size(); i++)
	      {
		  out[i].first = in[i].queryIdx;
		  out[i].second = in[i].trainIdx;
	      } 
	  }      

	  inline std::vector<std::pair<int,int> > convertMatches(const std::vector<cv::DMatch> &in)
	  {      
	      std::vector<std::pair<int,int> > out;
	      convertMatches(in,out);
	      return out;
	  } 

	  cv::Ptr<cv::FeatureDetector> detector;
	  cv::Ptr<cv::DescriptorExtractor> extractor;
	  double img_scale;
	  double trim_factor;
	  bool non_mean;
	  virtual ~NDTFrameProc() {
	      for(size_t i =0; i<frames.size(); i++) {
		  delete frames[i];
	      }
	      frames.clear();
	  }
     private:
	  
	  void detectKeypoints(NDTFrame<PointT> *f) const;
	  void calcDescriptors(NDTFrame<PointT> *f) const;
    public:
	          EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//	  lslgeneric::NDTMatcherFeatureD2D<PointT,PointT> *matcher_feat; //(corr);


     };
     
     
     
} // namespace

#include <ndt_feature_reg/impl/ndt_frame_proc.hpp>
#endif
