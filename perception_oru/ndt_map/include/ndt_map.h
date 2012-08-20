/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef NDT_MAP_HH
#define NDT_MAP_HH

#include <spatial_index.h>
#include <ndt_cell.h>
#include <depth_camera.h>

#include <cstdlib>

#include <cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace lslgeneric {

    /** \brief Implements an NDT based spatial index
        \details This is an interface to a SpatialIndex (custom defined)
	that contains NDT cells. Provides methods to create from a PointCloud
    */
    template <typename PointT>
    class NDTMap {
	public:
	    NDTMap() {
		index_ = NULL;
		//std::cout<<"NULL INDEX\n";
	    }
	    /** default constructor. The SpatialIndex sent as a paramter 
	      is used as a factory every time that loadPointCloud is called. 
	      it can/should be deallocated outside the class after the destruction of the NDTMap
	     */
	    NDTMap(SpatialIndex<PointT> *idx) 
	    {
		//std::cout<<"STORE INDEX PROTOTYPE\n";
		index_ = idx;
		//this is used to prevent memory de-allocation of the *si
		//si was allocated outside the NDT class and should be deallocated outside
		isFirstLoad_=true;
	    }
	    NDTMap(const NDTMap& other)
	    {
		//std::cout<<"COPY MAP\n";
		if(other.index_ != NULL) {
		    //std::cout<<"COPY INDEX\n";
		    this->index_ = index_->copy();
		    isFirstLoad_ = false;
		}	
	    }
	    virtual ~NDTMap()
	    {
		//std::cout<<"DELETE MAP\n";
		if(index_ !=NULL && !isFirstLoad_) {
		    //std::cout<<"DELETE INDEX\n";
		    delete index_;
		}
	    }
		void addPointCloud(const pcl::PointCloud<PointT> &pc);
			
	    void loadPointCloud(const pcl::PointCloud<PointT> &pc);
	    /// each entry in the indices vector contains a set of indices to a NDC cell.
	    void loadPointCloud(const pcl::PointCloud<PointT> &pc, const std::vector<std::vector<size_t> > &indices);
	    
	    void loadDepthImage(const cv::Mat& depthImage, DepthCamera<PointT> &cameraParams);
	    pcl::PointCloud<PointT> loadDepthImageFeatures(const cv::Mat& depthImage, std::vector<cv::KeyPoint> &keypoints, 
				size_t &supportSize, double maxVar, DepthCamera<PointT> &cameraParams, bool estimateParamsDI=false, bool nonMean = false);
	    void computeNDTCells();
	    	    
	    void writeToVRML(const char* filename);
	    virtual void writeToVRML(FILE* fout);
	    virtual void writeToVRML(FILE* fout, Eigen::Vector3d col);

	    int writeToJFF(const char* filename);
	    int writeLazyGridJFF(FILE * jffout);
	    int writeCellVectorJFF(FILE * jffout);
	    int writeOctTreeJFF(FILE * jffout);

	    int loadFromJFF(const char* filename);

	    inline SpatialIndex<PointT>* getMyIndex() const 
	    {
		return index_;
	    }
	    /// return the spatial index used as a string
	    std::string getMyIndexStr() const;
	    /// return the spatial index used as an integer
	    int getMyIndexInt() const;
	    
	    //computes the likelihood of a single observation
    	    double getLikelihoodForPoint(PointT pt);
	    
	    //computes the likelihood of a single observation
    	    //double getLikelihoodForPointWithInterpolation(PointT pt);
	    
	    //returns the covariance matrix of the closest cell to refPoint
	    bool getCellForPoint(const PointT &refPoint, NDTCell<PointT> *&cell);
	    std::vector<NDTCell<PointT>*> getCellsForPoint(const PointT pt, double radius);

	    ///return the cell using a specific index (not available for all spatialindexes), will return NULL if the idx is not valid.
	    NDTCell<PointT>* getCellIdx(unsigned int idx);

	    std::vector<NDTCell<PointT>*> pseudoTransformNDT(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T);
	    /**
	      * Returns all computed cells from the map
	      */
	    //std::vector<lslgeneric::NDTCell<pcl::PointXYZ>*> getAllCells();
			std::vector<lslgeneric::NDTCell<PointT>*> getAllCells();

	    int numberOfActiveCells();

	    //tsv: temporary debug function
	    void debugToVRML(const char* fname, pcl::PointCloud<PointT> &pc);
	protected:
	    SpatialIndex<PointT> *index_;
	    bool isFirstLoad_;

	public:
	      EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    };

} // end namespace

#include <impl/ndt_map.hpp>

#endif
