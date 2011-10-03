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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <SpatialIndex.hh>
#include <cstdlib>
#include <NDTCell.hh>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lslgeneric {

    /** \brief Implements an NDT based spatial index
        \details This is an interface to a SpatialIndex (custom defined)
	that contains NDT cells. Provides methods to create from a PointCloud
    */
    class NDTMap {
	public:
	    NDTMap(SpatialIndex *idx);
	    NDTMap(const NDTMap& other);
	    virtual ~NDTMap();

	    void loadPointCloud(pcl::PointCloud<pcl::PointXYZ> pc);
	    void computeNDTCells();
	    
	    void writeToVRML(const char* filename);
	    virtual void writeToVRML(FILE* fout, bool bOctMap = false);
	    virtual void writeToVRML(FILE* fout, Eigen::Vector3d col);

	    SpatialIndex* getMyIndex() const;
	    
	    ///use this to set the parameters for the NDTMap. \note be careful, remember that the parameters are static, thus global
	    static void setParameters();	    

	    //computes the likelihood of a single observation
    	    double getLikelihoodForPoint(pcl::PointXYZ pt);
	    
	    //computes the likelihood of a single observation
    	    double getLikelihoodForPointWithInterpolation(pcl::PointXYZ pt);
	    
	    //returns the covariance matrix of the closest cell to refPoint
	    bool getCellForPoint(const pcl::PointXYZ &refPoint, NDTCell *&cell);
	    std::vector<NDTCell*> getCellsForPoint(const pcl::PointXYZ pt, double radius);

	    std::vector<NDTCell*> pseudoTransformNDT(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T);
	    //tsv: temporary debug function
	    void debugToVRML(const char* fname, pcl::PointCloud<pcl::PointXYZ> &pc);
	protected:
	    SpatialIndex *index;
	    bool isFirstLoad;

	    static bool parametersSet;
	public:
	      EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    };

} // end namespace

#endif
