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
#ifndef NDT_CELL_HH
#define NDT_CELL_HH

#include <OctTree.hh>
#include <Eigen/Core>

namespace lslgeneric {
    /** \brief implements a normal distibution cell
        \details The base class for all NDT indeces, contains
	mean, covariance matrix as well as eigen decomposition of covariance
    */
    class NDTCell : public OctCell {
	public:
	    bool hasGaussian;
	    enum CellClass {HORIZONTAL=0, VERTICAL, INCLINED, ROUGH, UNKNOWN};
	    NDTCell();
	    virtual ~NDTCell();
	    
	    virtual Cell* clone();
	    virtual Cell* copy();

	    void computeGaussian();
	    void updateObservation();

	    void classify();
	    CellClass getClass() const;
	    
	    void writeToVRML(FILE *fout, Eigen::Vector3d col = Eigen::Vector3d(0,0,0));

	    Eigen::Matrix3d getCov();
	    Eigen::Matrix3d getInverseCov();
	    Eigen::Vector3d getMean();
	    void setCov(Eigen::Matrix3d);
	    void setMean(Eigen::Vector3d);
	    Eigen::Matrix3d getEvecs() const;	    
	    Eigen::Vector3d getEvals() const;	    
	    void setEvals(Eigen::Vector3d &ev) { 
		evals = ev;
	    }		
	    
	    ///use this to set the parameters for the NDTCell. \note be careful, remember that the parameters are static, thus global
	    static void setParameters(double _EVAL_ROUGH_THR   =0.1, 
				      double _EVEC_INCLINED_THR=8*M_PI/18, 
				      double _EVAL_FACTOR      =100 
				     );
	    double getLikelihood(pcl::PointXYZ pt);

	private:
	    Eigen::Matrix3d cov;
	    Eigen::Matrix3d icov;
	    Eigen::Matrix3d evecs;
	    Eigen::Vector3d mean;
	    Eigen::Vector3d evals;
	    CellClass cl;
	    static bool parametersSet;
	    static double EVAL_ROUGH_THR;// = 0.1;
	    static double EVEC_INCLINED_THR; // = cos(8*M_PI/18);//10 degree slope;
	    static double EVAL_FACTOR;
	    double d1,d2;

	///tsv: for now i will keep the planner-specific stuff here
	///     if it gets too much i'll move it
	public:
	    double cost;
	public:
	      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
double geomDist(pcl::PointXYZ p1, pcl::PointXYZ p2);
};

#endif
