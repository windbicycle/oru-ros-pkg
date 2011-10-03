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
#ifndef NDT_WAVEFRONT_PLANNER_HH
#define NDT_WAVEFRONT_PLANNER_HH

#include <cstdio>
#include <Eigen/Core>
#include <Pose.hh>
//#include <vector>
#include <pcl/point_types.h>

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#include<Eigen/StdVector>
#endif

namespace lslgeneric {

class NDTMap;
class NDTCell;
class RobotKinematics;

/** \brief implements an extension of the wavefront planner using ndt
    \details details on the logic can be found at 
    \todo move all thresholds/parameters to a config
*/

class NDTWavefrontPlanner {
    public:
	NDTWavefrontPlanner();
	NDTWavefrontPlanner(NDTMap* _myMap, RobotKinematics * _myKin);

	NDTMap *getMyMap() const;
	void setMyMap(NDTMap *myMap);
	
	RobotKinematics* getMyKinematics() const;
	void setMyKinematics(RobotKinematics *mk);

	void planPath(const Pose3 start, const Pose3 goal);

	///output the map and planned path
	void writeToVRML(const char* filename);
	virtual void writeToVRML(FILE* fout);

	///use this to set the parameters for the NDTCell. \note be careful, remember that the parameters are static, thus global
	static void setParameters(double _MAX_COST = 10000);	    
	std::vector<NDTCell*> computeAccessibleNeighbors(NDTCell *cell);
	bool isCollision(NDTCell *cell);
	NDTCell* findClosestProper(NDTCell *cell, double radius);
	
    protected:
	NDTMap *myMap;
	RobotKinematics *myKin;

	void followGradient();
	
	std::vector<lslgeneric::Pose3,Eigen::aligned_allocator<Pose3> > path;
	//std::vector<Pose3> path;
	std::vector<NDTCell*> neighbors;
	NDTCell *goalCell, *startCell;
	bool isValid;
	bool transitPossible(NDTCell* from, NDTCell* to);

	//static parameters	
	static bool parametersSet;
	static double MAX_COST;// = 10000;
};

} //end namespace

#endif

