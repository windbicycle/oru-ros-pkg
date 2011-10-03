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
#ifndef LSLPOSE_HH
#define LSLPOSE_HH

#include <Eigen/Eigen>

namespace lslgeneric {

/** \brief This class provides storage for an oriented point in 3D
 *  \details
 *  Internally, the location of the point is stored as a vector, 
 *  while the orientation is stored as a quaternion
 */  
class Pose3 
{
public:
     Eigen::Vector3d pos;
     Eigen::Quaternion<double> rot;
     
     ///default empty constructor
     Pose3();
     ///copy constructor
     Pose3(const Pose3& other);
     ///parametrized constructor from vector and quaternion orientation
     Pose3(Eigen::Vector3d &_pos, Eigen::Quaternion<double> &_ori);	
     ///parametrized from 3 positions and yaw orientation
     Pose3(double x, double y, double z, double yaw);

     static Pose3 from2d(double x, double y, double yaw);
     static Pose3 from2d(const Eigen::Vector3d &vec);

     virtual double x() const { return pos(0); }
     virtual double y() const { return pos(1); }
     virtual double z() const { return pos(2); }
     
     virtual void setX(double v) { pos(0) = v; }
     virtual void setY(double v) { pos(1) = v; }
     virtual void setZ(double v) { pos(2) = v; }
     
     const Eigen::Vector3d& getPos() const { return pos; }
     Eigen::Vector3d& getPos() { return pos; }
     
     const Eigen::Quaternion<double>& getRot() const { return rot; }
     Eigen::Quaternion<double>& getRot() { return rot; }
     
     Pose3 add(const Pose3& other);
     //Pose3 sub(const Pose3& other);
     
     void writeToVRML(const char* filename);
     virtual void writeToVRML(FILE* fout);
    
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}; //end of namespace

#endif
