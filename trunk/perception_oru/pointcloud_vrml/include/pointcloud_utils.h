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
#ifndef LSL_POINT_CLOUD_UTILS
#define LSL_POINT_CLOUD_UTILS

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace lslgeneric
{
/* \brief Routines to read/write point clouds from VRML files */

template< typename PointT>
pcl::PointCloud<PointT> readVRML(const char* fname);
template< typename PointT>
pcl::PointCloud<PointT> readVRML(FILE* fout);

template< typename PointT>
pcl::PointCloud<PointT> readVRMLIntensity(const char* fname);
template< typename PointT>
pcl::PointCloud<PointT> readVRMLIntensity(FILE* fout);

template< typename PointT>
void writeToVRMLColor(const char* fname, pcl::PointCloud<PointT> &pc);
template< typename PointT>
void writeToVRMLColor(FILE* fout, pcl::PointCloud<PointT> &pc);

template< typename PointT>
void writeToVRMLIntensity(const char* fname, pcl::PointCloud<PointT> &pc,
                          Eigen::Vector3d col = Eigen::Vector3d(1,1,1));
template< typename PointT>
void writeToVRMLIntensity(FILE* fout, pcl::PointCloud<PointT> &pc,
                          Eigen::Vector3d col = Eigen::Vector3d(1,1,1));

template< typename PointT>
void writeToVRML(const char* fname, pcl::PointCloud<PointT> &pc,
                 Eigen::Vector3d col = Eigen::Vector3d(1,1,1));
template< typename PointT>
void writeToVRML(FILE* fout, pcl::PointCloud<PointT> &pc,
                 Eigen::Vector3d col = Eigen::Vector3d(1,1,1));

template< typename PointT>
pcl::PointCloud<PointT> transformPointCloud(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T,
        const pcl::PointCloud<PointT> &pc);
template< typename PointT>
void transformPointCloudInPlace(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T, pcl::PointCloud<PointT> &pc);

template< typename PointT>
double geomDist(PointT p1, PointT p2);

};
#include<impl/pointcloud_utils.hpp>

#endif

