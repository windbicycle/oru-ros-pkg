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
#ifndef ELLIPSOID_TREE_HH
#define ELLIPSOID_TREE_HH

#include <OctTree.hh>
#include <vector>
#include <NDTCell.hh>

namespace lslgeneric {

    /** \brief Implements an OctTree with Ellipsoid conditions used for splitting 
        \details 
     */
    class EllipsoidTree : public OctTree {
	protected:
	    std::vector<OctTree*> splitTree(OctTree *leaf);
	    std::vector<OctTree*> myTreeLeafs;
	    virtual void computeTreeLeafs();
	    
	    static double FLAT_FACTOR;
	    static bool parametersSet;

	public:
	    static double MIN_CELL_SIZE;
	    
	    ///dummy default constructor
	    EllipsoidTree();
	    ///creates an oct tree node with known center and size
	    EllipsoidTree(pcl::PointXYZ center, double xsize, double ysize, 
		    double zsize, OctCell* type, OctTree *_parent=NULL, unsigned int _depth=0);
	    virtual ~EllipsoidTree();

	    //addPoint now just saves the points in a vector (NDTCell)
	    virtual void addPoint(pcl::PointXYZ point);

	    //computes the actual tree
	    virtual void postProcessPoints();

	    virtual SpatialIndex* clone();

	    ///use this to set the parameters for the OctTree. If not called before creating the
	    ///first leaf, default parameters will be used. \note be careful, remember that the parameters are static, thus global
	    static void setParameters(double _MIN_CELL_SIZE = 0.5,
		    double _FLAT_FACTOR = 10
		    ); 

    };
} //end namespace
#endif
