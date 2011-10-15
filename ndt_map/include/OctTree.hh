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
#ifndef OCT_TREE_HH
#define OCT_TREE_HH

#include <SpatialIndex.hh>
#include <Cell.hh>
#include <vector>
#include <cstdio>

#include <Eigen/Core>

namespace lslgeneric {

    /** \brief Base cell class for \ref OctTree based indeces.
      * \details The basic OctCell is a wrapper around an std::vector 
      * point container. 
      */
    class OctCell : public Cell {
	public:
	    OctCell();
	    virtual ~OctCell() { points.clear(); }
	    OctCell(pcl::PointXYZ  _center, double &xsize, double &ysize, double &zsize);
	    OctCell(const OctCell& other);
	    std::vector<pcl::PointXYZ > points;
	    
	    virtual void writeToVRML(FILE *fout, Eigen::Vector3f color);
	    virtual Cell* clone();
	    virtual Cell* copy();
	    virtual void addPoint(pcl::PointXYZ pt) {
		points.push_back(pt);
	    }
	public:
	      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
    class NDTCell;

    /** \brief An Oct Tree data structure for storing 3D points
      * \details This is an implementation of a \ref SpatialIndex that splits space 
      * using a tree structure. Each node has 8 children that do not necessarily have
      * associated points. Points are stored using \ref OctCell cells. When inserting 
      * points the tree is split until the size BIG_CELL is reached. Further splits occur
      * when the number of points in a leaf goes above the MAX_POINTS threshold
      */
    class OctTree : public SpatialIndex {
	protected:
	    OctTree* parent;
	    OctTree *children[8];
	    OctCell *myCell; 
	    
	    std::vector<Cell*> myLeafs;
	    unsigned int depth;
	    bool leaf;
	    bool leafsCached;

	    /// @param maximum depth of the tree, after which no more splits
	    static int MAX_DEPTH;
	    static bool parametersSet;
	    

	    ///checks in which child node a point would belong
	    virtual size_t getIndexForPoint(const pcl::PointXYZ pt);
	    ///fills in the leafs vector when needed
	    virtual void computeLeafCells();


	public:
	    //--- OctTree Parameters ---//
	    /// @param number of points after which to split cell
	    static int MAX_POINTS;
	    /// @param at this level do not split any more
	    static double SMALL_CELL_SIZE;
	    /// @param split tree up to this size before allocating a cell
	    static double BIG_CELL_SIZE;
	    
	    ///dummy default constructor
	    OctTree();
	    ///creates an oct tree node with known center and size
	    OctTree(pcl::PointXYZ center, double xsize, double ysize, 
		    double zsize, OctCell* type, OctTree *_parent=NULL, unsigned int _depth=0);
	    virtual ~OctTree();

	    ///use this to set the parameters for the OctTree. If not called before creating the
	    ///first leaf, default parameters will be used. \note be careful, remember that the parameters are static, thus global
	    static void setParameters(double _BIG_CELL_SIZE	=4,
				      double _SMALL_CELL_SIZE   =0.5 ,	
			              int _MAX_POINTS		=10,
				      int _MAX_DEPTH		=20
				     );
	    
	    ///add a point to the index
	    virtual void addPoint(pcl::PointXYZ point);
	    ///returns a pointer to the cell containing the point or NULL if not found
	    virtual Cell* getCellForPoint(pcl::PointXYZ point);
	    virtual Cell* getMyCell();
	    virtual OctTree* getLeafForPoint(pcl::PointXYZ point);
	
	    ///sets the prototype for a cell
	    virtual void setCellType(Cell *type);

	    ///iterator through all cells in index, points at the begining
	    virtual std::vector<Cell*>::iterator begin();
	    ///iterator through all cells in index, points at the end
	    virtual std::vector<Cell*>::iterator end();

	    ///recursively print the tree
	    void print();
	    
	    ///output methods to a vrml file. draws the cell sizes and all points
	    void writeToVRML(const char* filename);
	    void writeToVRML(FILE* fout);

	    ///returns a child at the specified index
	    inline OctTree* getChild(int idx) { 
		if(idx<8 && idx>=0) {
		    return children[idx];
		} 
		return NULL;
	    }

	    inline bool isLeaf() {
		return leaf;
	    }
	    
	    /// cloning method inherited from spatial index
	    virtual SpatialIndex* clone();
	    virtual SpatialIndex* copy();

	    virtual void setCenter(const double &cx, const double &cy, const double &cz);
	    virtual void setSize(const double &sx, const double &sy, const double &sz);
	    
	    virtual void getNeighbors(pcl::PointXYZ point, const double &radius, std::vector<Cell*> &cells);
	    virtual bool intersectSphere(pcl::PointXYZ center, const double &radius) const;

	    virtual Cell* getClosestLeafCell(pcl::PointXYZ pt);
	    virtual NDTCell* getClosestNDTCell(const pcl::PointXYZ pt);

	    friend class AdaptiveOctTree;
	    friend class EllipsoidTree;
	    friend class MultilayeredOcGrid;
	public:
	      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} //end namespace

#endif
