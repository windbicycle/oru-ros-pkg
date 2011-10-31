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
#ifndef LSL_LAZZY_GRID_HH
#define LSL_LAZZY_GRID_HH

#include <SpatialIndex.hh>
//#include <pcl/kdtree/kdtree_ann.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace lslgeneric {

class NDTCell;
    /** \brief A spatial index represented as a grid map
        \details A grid map with delayed allocation of cells.
    */
class LazzyGrid : public SpatialIndex {
    public:
	LazzyGrid(double cellSize);
	LazzyGrid(LazzyGrid *prot);
	LazzyGrid(double sizeXmeters, double sizeYmeters, double sizeZmeters,
		  double cellSizeX, double cellSizeY, double cellSizeZ,
		  double _centerX, double _centerY, double _centerZ, 
		  Cell *cellPrototype ); 
	virtual ~LazzyGrid();

	virtual Cell* getCellForPoint(pcl::PointXYZ point);
	virtual void addPoint(pcl::PointXYZ point);

	//these two don't make much sense...
	virtual std::vector<Cell*>::iterator begin();
	virtual std::vector<Cell*>::iterator end();
	virtual int size();

	///clone - create an empty object with same type
	virtual SpatialIndex* clone();
	///copy - create the same object as a new instance
	virtual SpatialIndex* copy();

	///method to return all cells within a certain radius from a point
	virtual void getNeighbors(pcl::PointXYZ point, const double &radius, std::vector<Cell*> &cells);

	///sets the cell factory type
	virtual void setCellType(Cell *type);

	virtual void setCenter(const double &cx, const double &cy, const double &cz); 
	virtual void setSize(const double &sx, const double &sy, const double &sz); 

	virtual NDTCell* getClosestNDTCell(const pcl::PointXYZ pt);
	virtual std::vector<NDTCell*> getClosestNDTCells(const pcl::PointXYZ pt, double radius);
	virtual Cell* getCellAt(int indX, int indY, int indZ);
	virtual bool getLinkedAt(int indX, int indY, int indZ);

	void getCellSize(float &cx, float &cy, float &cz);
	void getGridSize(int &cx, int &cy, int &cz);
	void getCenter(float &cx, float &cy, float &cz);
	void getIndexForPoint(const pcl::PointXYZ& pt, int &idx, int &idy, int &idz);

	void initKDTree();
	void initialize();
	void writeLinksVRML(FILE* fout);

    private:
	bool initialized;
	Cell ****dataArray;
	bool ***linkedCells;
	Cell *protoType;
	std::vector<Cell*> activeCells;
	pcl::KdTreeFLANN<pcl::PointXYZ> meansTree;
	bool centerIsSet, sizeIsSet;
	pcl::KdTree<pcl::PointXYZ>::PointCloudPtr mp;

	double sizeXmeters, sizeYmeters, sizeZmeters;
	double cellSizeX, cellSizeY, cellSizeZ;
	double centerX, centerY, centerZ;
	int sizeX,sizeY,sizeZ;

	bool checkCellforNDT(int indX, int indY, int indZ);
	//void getIndexArrayForPoint(const pcl::PointXYZ& pt, int *&idx, int *&idy, int *&idz);
};


}; //end namespace

#endif
