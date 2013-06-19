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
#ifndef LSL_LAZZY_GRID_HH
#define LSL_LAZZY_GRID_HH

#include <spatial_index.h>
#include <ndt_cell.h>
//#include <pcl/kdtree/kdtree_ann.h>
//#include <pcl/kdtree/kdtree_flann.h>

namespace lslgeneric
{

/** \brief A spatial index represented as a grid map
    \details A grid map with delayed allocation of cells.
*/
template <typename PointT>
class LazyGrid : public SpatialIndex<PointT>
{
public:
    LazyGrid(double cellSize);
    LazyGrid(LazyGrid *prot);
    LazyGrid(double sizeXmeters, double sizeYmeters, double sizeZmeters,
             double cellSizeX, double cellSizeY, double cellSizeZ,
             double _centerX, double _centerY, double _centerZ,
             Cell<PointT> *cellPrototype );
    virtual ~LazyGrid();

    virtual Cell<PointT>* getCellForPoint(const PointT &point);
    virtual Cell<PointT>* addPoint(const PointT &point);

    //these two don't make much sense...
    ///iterator through all cells in index, points at the begining
    virtual typename SpatialIndex<PointT>::CellVectorItr begin();
    ///iterator through all cells in index, points at the end
    virtual typename SpatialIndex<PointT>::CellVectorItr end();
    virtual int size();

    ///clone - create an empty object with same type
    virtual SpatialIndex<PointT>* clone() const;
    virtual SpatialIndex<PointT>* copy() const;

    ///method to return all cells within a certain radius from a point
    virtual void getNeighbors(const PointT &point, const double &radius, std::vector<Cell<PointT>*> &cells);

    ///sets the cell factory type
    virtual void setCellType(Cell<PointT> *type);

    virtual void setCenter(const double &cx, const double &cy, const double &cz);
    virtual void setSize(const double &sx, const double &sy, const double &sz);

    virtual NDTCell<PointT>* getClosestNDTCell(const PointT &pt, bool checkForGaussian=true);
    virtual std::vector<NDTCell<PointT>*> getClosestNDTCells(const PointT &pt, int &n_neigh, bool checkForGaussian=true);
    virtual std::vector<NDTCell<PointT>*> getClosestCells(const PointT &pt);

    virtual inline void getCellAt(int indX, int indY, int indZ, Cell<PointT>* &cell){
	if(indX < sizeX && indY < sizeY && indZ < sizeZ && indX >=0 && indY >=0 && indZ >=0){
	    cell = dataArray[indX][indY][indZ];
	}else{
		cell = NULL;
		
	}
    }
    virtual inline void getCellAt(const PointT& pt, Cell<PointT>* &cell){
	int indX,indY,indZ;
	this->getIndexForPoint(pt,indX,indY,indZ);
	this->getCellAt(indX,indY,indZ,cell);
    }
    virtual inline void getNDTCellAt(int indX, int indY, int indZ, NDTCell<PointT>* &cell){
			if(indX < sizeX && indY < sizeY && indZ < sizeZ && indX >=0 && indY >=0 && indZ >=0){
					cell = dynamic_cast<NDTCell<PointT>*> (dataArray[indX][indY][indZ]);
			}else{
				cell = NULL;
			}
    }
    virtual inline void getNDTCellAt(const PointT& pt, NDTCell<PointT>* &cell){
			int indX,indY,indZ;
			this->getIndexForPoint(pt,indX,indY,indZ);
			this->getNDTCellAt(indX,indY,indZ,cell);
    }
    //virtual bool getLinkedAt(int indX, int indY, int indZ);

    void getCellSize(double &cx, double &cy, double &cz);
    void getGridSize(int &cx, int &cy, int &cz);
    void getGridSizeInMeters(double &cx, double &cy, double &cz);
    void getCenter(double &cx, double &cy, double &cz);
    virtual inline void getIndexForPoint(const PointT& pt, int &idx, int &idy, int &idz);
    Cell<PointT> * getProtoType()
    {
        return protoType;
    }

    //void initKDTree();
    virtual void initialize();
    virtual void initializeAll() ;

    Cell<PointT> ****getDataArrayPtr()
    {
        return dataArray;
    }

    ///reads map contents from .jff file
    virtual int loadFromJFF(FILE * jffin);
    inline bool traceLine(const Eigen::Vector3d &origin, const PointT &endpoint, const Eigen::Vector3d &diff, const double& maxz, std::vector<NDTCell<PointT>*> &cells);
    inline bool traceLineWithEndpoint(const Eigen::Vector3d &origin, const PointT &endpoint, const Eigen::Vector3d &diff, const double& maxz, std::vector<NDTCell<PointT>*> &cells, Eigen::Vector3d &final_point);
    bool isInside(const PointT& pt) {
			int indX,indY,indZ;
			this->getIndexForPoint(pt,indX,indY,indZ);
			return(indX < sizeX && indY < sizeY && indZ < sizeZ && indX >=0 && indY >=0 && indZ >=0);
    }
protected:
    bool initialized;
    Cell<PointT> ****dataArray;
    //bool ***linkedCells;
    Cell<PointT> *protoType;
    std::vector<Cell<PointT>*> activeCells;
    //pcl::KdTreeFLANN<PointT> meansTree;
    bool centerIsSet, sizeIsSet;
    //typename pcl::KdTree<PointT>::PointCloudPtr mp;

    double sizeXmeters, sizeYmeters, sizeZmeters;
    double cellSizeX, cellSizeY, cellSizeZ;
    double centerX, centerY, centerZ;
    int sizeX,sizeY,sizeZ;

    virtual bool checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian=true);
    //void getIndexArrayForPoint(const PointT& pt, int *&idx, int *&idy, int *&idz);
};


}; //end namespace
#include<impl/lazy_grid.hpp>

#endif
