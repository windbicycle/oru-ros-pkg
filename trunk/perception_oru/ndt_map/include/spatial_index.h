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
#ifndef SPATIAL_INDEX_HH
#define SPATIAL_INDEX_HH

#include <vector>
#include <cell.h>
#include <iostream>

namespace lslgeneric
{

/** \brief Base class for all spatial indexing structures
    \details
A SpatialIndex is anything that holds PointInterface pointers
and organizes them in a manner accessible from outside.
This class defines what is necessary to be a spatial index - namely
the ability to find the cell in which a point is placed and to store
newly observed points. It should also be possible to check the size of
the occupied space, as well as to get cells neighboring any given cell.
*/
template <typename PointT>
class SpatialIndex
{
protected:

public:
    typedef std::vector<Cell<PointT>*> CellPtrVector;
    typedef typename CellPtrVector::iterator CellVectorItr;

    virtual ~SpatialIndex()
    {
    }

    virtual Cell<PointT>* getCellForPoint(const PointT &point) = 0;
    virtual void addPoint(const PointT &point) = 0;

    ///iterator through all cells in index, points at the begining
    virtual CellVectorItr begin() = 0;
    ///iterator through all cells in index, points at the end
    virtual CellVectorItr end() = 0;
    // should be 'pure'?
    virtual int size() const
    {
        return -1;
    }

    ///clone - create an empty object with same type
    virtual SpatialIndex<PointT>* clone() const = 0;
    ///copy - create the same object as a new instance
    virtual SpatialIndex<PointT>* copy() const = 0;

    ///the following methods provide index specific functionalities and
    ///don't have to be implemented by all sub-classes
    virtual void setCenter(const double &cx, const double &cy, const double &cz) {}
    virtual void setSize(const double &sx, const double &sy, const double &sz) {}

    ///method to return all cells within a certain radius from a point
    virtual void getNeighbors(const PointT &point, const double &radius, std::vector<Cell<PointT>*> &cells)= 0;

    ///sets the cell factory type
    virtual void setCellType(Cell<PointT> *type) = 0;

    ///reads map contents from .jff file
    virtual int loadFromJFF(FILE * jffin) const
    {
        std::cerr << "Calling from SpatialIndex.h\n";
        return -1;
    }
};

} //end namespace


#endif
