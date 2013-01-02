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
#ifndef NDT_MAP_HH
#define NDT_MAP_HH

#include <spatial_index.h>
#include <ndt_cell.h>
#include <depth_camera.h>

#include <cstdlib>
#include<lazy_grid.h>
#include <cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace lslgeneric
{

/**
*  \brief Implements an NDT based spatial index
*  \author Jari Saarinen (jari.saarinen@aalto.fi) and Todor Stoyanov (todor.stoyanov@oru.se)
*  \version 2.0
*  \details This class contains an interface to a SpatialIndex (custom defined)
* that contains NDT cells. Provides methods to create from a PointCloud.
*
* This class implements two approaches to NDT mapping -  "traditional" NDT approach, as well as
* a novel NDT Occupancy Map approach.
*
* The "traditional" approach uses only the measurement points and a single
* scan in order to construct the NDT map.
*
* The NDT-OM fuses incrementally measurement using Recursive Sample Covariance (RSC)
* approach. It also models the occupancy and free space, it adapts to changes in the cell
* etc.
*
* Having these two versions combined also means that not all features are working with both.
* The known NDT-OM issues are
* - Only Lazy-Grid spatial index is supported
*
* The old interface (e.g. used in registration) loadPointCloud(const pcl::PointCloud<PointT> &pc, double range_limit = -1);
* works as before: it computes an NDT map using only the samples and without tracking occupancy.
*
* Since version 2.0 the ndt_map integrates also incremental update features. These are accessible through two methods:
* 1) void addPointCloudSimple(const pcl::PointCloud<PointT> &pc,double maxz=100.0);
* 2) void addPointCloud(const Eigen::Vector3d &origin, const pcl::PointCloud<PointT> &pc, double classifierTh=0.06, double maxz = 100.0, double sensor_noise = 0.25);
*
* The first one only updates the measurement points and thus is faster, but does not model free space and does not tolerate dynamics
* The second one uses ray tracing and a number of approaches to model the occupancy as well as adapts to the dynamics.
*
* In all cases the procedure to use the ndt_map is the following:
* 1) Add the measurement (by above mentioned load or add methods)
* 2) call computeNDTCells
*
* Afer this the map is updated. There are various methods to access the map elements documented in this header file that you may use.
*
* This class implements now the following papers, which you hopefully cite if you find this useful:
* 	Jari Saarinen, Henrik Andreasson, Todor Stoyanov, and Achim J.
* 	Lilienthal. Long-Term Mapping in Large-Scale Dynamic Environments
* 	Using 3D Normal Distribution Maps. In review for IEEE
* 	International Conference on Robotics and Automation (ICRA 2013), 2013.
*
*  Jari Saarinen, Henrik Andreasson, Todor Stoyanov, and Achim J.
*	 Lilienthal. Long-Term Mapping in Large-Scale Dynamic Environments
*  Using 3D Normal Distribution Maps.  In review for IEEE
*  International Conference on Robotics and Automation (ICRA 2013), 2013.
*
*  There is also an implementation of modeling of the dynamics (edata structure in ndt_cell):
*
*  Jari Saarinen, Henrik Andreasson and Achim J Lilienthal,
*  "Independent Markov chain occupancy grid maps for representation of dynamic environments,"
*  in IROS2012 Conference Proceedings, Vilamoura, Algarve, Portugal: IEEE, 2012, pp. 3489-3495.
*
* In addition, this class provide the basis for NDT registration, which is further discussed in the \ref ndt_registration package. The relevant publications are:
*
*
*/
template <typename PointT>
class NDTMap
{
public:
    NDTMap()
    {
        index_ = NULL;
    }
    /** default constructor. The SpatialIndex sent as a paramter
    	*		is used as a factory every time that loadPointCloud is called.
    	*		it can/should be deallocated outside the class after the destruction of the NDTMap
    */
    NDTMap(SpatialIndex<PointT> *idx)
    {

        index_ = idx;
        //this is used to prevent memory de-allocation of the *si
        //si was allocated outside the NDT class and should be deallocated outside
        isFirstLoad_=true;
        map_sizex = -1.0;
        map_sizey = -1.0;
        map_sizez = -1.0;
        is3D = true;
    }

    NDTMap(const NDTMap& other)
    {
        if(other.index_ != NULL)
        {
            this->index_ = index_->copy();
            isFirstLoad_ = false;
        }
    }

    /**
     * Construct with given centroid and sizes
     * @param cenx, ceny, cenz; (x,y,z) of the center of the map
     * @param sizex, sizey, sizez: The size of the map in each respective direction
     * NOTE: Implementation only for the laze grid
     **/
    NDTMap(SpatialIndex<PointT> *idx, float cenx, float ceny, float cenz, float sizex, float sizey, float sizez)
    {
        if(idx == NULL)
        {
            fprintf(stderr,"Idx == NULL - abort()\n");
            exit(1);
        }
        index_ = idx;

        //this is used to prevent memory de-allocation of the *si
        //si was allocated outside the NDT class and should be deallocated outside
        isFirstLoad_=false;

        NDTCell<PointT> *ptCell = new NDTCell<PointT>();
        index_->setCellType(ptCell);
        delete ptCell;
        index_->setCenter(cenx,ceny,cenz);
        index_->setSize(sizex,sizey,sizez);
        map_sizex = sizex;
        map_sizey = sizey;
        map_sizez = sizez;
        is3D=true;
        LazyGrid<PointT> *lz = dynamic_cast<LazyGrid<PointT>*>(index_);
        if(lz == NULL)
        {
            fprintf(stderr,"Unfortunately This constructor works only with Lazygrid!\n");
            exit(1);
        }
        lz->initializeAll();
    }

    /**
    	* Initilize with known values - normally this is done automatically, but in some cases you want to
    	* influence these - call only once and before calling any other function
    	*/
    void initialize(double cenx, double ceny, double cenz, double sizex, double sizey, double sizez)
    {
        isFirstLoad_=false;

        NDTCell<PointT> *ptCell = new NDTCell<PointT>();
        index_->setCellType(ptCell);
        delete ptCell;
        index_->setCenter(cenx,ceny,cenz);
        index_->setSize(sizex,sizey,sizez);
        map_sizex = sizex;
        map_sizey = sizey;
        map_sizez = sizez;
        is3D=true;
        LazyGrid<PointT> *lz = dynamic_cast<LazyGrid<PointT>*>(index_);
        if(lz == NULL)
        {
            fprintf(stderr,"Unfortunately This constructor works only with Lazygrid!\n");
            exit(1);
        }
        lz->initializeAll();
    }



    /**
    * Default destructor
    	*/
    virtual ~NDTMap()
    {
        //std::cout<<"DELETE MAP\n";
        if(index_ !=NULL && !isFirstLoad_)
        {
            //std::cout<<"DELETE INDEX\n";
            delete index_;
        }
    }

    void setMode(bool is3D_)
    {
        is3D=is3D_;
    }

    /**
    * Set the map size in meters - Must be called before first addPointCloud call if
    * you want to set the size - otherwise it is automatically determined
    */
    void setMapSize(float sx, float sy, float sz)
    {
        map_sizex = sx;
        map_sizey = sy;
        map_sizez = sz;
    }


    /**
    	* Add new pointcloud to map - This is the main interface for NDT-OM!
    	* Performs raytracing, updates conflicts and adds points to cells
    	* computeNDTCells must be called after calling this
    	*
    	* @param &origin is the position of the sensor, from where the scan has been taken from.
    	* @param &pc is the pointcloud to be added
    	* @param classifierTh A treshold to judge if the ray passes through a gaussian
    	* @param maxz threshold for the maximum z-coordinate value for the measurement point_cloud
    	* @param sensor_noise The expected standard deviation of the sensor noise
    	*/
    void addPointCloud(const Eigen::Vector3d &origin, const pcl::PointCloud<PointT> &pc, double classifierTh=0.06, double maxz = 100.0, double sensor_noise = 0.25);

    /**
    * This interface updates only the end points into the map without raytracing
    */
    void addPointCloudSimple(const pcl::PointCloud<PointT> &pc,double maxz=100.0);

    /**
    * Adds one measurement to the map using NDT-OM update step
    * @return true if an inconsistency was detected
    */

    bool addMeasurement(const Eigen::Vector3d &origin,PointT endpoint, double classifierTh, double maxz, double sensor_noise);


    /**
    * Adds a sample mean and covariance to the map
    * @param &ucov The covariance matrix to be added
    * @param &umean The mean of the normal distribution
    * @param numpointsindistribution The number of points used in computation of the sample mean and covariance
    */
    void addDistributionToCell(Eigen::Matrix3d &ucov, Eigen::Vector3d &umean, unsigned int numpointsindistribution);


    /**
    * loadPointCloud - You can call this if you are only interested in dealing with one scan
    * without need for fusing several ones or representing empty space and occupancy
    *
    * Otherwise you should always call addPointCloud (or if you don't want occupancy then addPointCloudSimple)
    *
    * \param pc the PointCloud that is to be loaded
    * \note every subsequent call will destroy the previous map!
    */
    void loadPointCloud(const pcl::PointCloud<PointT> &pc, double range_limit = -1);
    /// each entry in the indices vector contains a set of indices to a NDC cell.



    /// NOTE: These load functions are not supported by occupancy mapping
    /// FIXME: Todor, add documentation
    void loadPointCloud(const pcl::PointCloud<PointT> &pc, const std::vector<std::vector<size_t> > &indices);
    void loadDepthImage(const cv::Mat& depthImage, DepthCamera<PointT> &cameraParams);
    pcl::PointCloud<PointT> loadDepthImageFeatures(const cv::Mat& depthImage, std::vector<cv::KeyPoint> &keypoints,
            size_t &supportSize, double maxVar, DepthCamera<PointT> &cameraParams, bool estimateParamsDI=false, bool nonMean = false);



    /**
    * Computes the NDT-cells after a measurement has been added
    * @param cellupdatemode Defines the update mode (default CELL_UPDATE_MODE_SAMPLE_VARIANCE)
    * @param maxnumpoints Defines the forgetting factor (default 100000) the smaller the value the faster the adaptation
    */
    void computeNDTCells(int cellupdatemode = CELL_UPDATE_MODE_SAMPLE_VARIANCE, unsigned int maxnumpoints = 100000, float occupancy_limit=255, Eigen::Vector3d origin = Eigen::Vector3d(0,0,0), double sensor_noise=0.1);


    /**
    * Stuff for saving things
    */
    void writeToVRML(const char* filename);
    virtual void writeToVRML(FILE* fout);
    virtual void writeToVRML(FILE* fout, Eigen::Vector3d col);

    int writeToJFF(const char* filename);
    int writeLazyGridJFF(FILE * jffout);
    int writeCellVectorJFF(FILE * jffout);
    int writeOctTreeJFF(FILE * jffout);

    int loadFromJFF(const char* filename);

    inline SpatialIndex<PointT>* getMyIndex() const
    {
        return index_;
    }
    /// return the spatial index used as a string
    std::string getMyIndexStr() const;
    /// return the spatial index used as an integer
    int getMyIndexInt() const;

    //computes the likelihood of a single observation
    double getLikelihoodForPoint(PointT pt);

    ///Get the cell for which the point fall into (not the closest cell)
    bool getCellAtPoint(const PointT &refPoint, NDTCell<PointT> *&cell);

    /**
     * returns the closest cell to refPoint
     * Does not work with NDT-OM
     */
    bool getCellForPoint(const PointT &refPoint, NDTCell<PointT> *&cell, bool checkForGaussian=true) const;
    /**
     * Returns all the cells within radius
     * Does not work with NDT-OM
     */
    std::vector<NDTCell<PointT>*> getCellsForPoint(const PointT pt, int n_neighbours, bool checkForGaussian=true) const;
    /**
     * Returns all the cells within radius
     */
    std::vector<NDTCell<PointT>*> getInitializedCellsForPoint(const PointT pt) const;


    ///return the cell using a specific index (not available for all spatialindexes), will return NULL if the idx is not valid.
    NDTCell<PointT>* getCellIdx(unsigned int idx);

    /**
    * Returns a transformed NDT as a vector of NDT cells
    */
    std::vector<NDTCell<PointT>*> pseudoTransformNDT(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T);

    /**
     * Returns a transformed NDT as an NDT map with a CellVector data structure
     */
    NDTMap<PointT>* pseudoTransformNDTMap(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T);
    /**
     * Returns all computed cells from the map
     * This method gives all the vectors that contain a gaussian within a cell (hasGaussian is true).
     */
    std::vector<lslgeneric::NDTCell<PointT>*> getAllCells() const;

    /**
     * Returns all cells that have been initialized (including ones that do not contain gaussian at the moment).
     * This is useful if you want to use the empty cells or dynamic cells
     */
    std::vector<lslgeneric::NDTCell<PointT>*> getAllInitializedCells();


    int numberOfActiveCells();

    bool getCentroid(double &cx, double &cy, double &cz)
    {
        LazyGrid<PointT> *lz = dynamic_cast<LazyGrid<PointT>*>(index_);
        if(lz == NULL) return false;
        lz->getCenter(cx, cy, cz);
        return true;
    }
    bool getGridSizeInMeters(double &cx, double &cy, double &cz)
    {
        LazyGrid<PointT> *lz = dynamic_cast<LazyGrid<PointT>*>(index_);
        if(lz == NULL) return false;
        lz->getGridSizeInMeters(cx, cy, cz);
        return true;
    }

    //tsv: temporary debug function
    void debugToVRML(const char* fname, pcl::PointCloud<PointT> &pc);
protected:
    bool is3D;
    SpatialIndex<PointT> *index_;
    bool isFirstLoad_;
    float map_sizex;
    float map_sizey;
    float map_sizez;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    pcl::PointCloud<PointT> conflictPoints; ///< points that were conflicting during update


};

} // end namespace

#include <impl/ndt_map.hpp>

#endif
