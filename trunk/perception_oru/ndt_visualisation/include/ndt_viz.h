#ifndef NDT_VIZ_HH
#define NDT_VIZ_HH

#include <mrpt/gui.h>
#include <mrpt/base.h>
#include <mrpt/opengl.h>

#include <ndt_map.h>
#include <CMyEllipsoid.h>

template <typename PointT>
class NDTViz {

    public:
	mrpt::gui::CDisplayWindow3D *win3D;
	NDTViz(bool allocate_new_window=true) 
	{
	    if(allocate_new_window) 
	    {
		win3D = new mrpt::gui::CDisplayWindow3D("NDT Viz",800,600);
	    }
	    else 
	    {
		win3D = NULL;
	    }

	}


	void plotNDTSAccordingToOccupancy(float occupancy, lslgeneric::NDTMap<PointT> *map){
	    if(win3D == NULL) return;
	    std::vector<lslgeneric::NDTCell<PointT>*> global_ndts;
	    global_ndts = map->getAllCells();
	    fprintf(stderr," NUM NDT: %d ", global_ndts.size());

	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();
	    scene->clear();
	    unsigned int accepted_ndts=1;
	    double x = 0,y=0,s=0;
	    for(unsigned int i=0;i<global_ndts.size();i++){
		Eigen::Vector3d m = global_ndts[i]->getMean();
		if(!global_ndts[i]->hasGaussian_) continue;
		x+=m[0];
		y+=m[1];
		s+=1;
		if(global_ndts[i]->getOccupancy()>occupancy){
		    accepted_ndts++;
		    mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		    Eigen::Matrix3d cov = global_ndts[i]->getCov();
		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);

		    objEllip->setColor((m[2]+2.0)/3.0,0,0,0.6);

		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );
		}else if(global_ndts[i]->getOccupancy()<-0){
		    mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		    Eigen::Matrix3d cov = global_ndts[i]->getCov();
		    cov = cov;
		    //Eigen::Vector3d m = global_ndts[i]->getMean();

		    mrpt::math::CMatrixDouble M = cov;
		    objEllip->setCovMatrix(M);
		    objEllip->setLocation(m[0], m[1], m[2]);
		    objEllip->setColor(1.0,0,0,1.0);
		    objEllip->enableDrawSolid3D(true);
		    scene->insert( objEllip );
		}
	    }
	    win3D->setCameraPointingToPoint(x/s,y/s,3.0);
	    win3D->unlockAccess3DScene();
	    win3D->repaint();
	    fprintf(stderr,"(%lf %lf) s=%lf\n",x/s,y/s,s);
	    for(unsigned int i=0;i<global_ndts.size();i++) delete global_ndts[i];

	}

	void plotLocalNDTMap(pcl::PointCloud<PointT> &cloud, double resolution){
	    if(win3D == NULL) return;

	    lslgeneric::NDTMap<PointT> ndlocal(new lslgeneric::LazyGrid<PointT>(resolution));
	    ndlocal.addPointCloudSimple(cloud);
	    ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

	    std::vector<lslgeneric::NDTCell<PointT>*> ndts;
	    ndts = ndlocal.getAllCells();
	    mrpt::opengl::COpenGLScenePtr &scene = win3D->get3DSceneAndLock();

	    for(unsigned int i=0;i<ndts.size();i++){
		Eigen::Vector3d m = ndts[i]->getMean();
		if(m[2]>3.0) continue;

		mrpt::opengl::CMyEllipsoidPtr objEllip = mrpt::opengl::CMyEllipsoid::Create();
		Eigen::Matrix3d cov = ndts[i]->getCov();
		cov = cov;

		mrpt::math::CMatrixDouble M = cov;
		objEllip->setCovMatrix(M);
		objEllip->setLocation(m[0], m[1], m[2]);

		objEllip->setColor(0,1.0,0,0.6);
		objEllip->enableDrawSolid3D(true);
		scene->insert( objEllip );
	    }
	    win3D->unlockAccess3DScene();
	    for(unsigned int i=0;i<ndts.size();i++){
		delete ndts[i];
	    }

	}


};

#endif
