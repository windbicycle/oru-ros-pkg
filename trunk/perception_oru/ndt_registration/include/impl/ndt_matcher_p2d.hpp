#include "Eigen/Eigen"
#include "ndt_cell.h"
#include <fstream>
#include <vector>
#include <lazy_grid.h>
#include <oc_tree.h>

namespace lslgeneric
{

//#define DO_DEBUG_PROC

template <typename PointSource, typename PointTarget>
void NDTMatcherP2D<PointSource,PointTarget>::init(bool useDefaultGridResolutions, std::vector<double> _resolutions)
{

    ///////////
    double lfc1,lfc2,lfd3;
    double integral, outlier_ratio, support_size;
    integral = 0.1;
    outlier_ratio = 0.01;
    //outlier_ratio = 0.5;
    support_size = 4; //???
    lfc1 = (1-outlier_ratio)/integral;
    lfc2 = outlier_ratio/pow(support_size,3);
    lfd3 = -log(lfc2);
    lfd1 = -log( lfc1 + lfc2 ) - lfd3;
    lfd2 = -log((-log( lfc1 * exp( -0.5 ) + lfc2 ) - lfd3 ) / lfd1);
    //d1 = 0.1;
    //d2 = 0.001;
    //cout<<lfd1<<" "<<lfd2<<endl;
    ///////////
    useSimpleDerivatives = false;
    Jest.setZero();
    Jest.block<3,3>(0,0).setIdentity();
    Hest.setZero();
    NUMBER_OF_ACTIVE_CELLS = 0;

    if(useDefaultGridResolutions)
    {
        resolutions.push_back(0.2);
        resolutions.push_back(0.5);
        resolutions.push_back(1);
        resolutions.push_back(2);
    }
    else
    {
        resolutions = _resolutions;
    }
}

template <typename PointSource, typename PointTarget>
void NDTMatcherP2D<PointSource,PointTarget>::generateScoreDebug(const char* out, pcl::PointCloud<PointTarget>& fixed,
        pcl::PointCloud<PointSource>& moving)
{

    std::ofstream lg(out,std::ios_base::out);
    int N_LINEAR = 100;
    int N_ROT	 = 100;

    std::cout<<"generating scores...\n";
    for(int q = resolutions.size()-1; q>=0; q--)
    {
        current_resolution = resolutions[q];
        std::cout<<"res "<<current_resolution<<std::endl;
        double lfc1,lfc2,lfd3;
        double integral, outlier_ratio, support_size;
        integral = 0.1;
        outlier_ratio = 0.35;
        support_size = current_resolution;
        lfc1 = (1-outlier_ratio)/integral;
        lfc2 = outlier_ratio/pow(support_size,3);
        lfd3 = -log(lfc2);
        lfd1 = -(-log( lfc1 + lfc2 ) - lfd3);
        lfd2 = -log((-log( lfc1 * exp( -0.5 ) + lfc2 ) - lfd3 ) / -lfd1);

        double lmin=-2, lmax=2, rmin=-M_PI/2, rmax=M_PI/2;
        double lstep = (lmax-lmin)/(N_LINEAR-1);
        double rstep = (rmax-rmin)/(N_ROT-1);
        Eigen::MatrixXd S(6,N_LINEAR);
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;

        LazyGrid<PointTarget> prototype(current_resolution);
        NDTMap<PointTarget> ndt( &prototype );
        ndt.loadPointCloud( fixed );
        ndt.computeNDTCells();

        int k=0;
        for(double x=lmin; x<lmax; x+=lstep)
        {
            T = Eigen::Translation<double,3>(x,0,0);
            //T = Eigen::Transform<double,3>(x,0,0);
            pcl::PointCloud<PointSource> cloud = moving;
            lslgeneric::transformPointCloudInPlace(T,cloud);

            S(0,k) = scorePointCloud(cloud,ndt);
            k++;
        }
        k=0;
        for(double x=lmin; x<lmax; x+=lstep)
        {
            T = Eigen::Translation<double,3>(0,x,0);
            //T = Eigen::Transform<double,3>(0,x,0);
            pcl::PointCloud<PointSource> cloud = moving;
            lslgeneric::transformPointCloudInPlace(T,cloud);

            S(1,k) = scorePointCloud(cloud,ndt);
            k++;
        }
        k=0;
        for(double x=lmin; x<lmax; x+=lstep)
        {
            T = Eigen::Translation<double,3>(0.,0.,x);
            //T = Eigen::Transform<double,3>(0.,0.,x);
            pcl::PointCloud<PointSource> cloud = moving;
            lslgeneric::transformPointCloudInPlace(T,cloud);

            S(2,k) = scorePointCloud(cloud,ndt);
            k++;
        }

        k=0;
        for(double r=rmin; r<rmax; r+=rstep)
        {
            T = Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ()) ;
            pcl::PointCloud<PointSource> cloud = moving;
            lslgeneric::transformPointCloudInPlace(T,cloud);
            S(3,k) = scorePointCloud(cloud,ndt);
            k++;
        }
        k=0;
        for(double r=rmin; r<rmax; r+=rstep)
        {
            T = Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ()) ;
            pcl::PointCloud<PointSource> cloud = moving;
            lslgeneric::transformPointCloudInPlace(T,cloud);

            S(4,k) = scorePointCloud(cloud,ndt);
            k++;
        }
        k=0;
        for(double r=rmin; r<rmax; r+=rstep)
        {
            T = Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitZ()) ;
            pcl::PointCloud<PointSource> cloud = moving;
            lslgeneric::transformPointCloudInPlace(T,cloud);

            S(5,k) = scorePointCloud(cloud,ndt);
            k++;
        }

        lg<<"Sp2f"<<(int)current_resolution<<" = ["<<S<<"];\n";
    }
    lg.close();

}

template <typename PointSource, typename PointTarget>
bool NDTMatcherP2D<PointSource,PointTarget>::match( pcl::PointCloud<PointTarget>& target,
        pcl::PointCloud<PointSource>& source,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T )
{

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Temp;
    T.setIdentity();
#ifdef DO_DEBUG_PROC
    char fname[50];
    snprintf(fname,49,"/home/tsv/ndt_tmp/initial_clouds.wrl");
    FILE *fout = fopen(fname,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");

    lslgeneric::writeToVRML<PointTarget>(fout,target,Eigen::Vector3d(0,1,0));
    lslgeneric::writeToVRML<PointSource>(fout,source,Eigen::Vector3d(1,0,0));
    fclose(fout);
#endif

    isIrregularGrid = false;
    bool ret;

    if(isIrregularGrid)
    {
        current_resolution=0.5;
        pcl::PointCloud<PointSource> cloud = subsample(source);
        OctTree<PointTarget> prototype;
        prototype.BIG_CELL_SIZE = 4;
        prototype.SMALL_CELL_SIZE = 0.5;
        NDTMap<PointTarget> ndt( &prototype );
        ndt.loadPointCloud( target );
        ndt.computeNDTCells();
        ret = this->match( ndt, cloud, T );

    }
    else
    {
        pcl::PointCloud<PointSource> moving = subsample(source);
        //cout<<"subsampled points size is "<<moving.points.size()<<endl;
        //iterative regular grid
        for(current_resolution = 2; current_resolution >= 0.5; current_resolution = current_resolution/2)
        {

            LazyGrid<PointTarget> prototype(current_resolution);
            NDTMap<PointTarget> ndt( &prototype );
            ndt.loadPointCloud( target );
            ndt.computeNDTCells();

            ret = this->match( ndt, moving, Temp );
            T = Temp*T;
            //transform moving
            lslgeneric::transformPointCloudInPlace(Temp,moving);
#ifdef DO_DEBUG_PROC
//	  cout<<"RESOLUTION: "<<current_resolution<<endl;
            Eigen::Vector3d out = Temp.rotation().eulerAngles(0,1,2);
            std::cout<<"OUT: "<<out.transpose()<<std::endl;
            std::cout<<"translation "<<Temp.translation().transpose()<<std::endl;
            //cout<<"--------------------------------------------------------\n";
            char fname[50];
            snprintf(fname,49,"/home/tsv/ndt_tmp/inner_cloud%lf.wrl",current_resolution);
            FILE *fout = fopen(fname,"w");
            fprintf(fout,"#VRML V2.0 utf8\n");
            lslgeneric::writeToVRML<PointTarget>(fout,target,Eigen::Vector3d(0,1,0));
            lslgeneric::writeToVRML<PointSource>(fout,moving,Eigen::Vector3d(1,0,0));
            //ndt.writeToVRML(fout);
            fclose(fout);
#endif
        }
    }

    return ret;
}

template <typename PointSource, typename PointTarget>
bool NDTMatcherP2D<PointSource,PointTarget>::covariance( pcl::PointCloud<PointTarget>& target,
        pcl::PointCloud<PointSource>& source,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
        Eigen::Matrix<double,6,6> &cov
                                                       )
{

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR;
    pcl::PointCloud<PointSource> cloud = source;
    lslgeneric::transformPointCloudInPlace(T,cloud);

    LazyGrid<PointTarget> prototype(current_resolution);
    NDTMap<PointTarget> ndt( &prototype );
    ndt.loadPointCloud( target );
    ndt.computeNDTCells();

    TR.setIdentity();
    Eigen::Matrix<double,6,1> sc;
    derivativesPointCloud(cloud,ndt,TR,sc,cov,true);
    //cout<<"cov:"<<cov<<endl;
    cov = 0.5*cov.inverse();
    //cout<<"cov2:"<<cov<<endl;

    return true;
}

template <typename PointSource, typename PointTarget>
bool NDTMatcherP2D<PointSource,PointTarget>::match( NDTMap<PointTarget>& targetNDT,
        pcl::PointCloud<PointSource>& source,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T )
{
    ///////////
    double lfc1,lfc2,lfd3;
    double integral, outlier_ratio, support_size;
    integral = 0.1;
    outlier_ratio = 0.35;
    //outlier_ratio = 0.35;
    support_size = current_resolution; //???
    lfc1 = (1-outlier_ratio)/integral;
    lfc2 = outlier_ratio/pow(support_size,3);
    lfd3 = -log(lfc2);
    lfd1 = -log( lfc1 + lfc2 ) - lfd3;
    lfd2 = -log((-log( lfc1 * exp( -0.5 ) + lfc2 ) - lfd3 ) / lfd1);
    //d1 = 0.1;
    //d2 = 0.001;
    //cout<<lfd1<<" "<<lfd2<<endl;
    ///////////
    useSimpleDerivatives = false;
    Jest.setZero();
    Jest.block<3,3>(0,0).setIdentity();
    Hest.setZero();

    //locals
    int ITR_MAX = 100;
    bool convergence = false;
    double score=0;
    double DELTA_SCORE = 0.0001;
//    double NORM_MAX = support_size/8, ROT_MAX = M_PI/18; //
    double NORM_MAX = 4*support_size, ROT_MAX = M_PI/4; //
    int itr_ctr = 0;
    double step_size = 1;
    Eigen::Matrix<double,6,1> pose_increment_v, pose_increment_reg_v, score_gradient; //column vectors
    Eigen::Matrix<double,6,6> Hessian;
    Eigen::Matrix3d cov;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR;
    Eigen::Vector3d transformed_vec, mean;
    bool ret = true;

    pcl::PointCloud<PointSource> prevCloud, nextCloud;
    prevCloud = source;
    nextCloud = source;
    T.setIdentity();
    TR.setIdentity();
    Eigen::Vector3d eulerAngles = T.rotation().eulerAngles(0,1,2);

    double scoreP = 0;
    while(!convergence)
    {
#ifdef DO_DEBUG_PROC
        if(itr_ctr == 0)
        {
            char fname[100];
            snprintf(fname,99,"/home/tsv/ndt_tmp/inner_cloud%lf_itr1_dbg.wrl",current_resolution);
            targetNDT.debugToVRML(fname, nextCloud);
        }
#endif

        score_gradient.setZero();
        Hessian.setZero();
        //derivativesPointCloud(source,targetNDT,T,score_gradient,Hessian,true);

        TR.setIdentity();
        derivativesPointCloud(prevCloud,targetNDT,TR,score_gradient,Hessian,true);

        pose_increment_v = Hessian.ldlt().solve(-score_gradient);

        score = scorePointCloud(prevCloud,targetNDT);
        //cout<<"iteration "<<itr_ctr<<" pose norm "<<(pose_increment_v.norm())<<" score "<<score<<endl;

//step control...
        double pnorm = sqrt(pose_increment_v(0)*pose_increment_v(0) + pose_increment_v(1)*pose_increment_v(1)
                            +pose_increment_v(2)*pose_increment_v(2));
        if(pnorm > NORM_MAX)
        {
            pose_increment_v(0) = NORM_MAX*pose_increment_v(0)/pnorm;
            pose_increment_v(1) = NORM_MAX*pose_increment_v(1)/pnorm;
            pose_increment_v(2) = NORM_MAX*pose_increment_v(2)/pnorm;
        }
        pose_increment_v(3) = normalizeAngle(pose_increment_v(3));
        pose_increment_v(3) = (pose_increment_v(3) > ROT_MAX) ? ROT_MAX : pose_increment_v(3);
        pose_increment_v(3) = (pose_increment_v(3) < -ROT_MAX) ? -ROT_MAX : pose_increment_v(3);
        pose_increment_v(4) = normalizeAngle(pose_increment_v(4));
        pose_increment_v(4) = (pose_increment_v(4) > ROT_MAX) ? ROT_MAX : pose_increment_v(4);
        pose_increment_v(4) = (pose_increment_v(4) < -ROT_MAX) ? -ROT_MAX : pose_increment_v(4);
        pose_increment_v(5) = normalizeAngle(pose_increment_v(5));
        pose_increment_v(5) = (pose_increment_v(5) > ROT_MAX) ? ROT_MAX : pose_increment_v(5);
        pose_increment_v(5) = (pose_increment_v(5) < -ROT_MAX) ? -ROT_MAX : pose_increment_v(5);
//	cout<<"H  =  ["<<Hessian<<"]"<<endl;
//	cout<<"grad= ["<<score_gradient.transpose()<<"]"<<endl;
//	cout<<"dg    "<<pose_increment_v.dot(score_gradient)<<endl;

        TR.setIdentity();
        TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
              Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

        step_size = lineSearchMT(score_gradient,pose_increment_v,prevCloud,TR,targetNDT);
        if(step_size < 0)
        {
            //    cout<<"can't decrease in this direction any more, done \n";
            return true;
        }
        pose_increment_v = step_size*pose_increment_v;


//	cout<<"incr= ["<<pose_increment_v.transpose()<<"]"<<endl;
        TR.setIdentity();
        TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
              Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
              Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;
        T = TR*T;

        //eulerAngles<<pose_increment_v(3),pose_increment_v(4),pose_increment_v(5);
        //eulerAngles = T.rotation().eulerAngles(0,1,2);

        prevCloud = lslgeneric::transformPointCloud<PointSource>(T,source);
        scoreP = score;
        score = scorePointCloud(prevCloud,targetNDT);

        //cout<<"iteration "<<itr_ctr<<" pose norm "<<(pose_increment_v.norm())<<" score_prev "<<scoreP<<" scoreN "<<score<<endl;
        //cout<<"step size "<<step_size<<endl;

        if(itr_ctr>0)
        {
            convergence = ((pose_increment_v.norm()) < DELTA_SCORE);
        }
        if(itr_ctr>ITR_MAX)
        {
            convergence = true;
            ret = false;
        }
        itr_ctr++;
    }
//    cout<<"res: "<<current_resolution<<" itr "<<itr_ctr<<endl;
//    cout<<"T: \n t = "<<T.translation().transpose()<<endl;
//    cout<<"r= \n"<<T.rotation()<<endl;

    this->finalscore = score/NUMBER_OF_ACTIVE_CELLS;
    return ret;
}


template <typename PointSource, typename PointTarget>
bool NDTMatcherP2D<PointSource,PointTarget>::update_score_gradient(Eigen::Matrix<double,6,1> &score_gradient,
        Eigen::Vector3d &transformed,
        Eigen::Matrix3d & Cinv)
{

    Eigen::Vector3d CxX, CxdX;
    double factor = (-lfd2*transformed.dot(Cinv*transformed)/2);

    //these conditions were copied from martin's code
    if(factor < -120)
    {
        return false;
    }
    factor = lfd2*exp(factor);
    if(factor > 1 || factor < 0 || factor*0 !=0)
    {
        return false;
    }
    factor *=lfd1;

    for(int i=0; i<6; i++)
    {
        CxdX = Cinv*Jest.col(i);
        score_gradient(i) += transformed.dot(CxdX)*factor;
    }
    return true;

}

template <typename PointSource, typename PointTarget>
void NDTMatcherP2D<PointSource,PointTarget>::update_hessian(Eigen::Matrix<double,6,6> &Hessian,
        Eigen::Vector3d &transformed,
        Eigen::Matrix3d & Cinv)
{

    Eigen::Vector3d CxX, CxdXdI, CxdXdJ, CxSecondOrder;
    CxX = Cinv*transformed;
    double factor = lfd1*lfd2*exp(-lfd2*transformed.dot(CxX)/2);
    for(int i=0; i<Hessian.rows(); i++)
    {
        for(int j=0; j<Hessian.cols(); j++)
        {

            CxdXdI = Cinv*Jest.col(i);
            CxdXdJ = Cinv*Jest.col(j);
            CxSecondOrder = Cinv*Hest.block<3,1>(3*i,j);

            Hessian(i,j) += factor*(-lfd2*transformed.dot(CxdXdI)*transformed.dot(CxdXdJ) +
                                    transformed.dot(CxSecondOrder) +
                                    Jest.col(j).dot(CxdXdI) );
        }
    }

}

template <typename PointSource, typename PointTarget>
void NDTMatcherP2D<PointSource,PointTarget>::precomputeAngleDerivatives(Eigen::Vector3d &eulerAngles)
{
    if(fabsf(eulerAngles(0)) < 10e-5) eulerAngles(0) = 0;
    if(fabsf(eulerAngles(1)) < 10e-5) eulerAngles(1) = 0;
    if(fabsf(eulerAngles(2)) < 10e-5) eulerAngles(2) = 0;
    double cx,cy,cz, sx,sy,sz;
    cx = cos(eulerAngles(0));
    cy = cos(eulerAngles(1));
    cz = cos(eulerAngles(2));
    sx = sin(eulerAngles(0));
    sy = sin(eulerAngles(1));
    sz = sin(eulerAngles(2));

    jest13 << (-sx*sz+cx*sy*cz) , (-sx*cz - cx*sy*sz) , (-cx*cy) ;
    jest23 << (cx*sz+sx*sy*cz) , (cx*cz-sx*sy*sz) , (-sx*cy);
    jest04 << (-sy*cz) , sy*sz , cy;
    jest14 << sx*cy*cz , (-sx*cy*sz) , sx*sy;
    jest24 << (-cx*cy*cz) , cx*cy*sz , (-cx*sy);
    jest05 << (-cy*sz) , (-cy*cz), 0;
    jest15 << (cx*cz-sx*sy*sz) , (-cx*sz - sx*sy*cz), 0;
    jest25 << (sx*cz + cx*sy*sz) ,(cx*sy*cz - sx*sz), 0;


    a2 << (-cx*sz-sx*sy*cz),(-cx*cz+sx*sy*sz),sx*cy;
    a3 << (-sx*sz+cx*sy*cz),(-cx*sy*sz-sx*cz),(-cx*cy);
    b2 << (cx*cy*cz),(-cx*cy*sz),(cx*sy);
    b3 << (sx*cy*cz),(-sx*cy*sz),(sx*sy);
    c2 << (-sx*cz-cx*sy*sz),(sx*sz-cx*sy*cz),0;
    c3 << (cx*cz-sx*sy*sz),(-sx*sy*cz-cx*sz),0;
    d1 << (-cy*cz),(cy*sz),(sy);
    d2 << (-sx*sy*cz),(sx*sy*sz),(sx*cy);
    d3 << (cx*sy*cz),(-cx*sy*sz),(-cx*cy);
    e1 << (sy*sz),(sy*cz),0;
    e2 << (-sx*cy*sz),(-sx*cy*cz),0;
    e3 << (cx*cy*sz),(cx*cy*cz),0;
    f1 << (-cy*cz),(cy*sz),0;
    f2 << (-cx*sz -sx*sy*cz),(-cx*cz+sx*sy*sz),0;
    f3 << (-sx*sz+cx*sy*cz),(-cx*sy*sz-sx*cz),0;

}

template <typename PointSource, typename PointTarget>
void NDTMatcherP2D<PointSource,PointTarget>::computeDerivatives(PointSource &pt)
{

    if(useSimpleDerivatives)
    {
        Jest(1,3) = -pt.z;
        Jest(2,3) = pt.y;
        Jest(0,4) = pt.z;
        Jest(2,4) = -pt.x;
        Jest(0,5) = -pt.y;
        Jest(1,5) = pt.x;

        return;
    }

    Eigen::Vector3d x;
    x<<pt.x,pt.y,pt.z;

    //full derivatives
    Jest(1,3) = x.dot(jest13);
    Jest(2,3) = x.dot(jest23);
    Jest(0,4) = x.dot(jest04);
    Jest(1,4) = x.dot(jest14);
    Jest(2,4) = x.dot(jest24);
    Jest(0,5) = x.dot(jest05);
    Jest(1,5) = x.dot(jest15);
    Jest(2,5) = x.dot(jest25);

    Eigen::Vector3d a,b,c,d,e,f;

    a<<0,x.dot(a2),x.dot(a3);
    b<<0,x.dot(b2),x.dot(b3);
    c<<0,x.dot(c2),x.dot(c3);
    d<<x.dot(d1),x.dot(d2),x.dot(d3);
    e<<x.dot(e1),x.dot(e2),x.dot(e3);
    f<<x.dot(f1),x.dot(f2),x.dot(f3);

    //Hest
    Hest.block<3,1>(9,3) = a;
    Hest.block<3,1>(12,3) = b;
    Hest.block<3,1>(15,3) = c;
    Hest.block<3,1>(9,4) = b;
    Hest.block<3,1>(12,4) = d;
    Hest.block<3,1>(15,4) = e;
    Hest.block<3,1>(9,5) = c;
    Hest.block<3,1>(12,5) = e;
    Hest.block<3,1>(15,5) = f;

}

template <typename PointSource, typename PointTarget>
double NDTMatcherP2D<PointSource,PointTarget>::scorePointCloud(pcl::PointCloud<PointSource> &source,
        NDTMap<PointTarget> &targetNDT)
{
    double score_here = 0;
    double score_native = 0;
    NDTCell<PointTarget> *cell;
    Eigen::Matrix3d icov;
    Eigen::Vector3d mean;
    Eigen::Vector3d point;
    NUMBER_OF_ACTIVE_CELLS = 0;
    for(unsigned int i=0; i<source.points.size(); i++)
    {
        point<<source.points[i].x,source.points[i].y,source.points[i].z;

        std::vector<NDTCell<PointTarget>*> cells = targetNDT.getCellsForPoint(source.points[i],current_resolution);
        for(unsigned int j=0; j<cells.size(); j++)
        {
            cell = cells[j];

            //{
            //    if(!targetNDT.getCellForPoint(source.points[i],cell)) {
            //        continue;
            //	}
            if(cell == NULL)
            {
                continue;
            }
            icov = cell->getInverseCov();
            mean = cell->getMean();
            double l = (point-mean).dot(icov*(point-mean));
            if(l*0 != 0) continue;

            if(isIrregularGrid)
            {
                //these lines should be done only for irregular grids!!
                double lfc1,lfc2,lfd3;
                double xs,ys,zs;
                double integral, outlier_ratio, support_size;
                integral = 0.1;
                outlier_ratio = 0.35;
                cell->getDimensions(xs,ys,zs);
                support_size = xs;
                lfc1 = (1-outlier_ratio)/integral;
                lfc2 = outlier_ratio/pow(support_size,3);
                lfd3 = -log(lfc2);
                lfd1 = -log( lfc1 + lfc2 ) - lfd3;
                lfd2 = -2*log((-log( lfc1 * exp( -0.5 ) + lfc2 ) - lfd3 ) / lfd1);
            }
            if(l > 120) continue;

            score_here += (lfd1*exp(-lfd2*l/2));
            score_native += (targetNDT.getLikelihoodForPoint(source.points[i]));
            NUMBER_OF_ACTIVE_CELLS += 1;
        }
    }
//    cout<<"here: "<<score_here<<" native: "<<score_native<<endl;
//    score_here /= NUMBER_OF_POINTS;
    return score_here;
}

//compute the score gradient of a point cloud + transformation to an NDT
template <typename PointSource, typename PointTarget>
void NDTMatcherP2D<PointSource,PointTarget>::derivativesPointCloud(pcl::PointCloud<PointSource> &source,
        NDTMap<PointTarget> &targetNDT,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
//	Eigen::Vector3d &eulerAngles,
        Eigen::Matrix<double,6,1> &score_gradient,
        Eigen::Matrix<double,6,6> &Hessian,
        bool computeHessian)
{

    NDTCell<PointTarget> *cell;
    Eigen::Vector3d transformed;
    Eigen::Matrix3d Cinv;
    Eigen::Vector3d eulerAngles = transform.rotation().eulerAngles(0,1,2);

    Jest.setZero();
    Jest.block<3,3>(0,0).setIdentity();
    Hest.setZero();

    score_gradient.setZero();
    Hessian.setZero();
    //precompute angles for the derivative matrices
    precomputeAngleDerivatives(eulerAngles);

    for(unsigned int i=0; i<source.points.size(); i++)
    {
        transformed<<source.points[i].x,source.points[i].y,source.points[i].z;
        transformed = transform*transformed;

        // vector<NDTCell*> cells = targetNDT.getCellsForPoint(source.points[i],current_resolution);
        // for( int j=0; j<cells.size(); j++) {
        // 	cell = cells[j];
        //
        {
            if(!targetNDT.getCellForPoint(source.points[i],cell))   //
            {
                continue;
            }

            if(cell == NULL)
            {
                continue;
            }
            transformed -=cell->getMean();
            Cinv = cell->getInverseCov();

            if(isIrregularGrid)
            {
                //these lines set the rescaling parameters for irregular cell grids
                double lfc1,lfc2,lfd3;
                double xs,ys,zs;
                double integral, outlier_ratio, support_size;
                integral = 0.1;
                outlier_ratio = 0.35;
                cell->getDimensions(xs,ys,zs);
                support_size = xs;
                lfc1 = (1-outlier_ratio)/integral;
                lfc2 = outlier_ratio/pow(support_size,3);
                lfd3 = -log(lfc2);
                lfd1 = -log( lfc1 + lfc2 ) - lfd3;
                lfd2 = -2*log((-log( lfc1 * exp( -0.5 ) + lfc2 ) - lfd3 ) / lfd1);
            }

            //compute Jest and Hest
            computeDerivatives(source.points[i]);

            //update score gradient
            if(!update_score_gradient(score_gradient, transformed, Cinv))
            {
                continue;
            }

            //update hessian matrix
            if(computeHessian)
            {
                update_hessian(Hessian, transformed, Cinv);
            }
            cell = NULL;
        }
    }
    score_gradient = -score_gradient;
    Hessian = -Hessian;
}

//perform line search to find the best descent rate (More&Thuente)
template <typename PointSource, typename PointTarget>
double NDTMatcherP2D<PointSource,PointTarget>::lineSearchMT(  Eigen::Matrix<double,6,1> &score_gradient_init,
        Eigen::Matrix<double,6,1> &increment,
        pcl::PointCloud<PointSource> &sourceCloud,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &globalT,
        NDTMap<PointTarget> &targetNDT)
{

    // default params
    double stp = 4.0; //default step
    double recoverystep = 0.05;
    double dginit = 0.0;
    double ftol = 0.0001; //epsilon 1
    double gtol = 0.9999; //epsilon 2
    double stpmax = 10.0;
    double stpmin = 0.0001;
    int maxfev = 10; //max function evaluations
    double xtol = 0.01; //window of uncertainty around the optimal step

    double direction = 1.0;
    //my temporary variables
    pcl::PointCloud<PointSource> cloudHere = sourceCloud;
    //cloudHere = lslgeneric::transformPointCloud(globalT,cloud);
    double score_init = 0.0;

    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ps,ps2;
    Eigen::Matrix<double,6,1> pincr, score_gradient_here;
    Eigen::Matrix<double,6,6> pseudoH;
    Eigen::Vector3d eulerAngles;
    /////

    int info = 0;			// return code
    int infoc = 1;		// return code for subroutine cstep

    // Compute the initial gradient in the search direction and check
    // that s is a descent direction.

    //we want to maximize s, so we should minimize -s
    score_init = scorePointCloud(cloudHere,targetNDT);

    //gradient directions are opposite for the negated function
    //score_gradient_init = -score_gradient_init;

//  cout<<"score_init "<<score_init<<endl;
//  cout<<"score_gradient_init "<<score_gradient_init.transpose()<<endl;
//  cout<<"increment "<<increment.transpose()<<endl;

    dginit = increment.dot(score_gradient_init);
//  cout<<"dginit "<<dginit<<endl;

    if (dginit >= 0.0)
    {
//    cout << "MoreThuente::cvsrch - wrong direction (dginit = " << dginit << ")" << endl;
        //return recoverystep; //TODO TSV -1; //

        increment = -increment;
        dginit = -dginit;
        direction = -1;

        if (dginit >= 0.0)
        {
//    cout << "MoreThuente::cvsrch - Non-descent direction (dginit = " << dginit << ")" << endl;
            //stp = recoverystep;
            //newgrp.computeX(oldgrp, dir, stp);
            return recoverystep;
        }
    }
    else
    {
//     cout<<"correct direction (dginit = " << dginit << ")" << endl;
    }

    // Initialize local variables.

    bool brackt = false;		// has the soln been bracketed?
    bool stage1 = true;		// are we in stage 1?
    int nfev = 0;			// number of function evaluations
    double dgtest = ftol * dginit; // f for curvature condition
    double width = stpmax - stpmin; // interval width
    double width1 = 2 * width;	// ???

    //cout<<"dgtest "<<dgtest<<endl;
    // initial function value
    double finit = 0.0;
    finit = score_init;

    // The variables stx, fx, dgx contain the values of the step,
    // function, and directional derivative at the best step.  The
    // variables sty, fy, dgy contain the value of the step, function,
    // and derivative at the other endpoint of the interval of
    // uncertainty.  The variables stp, f, dg contain the values of the
    // step, function, and derivative at the current step.

    double stx = 0.0;
    double fx = finit;
    double dgx = dginit;
    double sty = 0.0;
    double fy = finit;
    double dgy = dginit;

    // Get the linear solve tolerance for adjustable forcing term
    double eta_original = -1.0;
    double eta = 0.0;
    eta = eta_original;

    // Start of iteration.

    double stmin, stmax;
    double fm, fxm, fym, dgm, dgxm, dgym;

    while (1)
    {
        // Set the minimum and maximum steps to correspond to the present
        // interval of uncertainty.
        if (brackt)
        {
            stmin = MoreThuente::min(stx, sty);
            stmax = MoreThuente::max(stx, sty);
        }
        else
        {
            stmin = stx;
            stmax = stp + 4 * (stp - stx);
        }

        // Force the step to be within the bounds stpmax and stpmin.
        stp = MoreThuente::max(stp, stpmin);
        stp = MoreThuente::min(stp, stpmax);

        // If an unusual termination is to occur then let stp be the
        // lowest point obtained so far.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) ||
                (nfev >= maxfev - 1) || (infoc == 0) ||
                (brackt && (stmax - stmin <= xtol * stmax)))
        {
            stp = stx;
        }

        // Evaluate the function and gradient at stp
        // and compute the directional derivative.
        ///////////////////////////////////////////////////////////////////////////

        pincr = stp*increment;

        ps = Eigen::Translation<double,3>(pincr(0),pincr(1),pincr(2))*
             Eigen::AngleAxisd(pincr(3),Eigen::Vector3d::UnitX())*
             Eigen::AngleAxisd(pincr(4),Eigen::Vector3d::UnitY())*
             Eigen::AngleAxisd(pincr(5),Eigen::Vector3d::UnitZ());

        //ps2 = ps*globalT;
        //eulerAngles = ps2.rotation().eulerAngles(0,1,2);

        //eulerAngles<<pincr(3),pincr(4),pincr(5);
        cloudHere = lslgeneric::transformPointCloud(ps,sourceCloud);

        double f = 0.0;
        f = scorePointCloud(cloudHere,targetNDT);
        score_gradient_here.setZero();

        ps2.setIdentity();
        derivativesPointCloud(cloudHere,targetNDT,ps2,score_gradient_here,pseudoH,false);

        //derivativesPointCloud(cloud,ndt,ps,score_gradient_here,pseudoH,false);

        //derivativesPointCloud(cloudHere,ndt,ps2,score_gradient_here,pseudoH,false);
        //negate score gradient
        //score_gradient_here = -score_gradient_here;

        //cout<<"incr " <<pincr.transpose()<<endl;
        //cout<<"scg  " <<score_gradient_here.transpose()<<endl;
        //cout<<"score (f) "<<f<<endl;

        //VALGRIND_CHECK_VALUE_IS_DEFINED(score_gradient_here);
        //VALGRIND_CHECK_VALUE_IS_DEFINED(increment);
        double dg = 0.0;
        dg = increment.dot(score_gradient_here);


        //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
        //cout<<"dg = "<<dg<<endl;
        nfev ++;

///////////////////////////////////////////////////////////////////////////

        //cout<<"consider step "<<stp<<endl;
        // Armijo-Goldstein sufficient decrease
        double ftest1 = finit + stp * dgtest;
        //cout<<"ftest1 is "<<ftest1<<endl;

        // Test for convergence.

        if ((brackt && ((stp <= stmin) || (stp >= stmax))) || (infoc == 0))
            info = 6;			// Rounding errors

        if ((stp == stpmax) && (f <= ftest1) && (dg <= dgtest))
            info = 5;			// stp=stpmax

        if ((stp == stpmin) && ((f > ftest1) || (dg >= dgtest)))
            info = 4;			// stp=stpmin

        if (nfev >= maxfev)
            info = 3;			// max'd out on fevals

        if (brackt && (stmax-stmin <= xtol*stmax))
            info = 2;			// bracketed soln

        // RPP sufficient decrease test can be different
        bool sufficientDecreaseTest = false;
        sufficientDecreaseTest = (f <= ftest1);  // Armijo-Golstein

        //cout<<"ftest2 "<<gtol*(-dginit)<<endl;
        //cout<<"sufficientDecrease? "<<sufficientDecreaseTest<<endl;
        //cout<<"curvature ok? "<<(fabs(dg) <= gtol*(-dginit))<<endl;
        if ((sufficientDecreaseTest) && (fabs(dg) <= gtol*(-dginit)))
            info = 1;			// Success!!!!

        if (info != 0) 		// Line search is done
        {
            if (info != 1) 		// Line search failed
            {
                // RPP add
                // counter.incrementNumFailedLineSearches();

                //if (recoveryStepType == Constant)
                stp = recoverystep;

                //newgrp.computeX(oldgrp, dir, stp);

                //message = "(USING RECOVERY STEP!)";

            }
            else 			// Line search succeeded
            {
                //message = "(STEP ACCEPTED!)";
            }

            //print.printStep(nfev, stp, finit, f, message);

            // Returning the line search flag
            //cout<<"LineSearch::"<<message<<" info "<<info<<endl;
            return stp;

        } // info != 0

        // RPP add
        //counter.incrementNumIterations();

        // In the first stage we seek a step for which the modified
        // function has a nonpositive value and nonnegative derivative.

        if (stage1 && (f <= ftest1) && (dg >= MoreThuente::min(ftol, gtol) * dginit))
        {
            stage1 = false;
        }

        // A modified function is used to predict the step only if we have
        // not obtained a step for which the modified function has a
        // nonpositive function value and nonnegative derivative, and if a
        // lower function value has been obtained but the decrease is not
        // sufficient.

        if (stage1 && (f <= fx) && (f > ftest1))
        {

            // Define the modified function and derivative values.

            fm = f - stp * dgtest;
            fxm = fx - stx * dgtest;
            fym = fy - sty * dgtest;
            dgm = dg - dgtest;
            dgxm = dgx - dgtest;
            dgym = dgy - dgtest;

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dgm);
            infoc = MoreThuente::cstep(stx,fxm,dgxm,sty,fym,dgym,stp,fm,dgm,
                                       brackt,stmin,stmax);

            // Reset the function and gradient values for f.

            fx = fxm + stx*dgtest;
            fy = fym + sty*dgtest;
            dgx = dgxm + dgtest;
            dgy = dgym + dgtest;

        }

        else
        {

            // Call cstep to update the interval of uncertainty
            // and to compute the new step.

            //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
            infoc = MoreThuente::cstep(stx,fx,dgx,sty,fy,dgy,stp,f,dg,
                                       brackt,stmin,stmax);

        }

        // Force a sufficient decrease in the size of the
        // interval of uncertainty.

        if (brackt)
        {
            if (fabs(sty - stx) >= 0.66 * width1)
                stp = stx + 0.5 * (sty - stx);
            width1 = width;
            width = fabs(sty-stx);
        }

    } // while-loop

}


template <typename PointSource, typename PointTarget>
int NDTMatcherP2D<PointSource,PointTarget>::MoreThuente::cstep(double& stx, double& fx, double& dx,
        double& sty, double& fy, double& dy,
        double& stp, double& fp, double& dp,
        bool& brackt, double stmin, double stmax)
{
    int info = 0;

    // Check the input parameters for errors.

    if ((brackt && ((stp <= MoreThuente::min(stx, sty)) || (stp >= MoreThuente::max(stx, sty)))) ||
            (dx * (stp - stx) >= 0.0) || (stmax < stmin))
        return info;

    // Determine if the derivatives have opposite sign.

    double sgnd = dp * (dx / fabs(dx));

    // First case. A higher function value.  The minimum is
    // bracketed. If the cubic step is closer to stx than the quadratic
    // step, the cubic step is taken, else the average of the cubic and
    // quadratic steps is taken.

    bool bound;
    double theta;
    double s;
    double gamma;
    double p,q,r;
    double stpc, stpq, stpf;

    if (fp > fx)
    {
        info = 1;
        bound = 1;
        theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        //VALGRIND_CHECK_VALUE_IS_DEFINED(theta);
        //VALGRIND_CHECK_VALUE_IS_DEFINED(dx);
        //VALGRIND_CHECK_VALUE_IS_DEFINED(dp);
        s = MoreThuente::absmax(theta, dx, dp);
        gamma = s * sqrt(((theta / s) * (theta / s)) - (dx / s) * (dp / s));
        if (stp < stx)
            gamma = -gamma;

        p = (gamma - dx) + theta;
        q = ((gamma - dx) + gamma) + dp;
        r = p / q;
        stpc = stx + r * (stp - stx);
        stpq = stx + ((dx / ((fx - fp) / (stp - stx) + dx)) / 2) * (stp - stx);
        if (fabs(stpc - stx) < fabs(stpq - stx))
            stpf = stpc;
        else
            stpf = stpc + (stpq - stpc) / 2;

        brackt = true;
    }

    // Second case. A lower function value and derivatives of opposite
    // sign. The minimum is bracketed. If the cubic step is closer to
    // stx than the quadratic (secant) step, the cubic step is taken,
    // else the quadratic step is taken.

    else if (sgnd < 0.0)
    {
        info = 2;
        bound = false;
        theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        s = MoreThuente::absmax(theta,dx,dp);
        gamma = s * sqrt(((theta/s) * (theta/s)) - (dx / s) * (dp / s));
        if (stp > stx)
            gamma = -gamma;
        p = (gamma - dp) + theta;
        q = ((gamma - dp) + gamma) + dx;
        r = p / q;
        stpc = stp + r * (stx - stp);
        stpq = stp + (dp / (dp - dx)) * (stx - stp);
        if (fabs(stpc - stp) > fabs(stpq - stp))
            stpf = stpc;
        else
            stpf = stpq;
        brackt = true;
    }

    // Third case. A lower function value, derivatives of the same sign,
    // and the magnitude of the derivative decreases.  The cubic step is
    // only used if the cubic tends to infinity in the direction of the
    // step or if the minimum of the cubic is beyond stp. Otherwise the
    // cubic step is defined to be either stmin or stmax. The
    // quadratic (secant) step is also computed and if the minimum is
    // bracketed then the the step closest to stx is taken, else the
    // step farthest away is taken.

    else if (fabs(dp) < fabs(dx))
    {
        info = 3;
        bound = true;
        theta = 3 * (fx - fp) / (stp - stx) + dx + dp;
        s = MoreThuente::absmax(theta, dx, dp);

        // The case gamma = 0 only arises if the cubic does not tend
        // to infinity in the direction of the step.

        gamma = s * sqrt(max(0,(theta / s) * (theta / s) - (dx / s) * (dp / s)));
        if (stp > stx)
            gamma = -gamma;

        p = (gamma - dp) + theta;
        q = (gamma + (dx - dp)) + gamma;
        r = p / q;
        if ((r < 0.0) && (gamma != 0.0))
            stpc = stp + r * (stx - stp);
        else if (stp > stx)
            stpc = stmax;
        else
            stpc = stmin;

        stpq = stp + (dp/ (dp - dx)) * (stx - stp);
        if (brackt)
        {
            if (fabs(stp - stpc) < fabs(stp - stpq))
                stpf = stpc;
            else
                stpf = stpq;
        }
        else
        {
            if (fabs(stp - stpc) > fabs(stp - stpq))
                stpf = stpc;
            else
                stpf = stpq;
        }
    }

    // Fourth case. A lower function value, derivatives of the same
    // sign, and the magnitude of the derivative does not decrease. If
    // the minimum is not bracketed, the step is either stmin or
    // stmax, else the cubic step is taken.

    else
    {
        info = 4;
        bound = false;
        if (brackt)
        {
            theta = 3 * (fp - fy) / (sty - stp) + dy + dp;
            s = MoreThuente::absmax(theta, dy, dp);
            gamma = s * sqrt(((theta/s)*(theta/s)) - (dy / s) * (dp / s));
            if (stp > sty)
                gamma = -gamma;
            p = (gamma - dp) + theta;
            q = ((gamma - dp) + gamma) + dy;
            r = p / q;
            stpc = stp + r * (sty - stp);
            stpf = stpc;
        }
        else if (stp > stx)
            stpf = stmax;
        else
            stpf = stmin;
    }

    // Update the interval of uncertainty. This update does not depend
    // on the new step or the case analysis above.

    if (fp > fx)
    {
        sty = stp;
        fy = fp;
        dy = dp;
    }
    else
    {
        if (sgnd < 0.0)
        {
            sty = stx;
            fy = fx;
            dy = dx;
        }
        stx = stp;
        fx = fp;
        dx = dp;
    }

    // Compute the new step and safeguard it.

    stpf = MoreThuente::min(stmax, stpf);
    stpf = MoreThuente::max(stmin, stpf);
    stp = stpf;
    if (brackt && bound)
    {
        if (sty > stx)
            stp = min(stx + 0.66 * (sty - stx), stp);
        else
            stp = max(stx + 0.66 * (sty - stx), stp);
    }

    return info;

}

template <typename PointSource, typename PointTarget>
double NDTMatcherP2D<PointSource,PointTarget>::MoreThuente::min(double a, double b)
{
    return (a < b ? a : b);
}

template <typename PointSource, typename PointTarget>
double NDTMatcherP2D<PointSource,PointTarget>::MoreThuente::max(double a, double b)
{
    return (a > b ? a : b);
}

template <typename PointSource, typename PointTarget>
double NDTMatcherP2D<PointSource,PointTarget>::MoreThuente::absmax(double a, double b, double c)
{
    a = fabs(a);
    b = fabs(b);
    c = fabs(c);

    if (a > b)
        return (a > c) ? a : c;
    else
        return (b > c) ? b : c;
}

template <typename PointSource, typename PointTarget>
pcl::PointCloud<PointSource> NDTMatcherP2D<PointSource,PointTarget>::subsample(pcl::PointCloud<PointSource>& original)
{

    std::string subsampleType = "TREE";
    if(subsampleType == "NONE")
    {
        return original;
    }
    if(subsampleType == "GRID")
    {
        double subsampleRes = 0.4;//current_resolution/2;
        pcl::PointCloud<PointSource> res;
        LazyGrid<PointSource> prototype(subsampleRes);
        NDTMap<PointSource> ndt( &prototype );
        ndt.loadPointCloud( original );
        typename std::vector<Cell<PointSource>*>::iterator it = ndt.getMyIndex()->begin();

        while(it!=ndt.getMyIndex()->end())
        {
            NDTCell<PointSource>* ndcell = dynamic_cast<NDTCell<PointSource>*>(*it);
            if(ndcell!=NULL)
            {
                if(ndcell->points_.size() > 0)
                {
                    res.points.push_back(ndcell->points_.front());
                }
            }
            it++;
        }
        return res;

    }

    if(subsampleType == "MEAN")
    {
        pcl::PointCloud<PointSource> res;

        NDTMap<PointSource> ndt( new OctTree<PointSource>() );
        ndt.loadPointCloud( original );
        ndt.computeNDTCells();
        typename std::vector<Cell<PointSource>*>::iterator it = ndt.getMyIndex()->begin();

        while(it!=ndt.getMyIndex()->end())
        {
            NDTCell<PointSource>* ndcell = dynamic_cast<NDTCell<PointSource>*>(*it);
            if(ndcell!=NULL)
            {
                if(ndcell->points_.size() > 0)
                {
                    PointSource pt;
                    Eigen::Vector3d m = ndcell->getMean();
                    pt.x = m(0);
                    pt.y = m(1);
                    pt.z = m(2);
                    res.points.push_back(pt);
                }
            }
            it++;
        }
        return res;

    }

    if(subsampleType == "TREE")
    {
        //   double subsampleRes = 0.2;
        pcl::PointCloud<PointSource> res;
        //LazyGrid prototype(subsampleRes);
        //NDTMap ndt( &prototype );

        NDTMap<PointSource> ndt( new OctTree<PointSource>() );
        ndt.loadPointCloud( original );
        ndt.computeNDTCells();
        typename std::vector<Cell<PointSource>*>::iterator it = ndt.getMyIndex()->begin();

        while(it!=ndt.getMyIndex()->end())
        {
            NDTCell<PointSource>* ndcell = dynamic_cast<NDTCell<PointSource>*>(*it);
            if(ndcell!=NULL)
            {
                if(ndcell->points_.size() > 0)
                {
                    res.points.push_back(ndcell->points_.front());
                }
            }
            it++;
        }
        return res;
    }


    return original;
}

template <typename PointSource, typename PointTarget>
double NDTMatcherP2D<PointSource,PointTarget>::normalizeAngle(double a)
{
    //set the angle between -M_PI and M_PI
    return atan2(sin(a), cos(a));

}

}
