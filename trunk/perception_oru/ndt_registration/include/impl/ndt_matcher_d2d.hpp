#include "oc_tree.h"
#include "ndt_cell.h"
#include "lazy_grid.h"
#include "pointcloud_utils.h"

#include "Eigen/Eigen"
#include <fstream>

namespace lslgeneric {

#define DO_DEBUG_PROC
    
template <typename PointSource, typename PointTarget>
void NDTMatcherD2D<PointSource,PointTarget>::init(bool _isIrregularGrid, 
			     bool useDefaultGridResolutions, std::vector<double> _resolutions){
    Jest.setZero();
    Jest.block<3,3>(0,0).setIdentity();
    Hest.setZero();
    Zest.setZero();
    ZHest.setZero();

    isIrregularGrid = _isIrregularGrid;
    if(useDefaultGridResolutions) {
	resolutions.push_back(0.2);
	resolutions.push_back(0.5);
	resolutions.push_back(1);
	resolutions.push_back(2);
    } else {
	resolutions = _resolutions;
    }

    precomputeAngleDerivatives();
    current_resolution = 0.1; // Argggg!!! This is very important to have initiated! (one day debugging later) :-) TODO do we need to set it to anything better?
    lfd1 = 1; //lfd1/(double)sourceNDT.getMyIndex()->size(); //current_resolution*2.5;
    lfd2 = 0.05; //0.1/current_resolution;
    ITR_MAX = 10;
    
}
    

template <typename PointSource, typename PointTarget>
bool NDTMatcherD2D<PointSource,PointTarget>::match( pcl::PointCloud<PointTarget>& target, 
			pcl::PointCloud<PointSource>& source,
			Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T ,
			bool useInitialGuess)
{
    
  struct timeval tv_start, tv_end;
  struct timeval tv_start0, tv_end0;
  double time_load =0, time_match=0, time_combined=0;

  gettimeofday(&tv_start0,NULL);

  //initial guess
  pcl::PointCloud<PointSource> sourceCloud = source;
  if(useInitialGuess) {
      lslgeneric::transformPointCloudInPlace(T,sourceCloud);
  }

  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Temp;
  T.setIdentity();  
  bool ret;

#ifdef DO_DEBUG_PROC
  char fname[50];
  snprintf(fname,49,"initial_clouds.wrl");
  FILE *fout = fopen(fname,"w");
  fprintf(fout,"#VRML V2.0 utf8\n");

  lslgeneric::writeToVRML(fout,target,Eigen::Vector3d(1,0,0));
  lslgeneric::writeToVRML(fout,source,Eigen::Vector3d(0,1,0));
  fclose(fout);
#endif

  if(isIrregularGrid) {

      OctTree<PointTarget> pr1;
      NDTMap<PointTarget> targetNDT( &pr1 );
      targetNDT.loadPointCloud( target );
      targetNDT.computeNDTCells();

      OctTree<PointSource> pr2;
      NDTMap<PointSource> sourceNDT( &pr2 );
      sourceNDT.loadPointCloud( source );
      sourceNDT.computeNDTCells();

      ret = this->match( targetNDT, sourceNDT, T );

  } else {

      //iterative regular grid
      //TODO fix to be intuitative...
      for(int r_ctr = resolutions.size()-1; r_ctr >=0;  r_ctr--) { 

	  current_resolution = resolutions[r_ctr];

	  LazyGrid<PointSource> prototypeSource(current_resolution);
	  LazyGrid<PointTarget> prototypeTarget(current_resolution);
	 
	  gettimeofday(&tv_start,NULL);
	  NDTMap<PointTarget> targetNDT( &prototypeTarget );
	  targetNDT.loadPointCloud( target );
	  targetNDT.computeNDTCells();

	  NDTMap<PointSource> sourceNDT( &prototypeSource );
	  sourceNDT.loadPointCloud( sourceCloud );
	  sourceNDT.computeNDTCells();
	  gettimeofday(&tv_end,NULL);

	  time_load += (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.; 
	  Temp.setIdentity();  
	  
	  gettimeofday(&tv_start,NULL);
	  ret = this->match( targetNDT, sourceNDT, Temp );
	  lslgeneric::transformPointCloudInPlace(Temp,sourceCloud);
	  gettimeofday(&tv_end,NULL);
	  
	  time_match += (tv_end.tv_sec-tv_start.tv_sec)*1000.+(tv_end.tv_usec-tv_start.tv_usec)/1000.; 

	  //transform moving
	  T = Temp*T;

#ifdef DO_DEBUG_PROC
	  std::cout<<"RESOLUTION: "<<current_resolution<<std::endl;
	  std::cout<<"rotation   : "<<Temp.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
	  std::cout<<"translation: "<<Temp.translation().transpose()<<std::endl;
	  std::cout<<"--------------------------------------------------------\nOverall Transform:\n";
	  std::cout<<"rotation   : "<<T.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
	  std::cout<<"translation: "<<T.translation().transpose()<<std::endl;
	  
	  char fname[50];
	  snprintf(fname,49,"inner_cloud%lf.wrl",current_resolution);
	  FILE *fout = fopen(fname,"w");
	  fprintf(fout,"#VRML V2.0 utf8\n");

	  std::vector<NDTCell<PointSource>*> nextNDT = sourceNDT.pseudoTransformNDT(Temp);
	  targetNDT.writeToVRML(fout,Eigen::Vector3d(1,0,0));

	  for(unsigned int i=0; i<nextNDT.size(); i++) 
	  {
	      if(nextNDT[i]!=NULL) 
	      {
		  nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
		  delete nextNDT[i];
	      }
	  }
	  lslgeneric::writeToVRML(fout,sourceCloud,Eigen::Vector3d(0,1,0));
	  fclose(fout);
	  
	  snprintf(fname,49,"cloud%lf.wrl",current_resolution);
	  fout = fopen(fname,"w");
	  fprintf(fout,"#VRML V2.0 utf8\n");
	  lslgeneric::writeToVRML(fout,target,Eigen::Vector3d(1,0,0));
	  lslgeneric::writeToVRML(fout,sourceCloud,Eigen::Vector3d(0,1,0));
	  fclose(fout);
	  
#endif
      }
  }
  gettimeofday(&tv_end0,NULL);
  time_combined = (tv_end0.tv_sec-tv_start0.tv_sec)*1000.+(tv_end0.tv_usec-tv_start0.tv_usec)/1000.; 
  std::cout<<"load: "<<time_load<<" match "<<time_match<<" combined "<<time_combined<<std::endl;
  return ret;
}

template <typename PointSource, typename PointTarget>
bool NDTMatcherD2D<PointSource,PointTarget>::match( NDTMap<PointTarget>& targetNDT, 
			NDTMap<PointSource>& sourceNDT,
			Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T ,
			bool useInitialGuess)
{
    /*
    Jest.setZero();
    Jest.block<3,3>(0,0).setIdentity();
    Hest.setZero();
    Zest.setZero();
    ZHest.setZero();*/
    
//    lfd1 = 1;//lfd1/(double)sourceNDT.getMyIndex()->size(); //current_resolution*2.5;
//    lfd2 = 0.05; //0.1/current_resolution;
//    cout<<lfd1<<" "<<lfd2<<endl;
    ///////////
  
    //locals 
    bool convergence = false;
    //double score=0;
    double DELTA_SCORE = 10e-4*current_resolution;
    //double DELTA_SCORE = 0.0005;
    double NORM_MAX = 4*current_resolution, ROT_MAX = M_PI/4; //
    int itr_ctr = 0;
    double step_size = 1;
    Eigen::Matrix<double,6,1> pose_increment_v, pose_increment_reg_v, score_gradient; //column vectors
    Eigen::Matrix<double,6,6> Hessian;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR;
    Eigen::Vector3d transformed_vec, mean;
    bool ret = true;
    if(!useInitialGuess) {
	T.setIdentity();
    }

    std::vector<NDTCell<PointSource>*> nextNDT = sourceNDT.pseudoTransformNDT(T);
/*
#ifdef DO_DEBUG_PROC	  
    char fname[50];
    snprintf(fname,49,"innit%lf.wrl",current_resolution);
    FILE *fout = fopen(fname,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    targetNDT.writeToVRML(fout,Eigen::Vector3d(1,0,0));
    for(unsigned int i=0; i<nextNDT.size(); i++) 
    {
	if(nextNDT[i]!=NULL) 
	{
	    nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
	}
    }
    fclose(fout);
#endif
*/

    while(!convergence) 
    {

	TR.setIdentity();
	derivativesNDT(nextNDT,targetNDT,TR,score_gradient,Hessian,true);
	if (fabs(pose_increment_v.dot(score_gradient))<= std::numeric_limits<double>::min())
	{
	    std::cout<<"gradient vanished\n";
	    return true;
	}
//	cout<<"H  =  ["<<Hessian<<"]"<<endl;
//	cout<<"grad= ["<<score_gradient.transpose()<<"]"<<endl;
//	cout<<"pose_increment_v= ["<<pose_increment_v<<"]"<<endl;
//	cout<<"dg    "<<pose_increment_v.dot(score_gradient)<<endl;
	
	pose_increment_v = Hessian.ldlt().solve(-score_gradient);
	
	//pose_increment_v = Hessian.svd().solve(-score_gradient);
        
	//std::cout<<"iteration "<<itr_ctr<<" pose norm "<<(pose_increment_v.norm())<<std::endl;
	//make the initial increment reasonable...
	//cout<<"incr_init = ["<<pose_increment_v.transpose()<<"]"<<endl;
	
	double pnorm = sqrt(pose_increment_v(0)*pose_increment_v(0) + pose_increment_v(1)*pose_increment_v(1) 
			    +pose_increment_v(2)*pose_increment_v(2));
	if(pnorm > NORM_MAX) {
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
	
	step_size = lineSearchMT(score_gradient,pose_increment_v,nextNDT,TR,targetNDT);
	if(step_size < 0) {
	    std::cout<<"can't decrease in this direction any more, done \n";
	    return true;
	}
	pose_increment_v = step_size*pose_increment_v;
	
	TR.setIdentity();
	TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
	    Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
	    Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;
	
	//std::cout<<"incr= ["<<pose_increment_v.transpose()<<"]"<<std::endl;
	//transform source NDT
	Eigen::Matrix3d t2 = (T.rotation().transpose()*TR.rotation()*T.rotation());
	
	for(unsigned int i=0; i<nextNDT.size(); i++) 
	{
	    if(nextNDT[i]!=NULL)
	    {
		if(nextNDT[i]->hasGaussian_) 
		{
		    Eigen::Vector3d _mean = nextNDT[i]->getMean();
		    Eigen::Matrix3d _cov = nextNDT[i]->getCov();
		    _mean = TR*_mean;
		    _cov = t2.transpose()*_cov*t2;
		    nextNDT[i]->setMean(_mean);
		    nextNDT[i]->setCov(_cov);
		}
//		delete nextNDT[i];
	    }
	}

	T = TR*T;
	//nextNDT = sourceNDT.pseudoTransformNDT(T);
	if(itr_ctr>0) {
	    convergence = ((pose_increment_v.norm()) < DELTA_SCORE);
	}
	if(itr_ctr>ITR_MAX) {
	    convergence = true;
	    ret = false;
	}
	itr_ctr++;
	//std::cout<<"step size "<<step_size<<std::endl;
    }
/*    
    snprintf(fname,49,"final.wrl");
    fout = fopen(fname,"w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    targetNDT.writeToVRML(fout,Eigen::Vector3d(1,0,0));
    for(unsigned int i=0; i<nextNDT.size(); i++) 
    {
	if(nextNDT[i]!=NULL) 
	{
	    nextNDT[i]->writeToVRML(fout,Eigen::Vector3d(0,1,0));
	}
    }
    fclose(fout);
*/    
    //std::cout<<"res "<<current_resolution<<" itr "<<itr_ctr<<std::endl;
    for(unsigned int i=0; i<nextNDT.size(); i++) {
	if(nextNDT[i]!=NULL)
	    delete nextNDT[i];
    }

//    this->finalscore = score/NUMBER_OF_ACTIVE_CELLS;

    return ret;
}
    
template <typename PointSource, typename PointTarget>
bool NDTMatcherD2D<PointSource,PointTarget>::covariance( pcl::PointCloud<PointTarget>& target, 
	pcl::PointCloud<PointSource>& source,
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
	Eigen::Matrix<double,6,6> &cov
	) {
    
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR;
    TR.setIdentity();

    pcl::PointCloud<PointSource> sourceCloud = lslgeneric::transformPointCloud(T,source);

    LazyGrid<PointSource> prototypeSource(resolutions.front());
    LazyGrid<PointTarget> prototypeTarget(resolutions.front());

    NDTMap<PointTarget> targetNDT( &prototypeTarget );
    targetNDT.loadPointCloud( target );
    targetNDT.computeNDTCells();

    NDTMap<PointSource> sourceNDT( &prototypeSource );
    sourceNDT.loadPointCloud( sourceCloud );
    sourceNDT.computeNDTCells();

    this->covariance(targetNDT,sourceNDT,TR,cov);

    return true;
}

template <typename PointSource, typename PointTarget>
bool NDTMatcherD2D<PointSource,PointTarget>::covariance( NDTMap<PointTarget>& targetNDT, 
	NDTMap<PointSource>& sourceNDT,
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
	Eigen::Matrix<double,6,6> &cov
	) {

    double sigmaS = (0.03)*(0.03);
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR;
    TR.setIdentity();
    
    std::vector<NDTCell<PointSource>*> sourceNDTN = sourceNDT.pseudoTransformNDT(T);
    std::vector<NDTCell<PointTarget>*> targetNDTN = targetNDT.pseudoTransformNDT(T);

    Eigen::Matrix<double,6,1> scg; //column vectors
    int NM = sourceNDTN.size() + targetNDTN.size();

    Eigen::MatrixXd Jdpdz(NM,6);
    
    NDTCell<PointTarget> *cell;
    Eigen::Vector3d transformed;
    Eigen::Vector3d meanMoving, meanFixed;
    Eigen::Matrix3d CMoving, CFixed, Cinv;
    bool exists = false;
    double det = 0;
    Eigen::Matrix<double,6,1> ones;
    ones<<1,1,1,1,1,1;
    derivativesNDT(sourceNDTN,targetNDT,T,scg,cov,true);

    Eigen::Matrix3d Q;
    Jdpdz.setZero();
    Q.setZero();
    
    PointTarget point; 
    //now compute Jdpdz
    for(int i=0; i<sourceNDTN.size(); i++) {
	meanMoving = sourceNDTN[i]->getMean();
	point.x = meanMoving(0); 
	point.y = meanMoving(1); 
	point.z = meanMoving(2);

	if(!targetNDT.getCellForPoint(point,cell)) {
	    continue;
	}
	if(cell == NULL) {
	    continue;
	}
	if(cell->hasGaussian_) {

	    meanFixed = cell->getMean();
	    transformed = meanMoving-meanFixed;
	    CFixed = cell->getCov(); 
	    CMoving= sourceNDTN[i]->getCov();

	    (CFixed+CMoving).computeInverseAndDetWithCheck(Cinv,det,exists);
	    if(!exists) continue;

	    //compute Jdpdz.col(i)
	    double factor = (-transformed.dot(Cinv*transformed)/2);
	    //these conditions were copied from martin's code
	    if(factor < -120) {
		continue; 
	    }
	    factor = exp(lfd2*factor)/2;
	    if(factor > 1 || factor < 0 || factor*0 !=0) {
		continue;
	    }

	    Q = -sigmaS*Cinv*Cinv;

	    Eigen::Matrix<double,6,1> G, xtQJ;

	    G.setZero();
	    for(int q=0; q<6; q++) {
		G(q) =  -transformed.transpose()*Q*Zest.block<3,3>(0,3*q)*Cinv*transformed;
		G(q) = G(q) -transformed.transpose()*Cinv*Zest.block<3,3>(0,3*q)*Q*transformed;
	    }

	    xtQJ = transformed.transpose()*Q*Jest;

	    double f1 = (transformed.transpose()*Q*transformed);
	    G = G + xtQJ + (-lfd2/2)*f1*ones;
	    G = G*factor*lfd1*lfd2/2;

	    Jdpdz.row(i) = G.transpose();
	    
	    for(int j=0; j<targetNDTN.size(); j++) {
		if(targetNDTN[j]->getMean() == meanFixed) {

		    Jdpdz.row(j+sourceNDTN.size()) = Jdpdz.row(j+sourceNDTN.size())+G.transpose();
		    continue;
		}
	    }

	    cell = NULL;
	}
    }
     
   
    //cout<<Jdpdz.transpose()<<endl; 

    Eigen::MatrixXd JK(6,6);
    JK = sigmaS*Jdpdz.transpose()*Jdpdz;
    
    //cout<<"J*J'\n"<<JK<<endl;
    //cout<<"H\n"<<cov<<endl;

    cov = cov.inverse()*JK*cov.inverse();
    //cov = cov.inverse();//*fabsf(scoreNDT(sourceNDTN,targetNDT)*2/3);
    //cout<<"cov\n"<<cov<<endl;

    for(unsigned int q=0; q<sourceNDTN.size(); q++) {
	delete sourceNDTN[q];
    }
    sourceNDTN.clear();
    
    return true;
}


template <typename PointSource, typename PointTarget>
bool NDTMatcherD2D<PointSource,PointTarget>::update_gradient_hessian(
	Eigen::Matrix<double,6,1> &score_gradient,
	Eigen::Matrix<double,6,6> &Hessian,
	Eigen::Vector3d & x,
	Eigen::Matrix3d & B) {

    //vars for gradient
    Eigen::Matrix<double,6,1> xtBJ, xtBZBx, Q;
    //vars for hessian
    Eigen::Matrix<double,6,6> JtBJ, xtBZBJ, xtBH, xtBZBZBx, xtBZhBx;
    Eigen::Matrix<double,1,3> TMP1;

    double factor = (-x.dot(B*x)/2);

    //these conditions were copied from martin's code
    if(factor < -120) {
	 //std::cout << "factor : " << factor << std::endl;
	 return false;
    }

    factor = exp(lfd2*factor)/2;
    if(factor > 1 || factor < 0 || factor*0 !=0) {
	 //std::cout << "factor > 1 || factor < 0 || factor*0 !=0" << std::endl;
	 return false;
    }
   
    xtBJ = 2*x.transpose()*B*Jest;

    for(unsigned int i=0; i<6; i++) {
	TMP1 = x.transpose()*B*Zest.block<3,3>(0,3*i)*B;
	xtBZBx(i) = -TMP1*x;
	xtBZBJ.row(i) = -TMP1*Jest;
    }
    Q = xtBJ+xtBZBx;
   
    factor = lfd1*lfd2*factor; 
    score_gradient += Q*factor;
    
    
    for(unsigned int i=0; i<6; i++) {
	for(unsigned int j=0; j<6; j++) {
	    xtBH(i,j) = x.transpose()*B*Hest.block<3,1>(3*i,j); 
	    xtBZBZBx(i,j) = x.transpose()*B*Zest.block<3,3>(0,3*i)*B*Zest.block<3,3>(0,3*j)*B*x;
	    xtBZhBx(i,j) = x.transpose()*B*ZHest.block<3,3>(3*j,3*i)*B*x;
	}
    }

    Hessian += 2*factor*(Jest.transpose()*B*Jest - lfd2*Q*Q.transpose()/4 - 
			2*xtBZBJ + xtBH - xtBZBZBx - xtBZhBx/2 );
    
    return true;

}
    
template <typename PointSource, typename PointTarget>
void NDTMatcherD2D<PointSource,PointTarget>::precomputeAngleDerivatives() {
    
    //For Jest
    jest13 << 0, 0, -1; 
    jest23 << 0, 1, 0; 
    jest04 << 0, 0, 1; 
    jest14 << 0, 0, 0; 
    jest24 << -1, 0, 0; 
    jest05 << 0, -1, 0; 
    jest15 << 1, 0, 0; 
    jest25 << 0,0,0;

    //for Hest
    a2 << 0, -1, 0;
    a3 << 0,0,-1;
    b2 << 1,0,0; 
    b3 << 0,0,0; 
    c2 << 0,0,0; 
    c3 << 1,0,0; 
    d1 << -1,0,0;
    d2 << 0,0,0; 
    d3 << 0,0,-1;
    e1 << 0,0,0; 
    e2 << 0,0,0; 
    e3 << 0,1,0; 
    f1 << -1,0,0;
    f2 << 0,-1,0;
    f3 << 0,0,0; 

    //for Zest
    dRdx<<0,0,0,  0,0,1, 0,-1,0;
    dRdy<<0,0,-1, 0,0,0, 1,0,0;
    dRdz<<0,1,0, -1,0,0, 0,0,0 ;

    //for ZHest
    dRdxdx<<0,0,0, 
	    0,-1,0, 
	    0,0,-1 ;

    dRdxdy<<0,0,0, 
	    1,0,0,  
	    0,0,0;

    dRdxdz<<0,0,0, 
	    0,0,0, 
	    1,0,0; 

    dRdydy<<-1,0,0, 
	    0,0,0, 
	    0,0,-1;

    dRdydz<< 0,0,0, 
	    0,0,0, 
	    0,1,0;

    dRdzdz<< -1,0,0, 
	    0,-1,0, 
	    0,0,0; 
    
//    cout<<"XX:"<<dRdxdx.transpose()<<"\nXY:"<<dRdxdy.transpose()<<"\nXZ:"<<dRdxdz.transpose()<<endl;
//    cout<<"YY:"<<dRdydy.transpose()<<"\nYZ:"<<dRdydz.transpose()<<"\nZZ:"<<dRdzdz.transpose()<<endl;
    
}   

/*
void NDTMatcherD2D<PointSource,PointTarget>::precomputeAngleDerivatives(Eigen::Vector3d &eulerAngles) {
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

    //For Jest
    jest13 << (-sx*sz+cx*sy*cz) , (-sx*cz - cx*sy*sz) , (-cx*cy) ; 
    jest23 << (cx*sz+sx*sy*cz) , (cx*cz-sx*sy*sz) , (-sx*cy); 
    jest04 << (-sy*cz) , sy*sz , cy; 
    jest14 << sx*cy*cz , (-sx*cy*sz) , sx*sy; 
    jest24 << (-cx*cy*cz) , cx*cy*sz , (-cx*sy); 
    jest05 << (-cy*sz) , (-cy*cz), 0; 
    jest15 << (cx*cz-sx*sy*sz) , (-cx*sz - sx*sy*cz), 0; 
    jest25 << (sx*cz + cx*sy*sz) ,(cx*sy*cz - sx*sz), 0;

    //for Hest
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

    //for Zest
    dRdx<<0,0,0, sx*sz+cx*cz*sy,cx*sy*sz-cz*sx,cx*cy, cx*sz-cz*sx*sy,-cx*cz-sx*sy*sz,-cy*sx;
    dRdy<<-cz*sy,-sy*sz,-cy,cy*cz*sx, cy*sx*sz, -sx*sy,cx*cy*cz, cx*cy*sz,-cx*sy;
    dRdz<<(-cy*sz),(cy*cz),0, -cx*cz-sx*sy*sz,cz*sx*sy-cx*sz,0, cz*sx-cx*sy*sz,sx*sz+cx*cz*sy,0 ;

//    cout<<"X:"<<dRdx.transpose()<<"\nY:"<<dRdy.transpose()<<"\nZ:"<<dRdz.transpose()<<endl;

    //for ZHest
    dRdxdx<<0,0,0, cx*sz-cz*sx*sy,-cx*cz-sx*sy*sz,-cy*sx, -sx*sz-cx*cz*sy,cz*sx-cx*sy*sz,-cx*cy;
    dRdxdy<<0,0,0, cx*cy*cz,cx*cy*sz,-cx*sy, -cy*cz*sx,-cy*sx*sz,sx*sy;
    dRdxdz<<0,0,0, cz*sx-cx*sy*sz,sx*sz+cx*cz*sy,0, cx*cz+sx*sy*sz,cx*sz-cz*sx*sy,0;
    dRdydy<<-cy*cz,-cy*sz,sy, -cz*sx*sy,-sx*sy*sz,-cy*sx, -cx*cz*sy,-cx*sy*sz,-cx*cy;
    dRdydz<<sy*sz,-cz*sy, 0, -cy*sx*sz,cy*cz*sx,0, -cx*cy*sz,cx*cy*cz,0;
    dRdzdz<<-cy*cz,-cy*sz,0, cx*sz-cz*sx*sy,-cx*cz-sx*sy*sz,0, -sx*sz-cx*cz*sy,cz*sx-cx*sy*sz,0;
    
//    cout<<"XX:"<<dRdxdx.transpose()<<"\nXY:"<<dRdxdy.transpose()<<"\nXZ:"<<dRdxdz.transpose()<<endl;
//    cout<<"YY:"<<dRdydy.transpose()<<"\nYZ:"<<dRdydz.transpose()<<"\nZZ:"<<dRdzdz.transpose()<<endl;
}             
*/

template <typename PointSource, typename PointTarget>
void NDTMatcherD2D<PointSource,PointTarget>::computeDerivatives(PointSource &pt, Eigen::Matrix3d C1, Eigen::Matrix3d R){

    //R not necessary!!
    Eigen::Vector3d x;
    x<<pt.x,pt.y,pt.z;

    Jest(1,3) = -x(2); 
    Jest(2,3) = x(1);
    Jest(0,4) = x(2); 
    Jest(2,4) = -x(0);
    Jest(0,5) = -x(1);
    Jest(1,5) = x(0);

    Eigen::Vector3d a,b,c,d,e,f;
    
    a<<0,-x(1),-x(2); 
    b<<0,x(0),0;
    c<<0,0,x(0);
    d<<-x(0),0,-x(2);
    e<<0,0,x(1);
    f<<-x(0),-x(1),0;

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

    Eigen::Matrix3d CR = C1;
    //Zest
    Zest.block<3,3>(0,9) =  dRdx.transpose()*CR + CR.transpose()*dRdx;
    Zest.block<3,3>(0,12) = dRdy.transpose()*CR + CR.transpose()*dRdy;
    Zest.block<3,3>(0,15) = dRdz.transpose()*CR + CR.transpose()*dRdz;

    //original ones
    ZHest.block<3,3>(9,9) =   dRdxdx.transpose()*CR + 2*dRdx.transpose()*C1*dRdx + CR.transpose()*dRdxdx;
    ZHest.block<3,3>(12,12) = dRdydy.transpose()*CR + 2*dRdy.transpose()*C1*dRdy + CR.transpose()*dRdydy;
    ZHest.block<3,3>(15,15) = dRdzdz.transpose()*CR + 2*dRdz.transpose()*C1*dRdz + CR.transpose()*dRdzdz;
    
    ZHest.block<3,3>(9,12) = dRdxdy.transpose()*CR + dRdy.transpose()*C1*dRdx + dRdx.transpose()*C1*dRdy+ CR.transpose()*dRdxdy;
    ZHest.block<3,3>(9,15) = dRdxdz.transpose()*CR + dRdz.transpose()*C1*dRdx + dRdx.transpose()*C1*dRdz+ CR.transpose()*dRdxdz;
    ZHest.block<3,3>(12,15)= dRdydz.transpose()*CR + dRdz.transpose()*C1*dRdy + dRdy.transpose()*C1*dRdz+ CR.transpose()*dRdydz;
    
    ZHest.block<3,3>(12,9) =    ZHest.block<3,3>(9,12); 
    ZHest.block<3,3>(15,9) =    ZHest.block<3,3>(9,15); 
    ZHest.block<3,3>(15,11)=    ZHest.block<3,3>(12,15);
}

template <typename PointSource, typename PointTarget>
double NDTMatcherD2D<PointSource,PointTarget>::scoreNDT(std::vector<NDTCell<PointSource>*> &sourceNDT, NDTMap<PointTarget> &targetNDT,
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T)
{
    
    NUMBER_OF_ACTIVE_CELLS = 0;
    double score_here = 0;
    double det = 0;
    bool exists = false;
    NDTCell<PointTarget> *cell;
    Eigen::Matrix3d covCombined, icov, R;
    Eigen::Vector3d meanFixed;
    Eigen::Vector3d meanMoving;
    PointTarget point;

    R = T.rotation();
    //cout << "scoreNDT : sourceNDT.size() : " << sourceNDT.size() << endl;
    for(unsigned int i=0; i<sourceNDT.size(); i++) {
	meanMoving = T*sourceNDT[i]->getMean();
	point.x = meanMoving(0); point.y = meanMoving(1); point.z = meanMoving(2);	

	//vector<NDTCell*> cells = targetNDT.getCellsForPoint(point,current_resolution);
	//for( int j=0; j<cells.size(); j++) {
	//    cell = cells[j];
	//
	if(!targetNDT.getCellForPoint(point,cell)) {
	    continue;
	}
	{
	    if(cell == NULL) {
		continue;
	    }
	    if(cell->hasGaussian_) {
		meanFixed = cell->getMean();
		covCombined = cell->getCov() + R.transpose()*sourceNDT[i]->getCov()*R;
		covCombined.computeInverseAndDetWithCheck(icov,det,exists);
		if(!exists) continue;
		double l = (meanMoving-meanFixed).dot(icov*(meanMoving-meanFixed));
		if(l*0 != 0) continue;
		if(l > 120) continue;

		double sh = -lfd1*(exp(-lfd2*l/2));

		if(fabsf(sh) > 1e-10) {
		    NUMBER_OF_ACTIVE_CELLS++;
		}
		score_here += sh;
		//score_here += l;
	    }
	}
    }
    return score_here;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Added by Jari - Uses 1-p as a score instead of -p
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
double NDTMatcherD2D<PointSource,PointTarget>::scoreNDTPositive(std::vector<NDTCell<PointSource>*> &sourceNDT, NDTMap<PointTarget> &targetNDT,
		Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T)
{
	
	NUMBER_OF_ACTIVE_CELLS = 0;
	double score_here = 0;
	double det = 0;
	bool exists = false;
	NDTCell<PointTarget> *cell;
	Eigen::Matrix3d covCombined, icov, R;
	Eigen::Vector3d meanFixed;
	Eigen::Vector3d meanMoving;
	PointTarget point;

	R = T.rotation();
	//cout << "scoreNDT : sourceNDT.size() : " << sourceNDT.size() << endl;
	for(unsigned int i=0; i<sourceNDT.size(); i++) {			
		meanMoving = T*sourceNDT[i]->getMean();
		point.x = meanMoving(0); point.y = meanMoving(1); point.z = meanMoving(2);	
		
		if(!targetNDT.getCellForPoint(point,cell)) {
				score_here += 0.1;
				continue;
		}
		
		if(cell == NULL) {
			score_here += 0.1;
			continue;
		}
		if(cell->hasGaussian_) {
			meanFixed = cell->getMean();
			covCombined = cell->getCov() + R.transpose()*sourceNDT[i]->getCov()*R;
			covCombined.computeInverseAndDetWithCheck(icov,det,exists);
			if(!exists){
				score_here+=0.1;
				continue;
			}
			double l = (meanMoving-meanFixed).dot(icov*(meanMoving-meanFixed));
			if(l*0 != 0){
				score_here+=0.1;
				continue;
			}
			if(l > 120){
				score_here+=0.1;
				continue;
			}

			double sh = lfd1*(exp(-lfd2*l/2));

			if(fabsf(sh) > 1e-10) {
					NUMBER_OF_ACTIVE_CELLS++;
			}
			score_here += (1.0 - sh);
			//fprintf(stderr," '[i]=%lf' ", (1.0-sh));
			}else{
				score_here+=0.1;
			}
		
	}
	return score_here;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





template <typename PointSource, typename PointTarget>
//compute the score gradient of a point cloud + transformation to an NDT
void NDTMatcherD2D<PointSource,PointTarget>::derivativesNDT(
	std::vector<NDTCell<PointSource>*> &sourceNDT,
	NDTMap<PointTarget> &targetNDT,
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
	Eigen::Matrix<double,6,1> &score_gradient,
	Eigen::Matrix<double,6,6> &Hessian,
	bool computeHessian
	) {
    
	NDTCell<PointTarget> *cell;
	Eigen::Vector3d transformed;
	Eigen::Vector3d meanMoving, meanFixed;
	Eigen::Matrix3d CMoving, CFixed, Cinv, R;
	bool exists = false;
	double det = 0;

	R = transform.rotation();
/*
	Jest.setZero();
	Jest.block<3,3>(0,0).setIdentity();
	Hest.setZero();
	Zest.setZero();
	ZHest.setZero();
*/

	PointTarget point;
	score_gradient.setZero();
	Hessian.setZero();

	for(unsigned int i=0; i<sourceNDT.size(); i++) {
	    meanMoving = transform*sourceNDT[i]->getMean();
	    point.x = meanMoving(0); point.y = meanMoving(1); point.z = meanMoving(2);	
	    transformed = meanMoving;

	    //vector<NDTCell*> cells = targetNDT.getCellsForPoint(point,current_resolution);
	    //for( int j=0; j<cells.size(); j++) {
	    //    cell = cells[j];
	    //

	    if(!targetNDT.getCellForPoint(point,cell)) 
	    {
		 //std::cout << "!targetNDT.getCellForPoint(point,cell))" << std::endl;
		 continue;
	    }
	    {

		if(cell == NULL) {
		     //std::cout << "cell == NULL" << std::endl;
		    continue;
		}
		if(cell->hasGaussian_) {
		    meanFixed = cell->getMean();
		    transformed -= meanFixed;
		    CFixed = cell->getCov(); 
		    CMoving= R.transpose()*sourceNDT[i]->getCov()*R;

		    (CFixed+CMoving).computeInverseAndDetWithCheck(Cinv,det,exists);
		    if(!exists) 
		    {
			 //std::cout << "computeInverseAndDet... !exist" << std::endl;
			 continue;
		    }
		    //compute Jest, Hest, Zest, ZHest
		    computeDerivatives(point, CMoving, R);

		    //update score gradient
		    if(!update_gradient_hessian(score_gradient, Hessian, transformed, Cinv)) {
			 //std::cout << "!update_gradient_hessian" << std::endl;
			 continue;
		    }

		    cell = NULL;
		}
		else
		{
		     //std::cout << "had not gaussian!" << std::endl;
		}
	    }
	}
}	
   /* 
template <typename PointSource, typename PointTarget>
void NDTMatcherD2D<PointSource,PointTarget>::gradient_numeric( 
	    std::vector<NDTCell*> &moving,
	    NDTMap &fixed,
	    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
	    Eigen::Matrix<double,6,1> &score_gradient
	    ) {

	double epsilon = 10e-2;
	double h, score, scoreNew;
	volatile double zeta;

	//cout << "gradient_numeric : moving.size() : " << moving.size() << endl;
    

	//compute value at the current pose (transform)
	score = scoreNDT(moving, fixed);

//	cout<<"init gradient... score = "<<score<<endl;

	//for each of the components of the pose vector (transform)
	Eigen::Matrix<double,6,1> pose, poseP;
	score_gradient.setZero();
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> transformNew;
	std::vector<NDTCell*> movingN;
	//construct pose from transform
	pose.block<3,1>(0,0) = transform.translation();
	pose.block<3,1>(3,0) = transform.rotation().eulerAngles(0,1,2);
	
//	cout<<"Pose init: "<<pose.transpose()<<endl;

	for(unsigned int i=0; i<pose.rows(); i++) {
	    //compute the delta
	    //compute value of function
	    double x = pose(i);
	    h = sqrt(epsilon)*(x+epsilon);
	    zeta = x+h;
	    h = zeta-x;

	    poseP = pose;
	    poseP(i) += h;
	    
	    //construct transformNew from the new pose
	    transformNew.setIdentity();
	    transformNew =  Eigen::Translation<double,3>(poseP(0),poseP(1),poseP(2))*
		Eigen::AngleAxis<double>(poseP(3),Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(poseP(4),Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(poseP(5),Eigen::Vector3d::UnitZ()) ;

	    //transform moving
	    for(unsigned int q=0; q<movingN.size(); q++) {
		delete movingN[q];
	    }
	    movingN.clear();
	    for(unsigned int q=0; q<moving.size(); q++) {
		NDTCell *cell = moving[q];
		if(cell!=NULL) {
		    if(cell->hasGaussian) {
			Eigen::Vector3d mean = cell->getMean();
			Eigen::Matrix3d cov = cell->getCov();
			mean = transformNew*mean;
			cov = transformNew.rotation().transpose()*cov*transformNew.rotation();
			NDTCell* nd = (NDTCell*)cell->clone();
			nd->setMean(mean);
			nd->setCov(cov);
			movingN.push_back(nd);
		    }
		} 
	    }

	    scoreNew = scoreNDT(movingN, fixed);

//	    cout<<"sg:i: "<<i<<" pose "<<poseP.transpose()<<" scoreold "<<score<<" scorenew "<<scoreNew<<" diff "<<(score-scoreNew)<<endl;
//	    cout<<"scoreNew "<<scoreNew<<endl;

	    score_gradient(i) = (score-scoreNew)/h;
	}

}*/

//perform line search to find the best descent rate (More&Thuente)
template <typename PointSource, typename PointTarget>
double NDTMatcherD2D<PointSource,PointTarget>::lineSearchMT(  Eigen::Matrix<double,6,1> &score_gradient_init,
	Eigen::Matrix<double,6,1> &increment,
	std::vector<NDTCell<PointSource>*> &sourceNDT,
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &globalT,
	NDTMap<PointTarget> &targetNDT) {

  // default params
  double stp = 4.0; //default step
  double recoverystep = 0.05;
  double dginit = 0.0;
  double ftol = 0.0001; //epsilon 1
  double gtol = 0.9999; //epsilon 2
  double stpmax = 20.0;
  double stpmin = 0.01;
  int maxfev = 10; //max function evaluations
  double xtol = 0.01; //window of uncertainty around the optimal step

  double direction = 1.0;
  //my temporary variables
  //std::vector<NDTCell<PointSource>*> sourceNDTHere;
  /*
  for(unsigned int i=0; i<sourceNDT.size(); i++) {
      NDTCell<PointSource> *cell = sourceNDT[i];
      if(cell!=NULL) {
	  //Eigen::Vector3d mean = cell->getMean();
	  //Eigen::Matrix3d cov = cell->getCov();
	  NDTCell<PointSource>* nd = (NDTCell<PointSource>*)cell->copy();
	  //nd->setMean(mean);
	  //nd->setCov(cov);
	  sourceNDTHere.push_back(nd);
      } 
  }*/

  double score_init = 0.0;
  
  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ps;
  ps.setIdentity();
  Eigen::Matrix<double,6,1> pincr, score_gradient_here;
  Eigen::Matrix<double,6,6> pseudoH;
  Eigen::Vector3d eulerAngles;
  /////

  int info = 0;			// return code
  int infoc = 1;		// return code for subroutine cstep

  // Compute the initial gradient in the search direction and check
  // that s is a descent direction.
  
  //we want to maximize s, so we should minimize -s
  score_init = scoreNDT(sourceNDT,targetNDT,ps);
  
  //gradient directions are opposite for the negated function
  //score_gradient_init = -score_gradient_init;

//  cout<<"score_init "<<score_init<<endl;
//  cout<<"score_gradient_init "<<score_gradient_init.transpose()<<endl;
//  cout<<"increment "<<increment.transpose()<<endl;

  dginit = increment.dot(score_gradient_init);
//  cout<<"dginit "<<dginit<<endl;
    
  if (dginit >= 0.0) 
  {
    //std::cout << "MoreThuente::cvsrch - wrong direction (dginit = " << dginit << ")" << std::endl;
    //return recoverystep; //TODO TSV -1; //
    //return -1;

    increment = -increment;
    dginit = -dginit;
    direction = -1;

    if (dginit >= 0.0) 
    {
//    cout << "MoreThuente::cvsrch - Non-descent direction (dginit = " << dginit << ")" << endl;
    //stp = recoverystep;
    //newgrp.computeX(oldgrp, dir, stp);
    /*
    for(unsigned int i=0; i<sourceNDTHere.size(); i++) {
	if(sourceNDTHere[i]!=NULL)
	    delete sourceNDTHere[i];
    }
    */
    return recoverystep;
    }
  } else {
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

   /* 
    for(unsigned int i=0; i<sourceNDTHere.size(); i++) {
	if(sourceNDTHere[i]!=NULL)
	    delete sourceNDTHere[i];
    }
    //transform targetNDT to it's new position
    sourceNDTHere.clear();
    for(unsigned int i=0; i<sourceNDT.size(); i++) {
	NDTCell<PointSource> *cell = sourceNDT[i];
	if(cell!=NULL) {
	    Eigen::Vector3d mean = cell->getMean();
	    Eigen::Matrix3d cov = cell->getCov();
	    mean = ps*mean;
	    cov = ps.rotation().transpose()*cov*ps.rotation();
	    NDTCell<PointSource>* nd = (NDTCell<PointSource>*)cell->copy();
	    nd->setMean(mean);
	    nd->setCov(cov);
	    sourceNDTHere.push_back(nd);
	} 
    } */

    double f = 0.0;
    f = scoreNDT(sourceNDT,targetNDT,ps);
    score_gradient_here.setZero();
    
    //TSV: chaaaange!
    //ps2.setIdentity();
    //derivativesNDT(sourceNDTHere,targetNDT,ps2,score_gradient_here,pseudoH,false);
    //std::cout<<"scg1  " <<score_gradient_here.transpose()<<std::endl;
    derivativesNDT(sourceNDT,targetNDT,ps,score_gradient_here,pseudoH,false);
    //std::cout<<"scg2  " <<score_gradient_here.transpose()<<std::endl;
    
    //cout<<"incr " <<pincr.transpose()<<endl;
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
      /*for(unsigned int i=0; i<sourceNDTHere.size(); i++) {
	  if(sourceNDTHere[i]!=NULL)
	      delete sourceNDTHere[i];
      }*/
      return stp;

    } // info != 0
    
    // RPP add
    //counter.incrementNumIterations();

    // In the first stage we seek a step for which the modified
    // function has a nonpositive value and nonnegative derivative.

    if (stage1 && (f <= ftest1) && (dg >= MoreThuente::min(ftol, gtol) * dginit)) {
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
int NDTMatcherD2D<PointSource,PointTarget>::MoreThuente::cstep(double& stx, double& fx, double& dx,
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

  else {
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
double NDTMatcherD2D<PointSource,PointTarget>::MoreThuente::min(double a, double b)
{
  return (a < b ? a : b);
}

template <typename PointSource, typename PointTarget>
double NDTMatcherD2D<PointSource,PointTarget>::MoreThuente::max(double a, double b)
{
  return (a > b ? a : b);
}

template <typename PointSource, typename PointTarget>
double NDTMatcherD2D<PointSource,PointTarget>::MoreThuente::absmax(double a, double b, double c)
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
double NDTMatcherD2D<PointSource,PointTarget>::normalizeAngle(double a) {
    //set the angle between -M_PI and M_PI
    return atan2(sin(a), cos(a));

}

/*
template <typename PointSource, typename PointTarget>
void NDTMatcherD2D<PointSource,PointTarget>::generateScoreDebug(const char* out, pcl::PointCloud<pcl::PointXYZ>& fixed, pcl::PointCloud<pcl::PointXYZ>& moving) {

    std::ofstream lg(out,std::ios_base::out);
    int N_LINEAR = 100;
    int N_ROT	 = 100;
    //lfd1 = 1;
    //lfd2 = 1;
  
    cout<<"generating scores...\n"; 
    for(current_resolution = 4; current_resolution>=0.5; current_resolution/=2) {
	cout<<"res "<<current_resolution<<endl;
	double lfc1,lfc2,lfd3;
	double integral, outlier_ratio, support_size;
	integral = 0.1;
	outlier_ratio = 0.3;
	support_size = current_resolution; 
	lfc1 = (1-outlier_ratio)/integral;
	lfc2 = outlier_ratio/pow(support_size,3);
	lfd3 = -log(lfc2);
	lfd1 = -(-log( lfc1 + lfc2 ) - lfd3);
	lfd2 = -log((-log( lfc1 * exp( -0.5 ) + lfc2 ) - lfd3 ) / -lfd1);

	lfd1 = 1;//lfd1;
	lfd2 = 0.05;//0.8*lfd2;

	double lmin=-2, lmax=2, rmin=-M_PI/2, rmax=M_PI/2;
	double lstep = (lmax-lmin)/(N_LINEAR-1);
	double rstep = (rmax-rmin)/(N_ROT-1);
	Eigen::MatrixXd S(6,N_LINEAR);
	Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
	std::vector<NDTCell*> nextNDT;

	LazzyGrid prototype(current_resolution);
	NDTMap ndt( &prototype );
	ndt.loadPointCloud( fixed );
	ndt.computeNDTCells();

	double z=0, r=0, pitch=0, yaw=0;
	int k=0;
	for(double x=lmin; x<lmax; x+=lstep) {
	    T = Eigen::Translation<double,3>(x,0,0);
	    pcl::PointCloud<pcl::PointXYZ> cloud = moving;
	    lslgeneric::transformPointCloudInPlace(T,cloud);
	    NDTMap mov( &prototype );
	    mov.loadPointCloud( cloud );
	    mov.computeNDTCells();
	    T.setIdentity();
	    nextNDT = mov.pseudoTransformNDT(T);
	    
	    S(0,k) = scoreNDT(nextNDT,ndt);
	    for(unsigned int i=0; i<nextNDT.size(); i++) {
		if(nextNDT[i]!=NULL) delete nextNDT[i];
	    }
	    k++;
	}
	k=0;
	for(double x=lmin; x<lmax; x+=lstep) {
	    T = Eigen::Translation<double,3>(0,x,0);
	    pcl::PointCloud<pcl::PointXYZ> cloud = moving;
	    lslgeneric::transformPointCloudInPlace(T,cloud);
	    NDTMap mov( &prototype );
	    mov.loadPointCloud( cloud );
	    mov.computeNDTCells();
	    T.setIdentity();
	    nextNDT = mov.pseudoTransformNDT(T);

	    S(1,k) = scoreNDT(nextNDT,ndt);
	    for(unsigned int i=0; i<nextNDT.size(); i++) {
		if(nextNDT[i]!=NULL) delete nextNDT[i];
	    }
	    k++;
	}
	k=0;
	for(double x=lmin; x<lmax; x+=lstep) {
	    T = Eigen::Translation<double,3>(0,0,x);
	    pcl::PointCloud<pcl::PointXYZ> cloud = moving;
	    lslgeneric::transformPointCloudInPlace(T,cloud);
	    NDTMap mov( &prototype );
	    mov.loadPointCloud( cloud );
	    mov.computeNDTCells();
	    T.setIdentity();
	    nextNDT = mov.pseudoTransformNDT(T);

	    S(2,k) = scoreNDT(nextNDT,ndt);
	    for(unsigned int i=0; i<nextNDT.size(); i++) {
		if(nextNDT[i]!=NULL) delete nextNDT[i];
	    }
	    k++;
	}

	k=0;
	for(double r=rmin; r<rmax; r+=rstep) {
	    T = Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ()) ;
	    pcl::PointCloud<pcl::PointXYZ> cloud = moving;
	    lslgeneric::transformPointCloudInPlace(T,cloud);
	    NDTMap mov( &prototype );
	    mov.loadPointCloud( cloud );
	    mov.computeNDTCells();
	    T.setIdentity();
	    nextNDT = mov.pseudoTransformNDT(T);

	    S(3,k) = scoreNDT(nextNDT,ndt);
	    for(unsigned int i=0; i<nextNDT.size(); i++) {
		if(nextNDT[i]!=NULL) delete nextNDT[i];
	    }
	    k++;
	}
	k=0;
	for(double r=rmin; r<rmax; r+=rstep) {
	    T = Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitZ()) ;
	    pcl::PointCloud<pcl::PointXYZ> cloud = moving;
	    lslgeneric::transformPointCloudInPlace(T,cloud);
	    NDTMap mov( &prototype );
	    mov.loadPointCloud( cloud );
	    mov.computeNDTCells();
	    T.setIdentity();
	    nextNDT = mov.pseudoTransformNDT(T);

	    S(4,k) = scoreNDT(nextNDT,ndt);
	    for(unsigned int i=0; i<nextNDT.size(); i++) {
		if(nextNDT[i]!=NULL) delete nextNDT[i];
	    }
	    k++;
	}
	k=0;
	for(double r=rmin; r<rmax; r+=rstep) {
	    T = Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxis<double>(0,Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitZ()) ;
	    pcl::PointCloud<pcl::PointXYZ> cloud = moving;
	    lslgeneric::transformPointCloudInPlace(T,cloud);
	    NDTMap mov( &prototype );
	    mov.loadPointCloud( cloud );
	    mov.computeNDTCells();
	    T.setIdentity();
	    nextNDT = mov.pseudoTransformNDT(T);

	    S(5,k) = scoreNDT(nextNDT,ndt);
	    for(unsigned int i=0; i<nextNDT.size(); i++) {
		if(nextNDT[i]!=NULL) delete nextNDT[i];
	    }
	    k++;
	}

	lg<<"Sf2f"<<(int)current_resolution<<" = ["<<S<<"];\n";
    }
    lg.close();
    
}*/

}
