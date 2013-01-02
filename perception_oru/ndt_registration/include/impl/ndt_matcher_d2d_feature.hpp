#include <Eigen/Eigen>
namespace lslgeneric
{
/*
template <typename PointSource, typename PointTarget>
bool
NDTMatcherFeatureD2D<PointSource,PointTarget>::match( NDTMap<PointTarget>& targetNDT,
			     NDTMap<PointSource>& sourceNDT,
			     Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T)
{
    Jest.setZero();
    Jest.template block<3,3>(0,0).setIdentity();
    Hest.setZero();
    Zest.setZero();
    ZHest.setZero();

    this->precomputeAngleDerivatives();

    lfd1 = 1;//lfd1/(double)sourceNDT.getMyIndex()->size(); //current_resolution*2.5;
    lfd2 = 0.05; //0.1/current_resolution;
//    cout<<lfd1<<" "<<lfd2<<endl;
    ///////////

    //locals
//    int ITR_MAX = 100000; // Henrik, check if it helsp on really bad initial estimates...
    int ITR_MAX = 1000;
    bool convergence = false;
    double score=0;
    double scoreP=0;
//    double DELTA_SCORE = 10e-4*current_resolution;
    double DELTA_SCORE = 10e-5*this->current_resolution; //Henrik
      //double DELTA_SCORE = 0.0005;
    double NORM_MAX = 4*this->current_resolution, ROT_MAX = M_PI/4; //
//    double alfa = 0.9;
    int itr_ctr = 0;
    double step_size = 1;
    Eigen::Matrix<double,6,1> pose_increment_v, pose_increment_reg_v, score_gradient; //column vectors
    Eigen::Matrix<double,6,6> Hessian;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR;
    Eigen::Vector3d transformed_vec, mean;
    bool ret = true;
    //T.setIdentity();
    TR.setIdentity();

    //std::vector<NDTCell*> nextNDT = sourceNDT.pseudoTransformNDT(TR);
    std::vector<NDTCell<PointSource>*> nextNDT = sourceNDT.pseudoTransformNDT(T);

  //  double scoreInit = scoreNDT(nextNDT,targetNDT);
    while(!convergence) {

	TR.setIdentity();
	this->derivativesNDT(nextNDT,targetNDT,TR,score_gradient,Hessian,true);
	pose_increment_v = Hessian.ldlt().solve(-score_gradient);
        //pose_increment_v = Hessian.svd().solve(-score_gradient);

	double pnorm = sqrt(pose_increment_v(0)*pose_increment_v(0) + pose_increment_v(1)*pose_increment_v(1)
			    +pose_increment_v(2)*pose_increment_v(2));
	//cout << "pnorm : " << pnorm << endl;
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

//	pose_increment_v = pow(alfa,itr_ctr)*pose_increment_v;
	TR.setIdentity();
	TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
	    Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
	    Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

	step_size = this->lineSearchMT(score_gradient,pose_increment_v,nextNDT,TR,targetNDT);
	if(step_size < 0) {
	    cout<<"can't decrease in this direction any more, done \n";
	    return true;
	}
	pose_increment_v = step_size*pose_increment_v;


	TR.setIdentity();
	TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
	    Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
	    Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
	    Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;
	T = TR*T;

	for(unsigned int i=0; i<nextNDT.size(); i++) {
	    if(nextNDT[i]!=NULL)
		delete nextNDT[i];
	}
	nextNDT.clear();
	nextNDT = sourceNDT.pseudoTransformNDT(T);

	scoreP = score;
	score = scoreNDT(nextNDT,targetNDT);

	cout<<"iteration "<<itr_ctr<<" step "<<step_size<<" pose norm "<<(pose_increment_v.norm())<<" score_prev "<<scoreP<<" scoreN "<<score<<endl;

	if(itr_ctr>0) {
	    convergence = ((pose_increment_v.norm()) < DELTA_SCORE);
	}
	if(itr_ctr>ITR_MAX) {
	    convergence = true;
	    ret = false;
	}
	itr_ctr++;
//	cout<<"step size "<<step_size<<endl;
    }

//    cout<<"res "<<current_resolution<<" itr "<<itr_ctr<<endl;
//    double scoreFinal = scoreNDT(nextNDT,targetNDT);
//    cout<<"init "<<scoreInit<<" final "<<scoreFinal<<endl;
    for(unsigned int i=0; i<nextNDT.size(); i++) {
	if(nextNDT[i]!=NULL)
	    delete nextNDT[i];
    }

    this->finalscore = score/NUMBER_OF_ACTIVE_CELLS;
//    this->finalscore = score/(nextNDT.size()+targetNDT.getMyIndex()->size());
    cout<<"T: \n t = "<<T.translation().transpose()<<endl;
    cout<<"r= \n"<<T.rotation()<<endl;
    return ret;
}
*/

template <typename PointSource, typename PointTarget>
bool
NDTMatcherFeatureD2D<PointSource,PointTarget>::covariance( NDTMap<PointTarget>& targetNDT,
        NDTMap<PointSource>& sourceNDT,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
        Eigen::Matrix<double,6,6> &cov)
{
    //...
    assert(false);

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

    pcl::PointXYZ point;
    //now compute Jdpdz
    for(unsigned int j=0; j<_corr.size(); j++)
    {
        unsigned int i = _corr[j].second;
        meanMoving = sourceNDTN[i]->getMean();
        point.x = meanMoving(0);
        point.y = meanMoving(1);
        point.z = meanMoving(2);
        std::cout << "sourceNDTN.size() : " << sourceNDTN.size() << std::endl;
        std::cout << "targetNDTN.size() : " << targetNDTN.size() << std::endl;
        assert(sourceNDTN.size () <= _corr.size());
        cell = targetNDT.getCellIdx(_corr[j].first);

        if(cell == NULL)
        {
            continue;
        }
        if(cell->hasGaussian_)
        {

            meanFixed = cell->getMean();
            transformed = meanMoving-meanFixed;
            CFixed = cell->getCov();
            CMoving= sourceNDTN[i]->getCov();

            (CFixed+CMoving).computeInverseAndDetWithCheck(Cinv,det,exists);
            if(!exists) continue;

            //compute Jdpdz.col(i)
            double factor = (-transformed.dot(Cinv*transformed)/2);
            //these conditions were copied from martin's code
            if(factor < -120)
            {
                //	continue; //Henrik
            }
            factor = exp(lfd2*factor)/2;
            if(factor > 1 || factor < 0 || factor*0 !=0)
            {
                continue;
            }

            Q = -sigmaS*Cinv*Cinv;

            Eigen::Matrix<double,6,1> G, xtQJ;

            G.setZero();
            for(int q=0; q<6; q++)
            {
                G(q) =  -transformed.transpose()*Q*(Zest.template block<3,3>(0,3*q))*Cinv*transformed;
                G(q) = G(q) -transformed.transpose()*Cinv*(Zest.template block<3,3>(0,3*q))*Q*transformed;
            }

            xtQJ = transformed.transpose()*Q*Jest;

            double f1 = (transformed.transpose()*Q*transformed);
            G = G + xtQJ + (-lfd2/2)*f1*ones;
            G = G*factor*lfd1*lfd2/2;

            Jdpdz.row(i) = G.transpose();

            // Check if this is correct...
            for (size_t k = 0; k < _corr.size(); k++)
            {
                if (!_goodCorr[k])
                    continue;

                size_t l = _corr[k].first;
                if(targetNDTN[l]->getMean() == meanFixed)
                {

                    Jdpdz.row(l+sourceNDTN.size()) = Jdpdz.row(l+sourceNDTN.size())+G.transpose();
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

    for(unsigned int q=0; q<sourceNDTN.size(); q++)
    {
        delete sourceNDTN[q];
    }
    sourceNDTN.clear();
    return true;
}

struct sort_scores
{
    bool operator()(const std::pair<unsigned int,double> &left, const std::pair<unsigned int,double> &right)
    {
        return left.second < right.second;
    }
};

template <typename PointSource, typename PointTarget>
double
NDTMatcherFeatureD2D<PointSource,PointTarget>::scoreNDT(std::vector<NDTCell<PointSource>*> &sourceNDT, NDTMap<PointTarget> &targetNDT,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T)
{
    NUMBER_OF_ACTIVE_CELLS = 0;
    double score_here = 0;
    double det = 0;
    bool exists = false;
    NDTCell<PointTarget> *cell;
    Eigen::Matrix3d covCombined, icov;
    Eigen::Vector3d meanFixed;
    Eigen::Vector3d meanMoving;
    Eigen::Matrix3d R = T.rotation();
    std::vector<std::pair<unsigned int, double> > scores;
    for(unsigned int j=0; j<_corr.size(); j++)
    {
        unsigned int i = _corr[j].second;
        if (_corr[j].second >= (int)sourceNDT.size())
        {
            std::cout << "second correspondance : " << _corr[j].second << ", " << sourceNDT.size() << std::endl;
        }
        if (sourceNDT[i] == NULL)
        {
            std::cout << "sourceNDT[i] == NULL!" << std::endl;
        }
        meanMoving = T*sourceNDT[i]->getMean();

        cell = targetNDT.getCellIdx(_corr[j].first);
        {

            if(cell == NULL)
            {
                std::cout << "cell== NULL!!!" << std::endl;
            }
            else
            {
                if(cell->hasGaussian_)
                {
                    meanFixed = cell->getMean();
                    covCombined = cell->getCov() + R.transpose()*sourceNDT[i]->getCov()*R;
                    covCombined.computeInverseAndDetWithCheck(icov,det,exists);
                    if(!exists) continue;
                    double l = (meanMoving-meanFixed).dot(icov*(meanMoving-meanFixed));
                    if(l*0 != 0) continue;
                    if(l > 120) continue;

                    double sh = -lfd1*(exp(-lfd2*l/2));

                    if(fabsf(sh) > 1e-10)
                    {
                        NUMBER_OF_ACTIVE_CELLS++;
                    }
                    scores.push_back(std::pair<unsigned int, double>(j, sh));
                    score_here += sh;
                    //score_here += l;
                }
            }
        }
    }

    if (_trimFactor == 1.)
    {
        return score_here;
    }
    else
    {
        // Determine the score value
        if (scores.empty()) // TODO, this happens(!), why??!??
            return score_here;

        score_here = 0.;
        unsigned int index = static_cast<unsigned int>(_trimFactor * (scores.size() - 1));
        //	std::nth_element (scores.begin(), scores.begin()+index, scores.end(), sort_scores()); //boost::bind(&std::pair<unsigned int, double>::second, _1) < boost::bind(&std::pair<unsigned int, double>::second, _2));
        std::nth_element (scores.begin(), scores.begin()+index, scores.end(), boost::bind(&std::pair<unsigned int, double>::second, _1) < boost::bind(&std::pair<unsigned int, double>::second, _2));
        std::fill(_goodCorr.begin(), _goodCorr.end(), false);
        //	std::cout << "_goodCorr.size() : " << _goodCorr.size() << " scores.size() : " << scores.size() << " index : " << index << std::endl;
        for (unsigned int i = 0; i < _goodCorr.size(); i++)
        {
            if (i <= index)
            {
                score_here += scores[i].second;
                _goodCorr[scores[i].first] = true;
            }
        }
        return score_here;
    }
}

template <typename PointSource, typename PointTarget>
void
NDTMatcherFeatureD2D<PointSource,PointTarget>::derivativesNDT( std::vector<NDTCell<PointSource>*> &sourceNDT,
        NDTMap<PointTarget> &targetNDT,
        Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &transform,
        Eigen::Matrix<double,6,1> &score_gradient,
        Eigen::Matrix<double,6,6> &Hessian,
        bool computeHessian )
{
    NDTCell<PointTarget> *cell;
    Eigen::Vector3d transformed;
    Eigen::Vector3d meanMoving, meanFixed;
    Eigen::Matrix3d CMoving, CFixed, Cinv, R;
    bool exists = false;
    double det = 0;

    R = transform.rotation();

    Jest.setZero();
    (Jest.template block<3,3>(0,0)).setIdentity();
    Hest.setZero();
    Zest.setZero();
    ZHest.setZero();

    PointSource point;
    score_gradient.setZero();
    Hessian.setZero();

    for (unsigned int j = 0; j < _corr.size(); j++)
    {
        if (!_goodCorr[j])
            continue;

        unsigned int i = _corr[j].second;
        if (i >= sourceNDT.size())
        {
            std::cout << "sourceNDT.size() : " << sourceNDT.size() << ", i: " << i << std::endl;
        }
        assert(i < sourceNDT.size());
        meanMoving = transform*sourceNDT[i]->getMean();
        point.x = meanMoving(0);
        point.y = meanMoving(1);
        point.z = meanMoving(2);
        transformed = meanMoving;

        cell = targetNDT.getCellIdx(_corr[j].first);
        {
            if(cell == NULL)
            {
                continue;
            }
            if(cell->hasGaussian_)
            {
                meanFixed = cell->getMean();
                transformed -= meanFixed;
                CFixed = cell->getCov();
                CMoving= R*sourceNDT[i]->getCov()*R.transpose();

                (CFixed+CMoving).computeInverseAndDetWithCheck(Cinv,det,exists);
                if(!exists) continue;

                //compute Jest, Hest, Zest, ZHest
                this->computeDerivatives(meanMoving, CMoving);

                //update score gradient
                if(!update_gradient_hessian(score_gradient, Hessian, transformed, Cinv))
                {
                    continue;
                }

                cell = NULL;
            }
        }
    }
}

template <typename PointSource, typename PointTarget>
bool
NDTMatcherFeatureD2D<PointSource,PointTarget>::update_gradient_hessian(
    Eigen::Matrix<double,6,1> &score_gradient,
    Eigen::Matrix<double,6,6> &Hessian,
    Eigen::Vector3d & x,
    Eigen::Matrix3d & B)
{

    //vars for gradient
    Eigen::Matrix<double,6,1> xtBJ, xtBZBx, Q;
    //vars for hessian
    Eigen::Matrix<double,6,6> JtBJ, xtBZBJ, xtBH, xtBZBZBx, xtBZhBx;
    Eigen::Matrix<double,1,3> TMP1;

    double factor = (-x.dot(B*x)/2);

    //these conditions were copied from martin's code
    // Henrik, these conditions doesn't apply here if the initial estimate is too far away. One option could be to run a closed form solution step (ICP) first to avoid this problem.
    if(factor < -120)
    {
//    	 std::cout << "factor : " << factor << std::endl;
//    	 return false;
    }

    factor = exp(lfd2*factor)/2;
    if(factor > 1 || factor < 0 || factor*0 !=0)
    {
        std::cout << "factor > 1 || factor < 0 || factor*0 !=0" << std::endl;
        return false;
    }

    xtBJ = 2*x.transpose()*B*Jest;

    for(unsigned int i=0; i<6; i++)
    {
        TMP1 = x.transpose()*B*(Zest.template block<3,3>(0,3*i))*B;
        xtBZBx(i) = -TMP1*x;
        xtBZBJ.row(i) = -TMP1*Jest;
    }
    Q = xtBJ+xtBZBx;

    score_gradient += lfd1*lfd2*Q*factor;


    for(unsigned int i=0; i<6; i++)
    {
        for(unsigned int j=0; j<6; j++)
        {
            xtBH(i,j) = x.transpose()*B*(Hest.template block<3,1>(3*i,j));
            xtBZBZBx(i,j) = x.transpose()*B*(Zest.template block<3,3>(0,3*i))*B*(Zest.template block<3,3>(0,3*j))*B*x;
            xtBZhBx(i,j) = x.transpose()*B*(ZHest.template block<3,3>(3*j,3*i))*B*x;
        }
    }

    Hessian += lfd1*lfd2*2*factor*(Jest.transpose()*B*Jest - lfd2*Q*Q.transpose()/4 -
                                   2*xtBZBJ + xtBH - xtBZBZBx - xtBZhBx/2 );

    return true;

}

}
