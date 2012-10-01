#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef Eigen::Matrix<double,6,1> Vector6d;
extern Vector6d Pose;
Vector6d previousPose;

Eigen::Matrix4d Transformation=Eigen::MatrixXd::Identity(4,4);

extern cv::Mat depthImage;

extern bool validityMask[480][640];

extern const double Wmax;
extern const double resolution;
extern const double Dmax;
extern const double Dmin;

extern float myGrid[N][N][N];
extern float weightArray[N][N][N];

bool halt = false;
bool first_frame = true;
static const float fx = 525.f , fy = 525.f , cx = 319.5 , cy = 239.5;

//------------------------------------------------------------------------------
//SDF--Returns the Signed Distance at the given location------------------------
//------------------------------------------------------------------------------
double SDF(const Eigen::Vector4d &location) 
{
  double i,j,k;
  double x,y,z;
  if(std::isnan(location(0)+location(1)+location(2))) return Dmax;
  
  x = modf(location(0)/resolution + N/2, &i);
  y = modf(location(1)/resolution + N/2, &j);  
  z = modf(location(2)/resolution + N/2, &k);
    
  if(i>=N-1 || j>=N-1 || k>=N-1 || i<0 || j<0 || k<0)return Dmax;

  int I = int(i); int J = int(j);   int K = int(k);
  
  float* N1 = &myGrid[I][J][K];
  float* N2 = &myGrid[I][J+1][K];
  float* N3 = &myGrid[I+1][J][K];
  float* N4 = &myGrid[I+1][J+1][K];

  double a1,a2,b1,b2;
  a1 = double(N1[0]*(1-z)+N1[1]*z);
  a2 = double(N2[0]*(1-z)+N2[1]*z);
  b1 = double(N3[0]*(1-z)+N3[1]*z);
  b2 = double(N4[0]*(1-z)+N4[1]*z);
    
  return double((a1*(1-y)+a2*y)*(1-x) + (b1*(1-y)+b2*y)*x);
}
//------------------------------------------------------------------------------
//Gradient evaluation ----------------------------------------------------------
//------------------------------------------------------------------------------

bool validGradient(const Eigen::Vector4d &location) 
{

 /* The following code takes the floored coordinates of the center voxel and 
 subtracts one from each dimension. It then creates pointers in I, J for the 
 projection of the nodes in the I,J plane and dereferences the pointers at 
 the appropriate K to access all the elements.  

                      X--------X
                    /        / |
                  X--------X   ----X
                  |        |   | / |
              X----        |   X-------X
            /     |        | /       / |
          X-------X--------X-------X   |
          |     /        / |       |   |
          |   X--------X   |       |   |
     J    |   |        |   |       | /
     ^    X----        |   X-------X
     |        |        | / |  |
      --->I   X--------X   |  X
    /             |        | /
   v              X--------X
  K

*/

  double i,j,k;
  modf(location(0)/resolution + N/2, &i);
  modf(location(1)/resolution + N/2, &j);  
  modf(location(2)/resolution + N/2, &k);
  
  if(std::isnan(i) || std::isnan(j) || std::isnan(k)) return false;

  int I = int(i)-1; int J = int(j)-1;   int K = int(k)-1;  
  
  if(I>=N-4 || J>=N-3 || K>=N-3 || I<=1 || J<=1 || K<=1)return false;

  float* W10 = &weightArray[I+1][J+0][K];
  float* W20 = &weightArray[I+2][J+0][K];
 
  float* W01 = &weightArray[I+0][J+1][K];
  float* W11 = &weightArray[I+1][J+1][K];
  float* W21 = &weightArray[I+2][J+1][K];
  float* W31 = &weightArray[I+3][J+1][K];
  
  float* W02 = &weightArray[I+0][J+2][K];
  float* W12 = &weightArray[I+1][J+2][K];
  float* W22 = &weightArray[I+2][J+2][K];
  float* W32 = &weightArray[I+3][J+2][K];

  float* W13 = &weightArray[I+1][J+3][K];
  float* W23 = &weightArray[I+2][J+3][K];

  if( !W10[1]>0.f || !W10[2]>0.f || 
      !W20[1]>0.f || !W20[2]>0.f || 
      
      !W01[1]>0.f || !W01[2]>0.f ||
      !W11[0]>0.f || !W11[1]>0.f || !W11[2]>0.f || !W11[3]>0.f ||
      !W21[0]>0.f || !W21[1]>0.f || !W21[2]>0.f || !W21[3]>0.f ||
      !W31[1]>0.f || !W31[2]>0.f ||
      
      !W02[1]>0.f || !W02[2]>0.f ||
      !W12[0]>0.f || !W12[1]>0.f || !W12[2]>0.f || !W12[3]>0.f ||
      !W22[0]>0.f || !W22[1]>0.f || !W22[2]>0.f || !W22[3]>0.f ||
      !W32[1]>0.f || !W32[2]>0.f ||
      
      !W13[1]>0.f || !W13[2]>0.f ||
      !W23[1]>0.f || !W23[2]>0.f 
      ) return false;
  else return true;
}

double objectiveGradient(const Eigen::Vector4d &p, int stepSize, int dim)
{
  double delta=resolution*stepSize;
  Eigen::Vector4d p_offset = Eigen::Vector4d::Zero();
  p_offset(dim) = delta;

  return ((SDF(p+p_offset)) - (SDF(p-p_offset)))/(2.0*delta);
}

double gradientSDF(const Eigen::Vector4d &p, int dim)
{
  double delta=resolution;
  Eigen::Vector4d p_offset = Eigen::Vector4d::Zero();
  p_offset(dim) = delta;

  return ((SDF(p+p_offset)) - (SDF(p-p_offset)))/(2*delta);
}
//------------------------------------------------------------------------------
//Projection function-----------------------------------------------------------
//------------------------------------------------------------------------------
Eigen::Vector4d To3D(int row, int col, double depth)
{

  Eigen::Vector4d ret(double(col-cx)*depth/(fx),
                      double(row-cy)*depth/(fy),
                      double(depth),
                      1.0f);
  return ret;
}
//------------------------------------------------------------------------------
//Inverse projection function---------------------------------------------------
//------------------------------------------------------------------------------
cv::Point2d To2D(const Eigen::Vector4d &droplet, int rows, int cols)
{
  cv::Point2d pixel(0,0);  

  if(droplet(2) != 0)
  {
     pixel.x = (cx) + droplet(0)/droplet(2)*(fx);
     pixel.y = (cy) + droplet(1)/droplet(2)*(fy);
  }
  
  return pixel;  
}

//------------------------------------------------------------------------------
//Twist:return a skew-symmetric matrix from an input parameter-vector ----------
//------------------------------------------------------------------------------

Eigen::Matrix4d Twist(const Vector6d &xi)
{
  Eigen::Matrix4d M;
  
  M << 0.0  , -xi(2),  xi(1), xi(3),
       xi(2), 0.0   , -xi(0), xi(4),
      -xi(1), xi(0) , 0.0   , xi(5),
       0.0,   0.0   , 0.0   , 0.0  ;
  
  return M;
}

void Render(void){
  int imageWidth=640, imageHeight=480;
  double minStep = resolution;

  cv::Mat img(imageHeight,imageWidth,CV_8UC3);
  const Eigen::Matrix4d expmap = Transformation;
  const Eigen::Vector4d camera = expmap * Eigen::Vector4d(0.0,0.0,0.0,1.0);

  //Rendering loop
  #pragma omp parallel for 
  for(int u = 0; u < imageHeight; ++u)
  {
    for(int v = 0; v < imageWidth; ++v)
    {
      
      const Eigen::Vector4d p = expmap*To3D(u,v,1.0) - camera;
            
      double scaling = validityMask[u][v] ? double(depthImage.ptr<float>(u)[v])*0.9 : 0.2;
      
      double scaling_prev=0;
      int steps=0;
      double D = resolution;

      while(steps<32 && D>=minStep && D<N*resolution)
      { 
        double D_prev = D;
        D = SDF(camera + p*scaling);
     
        if(D < minStep && D_prev > 0 )
        {
          scaling = scaling_prev - (scaling-scaling_prev) * D_prev /
                                   ( D - D_prev);

          Eigen::Vector4d normal_vector = Eigen::Vector4d::Zero();
            
          for(int ii=0; ii<3; ++ii)
          {
            normal_vector(ii) = fabs(gradientSDF(camera + p*scaling,ii));            
          }   
          normal_vector.normalize();
            
        //  uchar color = uchar(floor(diffuse*100));
          img.at<cv::Vec3b>(u,v)[0]=normal_vector(0)*255;
          img.at<cv::Vec3b>(u,v)[1]=normal_vector(1)*255;
          img.at<cv::Vec3b>(u,v)[2]=normal_vector(2)*255;

          break;
        }
        scaling_prev = scaling;
        scaling = scaling + D;  
        ++steps;        
        img.at<cv::Vec3b>(u,v)[0]=uchar(30);
        img.at<cv::Vec3b>(u,v)[1]=uchar(30);
        img.at<cv::Vec3b>(u,v)[2]=uchar(30);
      }//ray
    }//col
  }//row    
 
   cv::imshow("Render", img);
   cv::waitKey(3);
   if(cv::waitKey(1)!=-1) halt=true;
}

//------------------------------------------------------------------------------
//Exit function, last instructions before exit ---------------------------------
//------------------------------------------------------------------------------
void exitFunction(void)
{
  //-add stuff here-//
  ROS_INFO("Closing node.");
  exit(0);
}


//------------------------------------------------------------------------------
//Pose estimation function------------------------------------------------------
//------------------------------------------------------------------------------
Vector6d EstimatePose(void)
{
  Vector6d xi;
  xi<<0.0,0.0,0.0,0.0,0.0,0.0; // + (Pose-previousPose)*0.1;
  Vector6d xi_prev = xi;

  const static double c = 0.02*Dmax;
  
  const static int iterations[3]={12, 8, 2};
  const static int stepSize[3] = {4, 2, 1};

  for(int lvl=0; lvl < 3; ++lvl)
  {
    for(int k=0; k<iterations[lvl]; ++k)
    {

      const Eigen::Matrix4d camToWorld = Twist(xi).exp()*Transformation;
      
      double A00=0.0,A01=0.0,A02=0.0,A03=0.0,A04=0.0,A05=0.0;
      double A10=0.0,A11=0.0,A12=0.0,A13=0.0,A14=0.0,A15=0.0;
      double A20=0.0,A21=0.0,A22=0.0,A23=0.0,A24=0.0,A25=0.0;
      double A30=0.0,A31=0.0,A32=0.0,A33=0.0,A34=0.0,A35=0.0;
      double A40=0.0,A41=0.0,A42=0.0,A43=0.0,A44=0.0,A45=0.0;
      double A50=0.0,A51=0.0,A52=0.0,A53=0.0,A54=0.0,A55=0.0;
      
      double g0=0.0, g1=0.0, g2=0.0, g3=0.0, g4=0.0, g5=0.0;
      
      for(int row=0; row<depthImage.rows-0; row+=stepSize[lvl])
      {          
        #pragma omp parallel for \
        default(shared) \
        reduction(+:g0,g1,g2,g3,g4,g5,A00,A01,A02,A03,A04,A05,A10,A11,A12,A13,A14,A15,A20,A21,A22,A23,A24,A25,A30,A31,A32,A33,A34,A35,A40,A41,A42,A43,A44,A45,A50,A51,A52,A53,A54,A55)
        for(int col=0; col<depthImage.cols-0; col+=stepSize[lvl])
        {
          if(!validityMask[row][col]) continue;
          double depth = double(depthImage.ptr<float>(row)[col]); 
          Eigen::Vector4d currentPoint = camToWorld*To3D(row,col,depth);
          
          if(!validGradient(currentPoint)) continue;
          double D = (SDF(currentPoint));
          double Dabs = fabs(D);
          if(D >= Dmax || D <= Dmin) continue;
          
          //partial derivative of SDF wrt position  
          Eigen::Matrix<double,1,3> dSDF_dx(objectiveGradient(currentPoint,1,0),
                                            objectiveGradient(currentPoint,1,1),
                                            objectiveGradient(currentPoint,1,2) 
                                            );

          //partial derivative of position wrt optimizaiton parameters
          Eigen::Matrix<double,3,6> dx_dxi; 
          dx_dxi << 0, currentPoint(2), -currentPoint(1), 1, 0, 0,
                    -currentPoint(2), 0, currentPoint(0), 0, 1, 0,
                    currentPoint(1), -currentPoint(0), 0, 0, 0, 1;

          //jacobian = derivative of SDF wrt xi (chain rule)
          Eigen::Matrix<double,1,6> J = dSDF_dx*dx_dxi;
          
          //double tukey = (1-(Dabs/c)*(Dabs/c))*(1-(Dabs/c)*(Dabs/c));
          double huber = Dabs < c ? 1.0 : c/Dabs;
          
          //Gauss - Newton approximation to hessian
          Eigen::Matrix<double,6,6> T1 = huber * J.transpose() * J;
          Eigen::Matrix<double,1,6> T2 = huber * J.transpose() * D;
          
          g0 = g0 + T2(0); g1 = g1 + T2(1); g2 = g2 + T2(2);
          g3 = g3 + T2(3); g4 = g4 + T2(4); g5 = g5 + T2(5);
          
          A00+=T1(0,0);A01+=T1(0,1);A02+=T1(0,2);A03+=T1(0,3);A04+=T1(0,4);A05+=T1(0,5);
          A10+=T1(1,0);A11+=T1(1,1);A12+=T1(1,2);A13+=T1(1,3);A14+=T1(1,4);A15+=T1(1,5);
          A20+=T1(2,0);A21+=T1(2,1);A22+=T1(2,2);A23+=T1(2,3);A24+=T1(2,4);A25+=T1(2,5);
          A30+=T1(3,0);A31+=T1(3,1);A32+=T1(3,2);A33+=T1(3,3);A34+=T1(3,4);A35+=T1(3,5);
          A40+=T1(4,0);A41+=T1(4,1);A42+=T1(4,2);A43+=T1(4,3);A44+=T1(4,4);A45+=T1(4,5);
          A50+=T1(5,0);A51+=T1(5,1);A52+=T1(5,2);A53+=T1(5,3);A54+=T1(5,4);A55+=T1(5,5);
        }//col
      }//row
      
      Eigen::Matrix<double,6,6> A;
      A<< A00,A01,A02,A03,A04,A05,
          A10,A11,A12,A13,A14,A15,
          A20,A21,A22,A23,A24,A25,
          A30,A31,A32,A33,A34,A35,
          A40,A41,A42,A43,A44,A45,
          A50,A51,A52,A53,A54,A55;
     double scaling = 1/A.maxCoeff();
      
      Vector6d g;
      g<< g0, g1, g2, g3, g4, g5;
      
      g = g * scaling;
      A = A * scaling;
      
      A = A + (0.01)*Eigen::MatrixXd::Identity(6,6);
      xi = xi - A.ldlt().solve(g);

      Vector6d Change = xi-xi_prev;  
      double Cnorm = Change.norm();
      xi_prev = xi;
      if(Cnorm < 0.0001) break;
    }//k

  }//level
  
  return xi;
}//function

//------------------------------------------------------------------------------
//Depth Fusing Function---------------------------------------------------------
//------------------------------------------------------------------------------
void FuseDepth(const sensor_msgs::Image::ConstPtr& msg)
{
  if(halt) exitFunction();
  
  cv_bridge::CvImageConstPtr bridge;
  try
  {
    bridge = cv_bridge::toCvShare(msg, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform depth image.");
    return;
  }

  depthImage = bridge->image;

  for(int row=0; row<depthImage.rows-0; ++row)
  { 
    const float* Drow = depthImage.ptr<float>(row);
    #pragma omp parallel for 
    for(int col=0; col<depthImage.cols-0; ++col)
    { 
      if(!isnan(Drow[col]) && Drow[col]>0.4)
      {
      validityMask[row][col]=true;
      }else
      {
        validityMask[row][col]=false;
      }
    }
  }

  Vector6d tempPose = Pose;

  if(!first_frame) 
  Pose = EstimatePose();
  else 
  first_frame = false;
  
  previousPose = tempPose;
  Transformation = Twist(Pose).exp()*Transformation;
  Pose = Pose*0;
  
  const Eigen::Matrix4d camToWorld = Transformation.inverse();
  const Eigen::Vector4d camera = camToWorld * Eigen::Vector4d(0.0,0.0,0.0,1.0);
  //Main 3D reconstruction loop
  
  for(int x = 0; x<N; ++x)
  { 
  #pragma omp parallel for 
    for(int y = 0; y<N;++y)
    { 
      float* previousD = &myGrid[x][y][0];
      float* previousW = &weightArray[x][y][0];      
      for(int z = 0; z<N; ++z)
      {           
        //define a ray and point it into the center of a node
        Eigen::Vector4d ray((x-N/2)*resolution, (y-N/2)*resolution , (z-N/2)*resolution, 1);        
        ray = camToWorld*ray;
        if(ray(2)-camera(2) < 0)continue;
        
        
        cv::Point2d uv;
        uv=To2D(ray, depthImage.rows, depthImage.cols );
        
        int j=floor(uv.x);
        int i=floor(uv.y);
              
        
        //if the projected coordinate is within image bounds
        if(i>0 && i<depthImage.rows-1 && j>0 && j <depthImage.cols-1 && validityMask[i][j] &&    
            validityMask[i-1][j] && validityMask[i][j-1])
        {
          const float* Di = depthImage.ptr<float>(i);
          double Eta; 
          const float W=1/((1+Di[j])*(1+Di[j]));
            
            Eta=(double(Di[j])-ray(2));       
            
            if(Eta >= Dmin)// && Eta<Dmax)
            {
              
              double D = std::min(Eta,Dmax);//*copysign(1.0,Eta);*perpendicular
                
              previousD[z] = (previousD[z] * previousW[z] + float(D) * W) /
                      (previousW[z] + W);

              previousW[z] = std::min(previousW[z] + W , float(Wmax));
            }//within visible region                    
        }//within bounds      
      }//z   
    }//y
  }//x
  Render();
  return;
}

