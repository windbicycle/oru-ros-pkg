Documentation for "depth_fuse" package.

manifest.xml                  - List of ros package dependencies
CMakeLists.txt                - Cmake configuration file
src/depth_fuse.cpp            - Main function
src/depth_fuse_functions.cpp  - Where all the magic happens

README.txt                    - This file

The Basics
--------------------------------------------------------------------------------
A.  Installation: make sure the directory is added to your ROS_PACKAGE_PATH 

B.  Compilation: rosmake depth_fuse

C.  Running the program: 

From the terminal, 
  
  rosrun depth_fuse depth_fuse
  
alternatively, 
  
  roscd depth_fuse && ./bin/depth_fuse

Currently the program accepts no command-line parameters nor uses any 
configuration files. All parameters are (unfortunately) hard-coded in source.

The depth_fuse node subscribes to messages on the /camera/depth/image topic. 

If the rendering window is in focus, pressing any key halts the program.

The Advanced stuff
--------------------------------------------------------------------------------

D.  Variables in global scope:

1.   Vector6f Pose
Stores the six parameters of an incremental pose transform. This is the variable
over which the optimization tries to minimize the registration error.

2.  Eigen::Matrix4f Transformation
This matrix represents the rigid body transformation from the origin to the
current camera position. The incremental transforms (see D.1.) are used to update
this variable.

3.  cv::Mat depthImage(480,640,CV_32FC1)
This array holds the current depth image after it has been converted from a ROS 
message.

4.  bool validityMask[480][640];
This array contains TRUE or FALSE to indicate where the depth image holds valid
(not NaN) data.

5.  float Wmax
This is the maximum weight used in the weighted update of our representation.

6.  float resolution
This is a value in meters, corresponding to the size of the edge of one voxel.
Voxels are assumed to be cube-shaped.

7.  float Dmax
This is the largest positive distance stored in the truncated signed distance 
function. All values that would have been larger than Dmax are set to Dmax.

8.  float Dmin
This is the largest (norm) negative value stored in the truncated signed distance
function. All values that would have been larger (norm) than Dmin are set to Dmin.

9.  float myGrid[N][N][N];
This grid contains the truncated signed distance function as a 3D grid of distance
values. Distance values are in the range [Dmin, Dmax]. It is initialized as
Dmax.

10. float weightArray[N][N][N];
This grid contains the weights associated with each distance in the grid (see 
D.9). Weights are in the range [0, Wmax]. It is initialized as zero.

11. bool halt
Variable to alert that a key has been pressed and that it's time to exit.

12. bool first_frame
Variable used to indicate that the current frame is the first frame. Only used
once, for obvious reasons.

13. float fx
Focal length (in the horizontal direction) of the depth sensor using a pinhole 
camera model, measured in pixels.

14. float fy
Focal length (in the vertical direction) of the depth sensor using a pinhole 
camera model, measured in pixels.


15. float cx
Center point of the sensor along the horizontal axis, with respect to the image, 
measured in pixels. 

16. float cy
Center point of the sensor along the vertical axis, with respect to the image, 
measured in pixels.

--------------------------------------------------------------------------------

E.  List of Functions:

1.  int main( int argc, char* argv[] )
Initializes variables, subscribes to topics, connects the program to the ROS
messaging system. Listens for messages. When a depth image is received. Function 
E.13 is called.

2.  float SDF(const Eigen::Vector4f &location)
Given a location in 3d (the 4th element of the input is just a scaling term, 
usually 1 and is ignored) the function linearly interpolates between 8 adjacent
distance values stored in myGrid (D.9) and returns the result.

3.  float fastSDF(const Eigen::Vector4f &location)
Alternative to E.2 simply floors the location and returns the value of the
resulting cell in myGrid (D.9).

4.  float objectiveGradient(const Eigen::Vector4f &p, int stepSize, int dim)
Computes the numeric derivative of the SDF along dimension dim, with a step size
of stepSize. This is done by central differences using the absolute value of the 
SDF (to avoid discontinuity issues)

5.  float gradientSDF(const Eigen::Vector4f &p, int dim)
Computes the numeric derivative of the SDF along dimension dim, with a step size
of 1 voxel. This is done by central differences.

6.  float curvatureSDF(const Eigen::Vector4f &p, int dim)
Computes a measure for curvature along dimension dim by taking the inner product
between adjacent gradient vectors (requires 6 calls to one of the above gradient
functions E.5 or E.6).

7.  Eigen::Vector4f To3D(int row, int col, float depth)
Takes the depth value of a pixel, along with it's row and column number. Uses 
the globally defined camera parameters to produce a homogeneous vector (hence 
the 4th element) in 3D-space.

8.  cv::Point2f To2D(const Eigen::Vector4f &droplet, int rows, int cols)
Takes a point in 3D (again in homogeneous coordinates) along with the total 
number of rows and columns in the image. Returns a 2d point in the openCV format.

9.  Eigen::Matrix4f Twist(const Vector6f &xi)
Takes a 6D parameter vector such as the one defined in D.1. and produces a 
matrix containing an antisymmetric block-matrix and a translation vector. The
resulting matrix is directly related to the Transformation in D.2 through an 
exponential mapping.

10. void Render(void)
Visualizes the 3D volume by means of a ray-casting algorithm. Can be adapted
to display the data with different colors. This function listens for keypresses
and sets the halt condition to TRUE if it detects a keypress event.

11. Vector6f EstimatePose(void)
Pose estimation by means of minimizing a point to SDF norm. Returns an
incremental transformation stored as a 6D vector (such as D.1).

12. void exitFunction(void)
If the halt condition is set to TRUE (by pressing a key in the window managed
by Render (E.10)) this function is called before the program exits. Put anything
or nothing at all here. 

13. void FuseDepth(const sensor_msgs::Image::ConstPtr& msg)
This function: 
a.  Receives the sensor message, converts and saves it into the globally 
    defined depthImage (D.3);

b.  If it is not the first frame, i.e., if D.12 is FALSE it calls on EstimatePose 
    (E.11) to estimate an incremental transformation from the last pose to the current;

c.  Updates Transformation (D.2) to store the current pose;

d.  Updates myGrid and weightArray using the depthImage;

e.  Calls Render (E.10);

f.  Returns to main.
