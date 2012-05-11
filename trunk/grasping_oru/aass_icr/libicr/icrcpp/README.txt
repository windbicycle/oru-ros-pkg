Library: icrcpp.a
Author: Robert Krug 
http://www.aass.oru.se/Research/Learning/rtkg.html

The library was tested under Ubuntu 10.04/11.04 utilizing the GNU gcc-4.4.3/4.5.2 compiler. For newer compilers, where the C++ technical report TR1 is moved to std standard, it might be necessary to change the scoping of the utilized shared pointers and the memory include directive. This can be done by changing the accordingly defined types in icrcpp/include/utilities.h. For loading 3D-models in Wavefront .obj format, the library libobj-0.1 by Ares Lagae is used which is currently available from here: http://people.cs.kuleuven.be/~ares.lagae/libobj/index.html. Convex hull computations are carried out via Qhull 2011.2, provided on Gitorious: http://gitorious.org/qhull/qhull/. Furthermore, Eigen-3.0.2 is necessary: http://bitbucket.org/eigen/eigen/get/3.0.1.tar.bz2. Eigen-3.0.2 is provided in the icrcpp/tools/ directory. It is recommended to use the republished versions of libobj-0.1 and Qhull 2011.2 which are available On GitHub. Further details are given below.

Robert Krug 2011/01/11

Configuration
*************

A number of parameters can be set in the file /icrcpp/include/config.h prior to compilation. By default, the ICR-computation is multithreaded in the number of fingers. For single-core processors the computation is faster when threading is disabled by uncommenting the line #define MULTITHREAD_ICR_COMPUTATION. In the same file, the default finger parameters can be changed. If DIVIDE_OWS_BY_LAMBDA is defined, the wrenches of the Object Wrench Space are divided by the largest occuring torque arm, which grants scale invariance. In /icrcpp/include/debug.h defining following statements writes text files in the icrcpp/debug directory.

DEBUG_OBJECT_LOADER writes the vertices and vertex-normals of the loaded object to 'points.txt' and 'normals.txt' respectively. The neighbor indices of each point are written to 'neighbors.txt'.

DEBUG_OWS writes the genereated wrench cones to 'wrenches.txt'. Note that if multiple different Object Wrench Spaces exist due to differing finger contact models, only the last one gets written.

DEBUG_DISCRETEWRENCHSPACE The hyperplanes describing the convex hull of the Grasp Wrench Space are written to 'hyperplanes.txt'. 

DEBUG_QHULL If Qhull produces an error, which can happen e.g. if a prototype-grasp is not force-closure, enabling this statement results into the according Qhull error message being written to cout.

Installation Instructions
*************************

libobj-0.1: Republished on git@github.com:rtkg/obj-1.0.git, tarball available from https://nodeload.github.com/rtkg/obj-1.0/tarball/master; In the root folder type ./configure, make , sudo make install. Note that in this version in the file /obj-0.1/obj/obj_parser.hpp the original typedef statement before the enum in line 50 was deleted, since it is superfluous and creates an error when compiling with the
-Werr flag.

Qhull 2011.1: Republished on git@github.com:rtkg/Qhull-2011.2.git, tarball available from https://nodeload.github.com/rtkg/Qhull-2011.2/tarball/master; In the root folder type make, sudo make install; Compared to the original version, the compiler optimization flags were changed from -O2 to -O3 which results in some speed-up when computing convex hulls with many input points. Also, in this version qh_QHpointer 1 in /Qhull-2011.2/src/libqhull/user.h was defined. This is necessary to use the C++ interface.    

For more information and clean-up, see the Readme-files of the aforementioned packages. The shared libraries for libobj-0.1 and Qhull 2011.2 are installed by default into /usr/local/lib. This directory needs to be included in the LD_LIBRARY_PATH. In case of different installation directories, The include statements in the /icrcpp/Makefile need to be changed accordingly.

icrcpp: Published on git@github.com:rtkg/icrcpp.git, tarball available from https://nodeload.github.com/rtkg/icrcpp/tarball/master; cd into the root folder and type make. For clean-up type make clean. To generate the documentation, run Doxygen in the /icrcpp/doc folder. An example is provided in/icrcpp/examples/src. To build it, cd into /icrcpp/examples and type make. Run the compiled example with ./ex, clean-up works with make-clean.









