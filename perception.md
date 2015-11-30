# Packages #

ndt\_registration - implements 3D-NDT registration point-to-distribution and distribution-to-distribution. More information on the two algorithms can be found in <a href='http://aass.oru.se/Research/Learning/publications/Magnusson_etal_2007-JFR-3D_Scan_Registration_for_Autonomous_Mining_Vehicles.html'> Magnusson et al, JFR 2007</a> and <a href='http://aass.oru.se/Research/Learning/publications/Stoyanov_etal_2012-ICRA_Point_Set_Registration_through_Minimization_of_the_L2_Distance_between_3D-NDT_Models.html'> Stoyanov et al, ICRA 2012</a>. Includes headers, sample test executable and experimental node.

ndt\_feature\_reg - implements local visual feature registration + 3D-NDT refinement, as described in <a href='http://aass.oru.se/Research/Learning/publications/Andreasson_Stoyanov-SPME2012-Real_Time_Registration_of_RGB-D_Data_using_Local_Visual_Features_and_3D-NDT_Registration.html'>Andreasson and Stoyanov, ICRA SPME workshop 2012</a>. Includes library, test executable and ROS node.

ndt\_map - base classes for 3D-NDT model creation.

ndt\_map\_builder - experimental package for 3D-NDT mapping, based on 3d\_ndt registration and graph slam

pointcloud\_vrml - routines for processing vrml files.

# Installation Instructions #

The packages inthis stack have no external dependencies, outside of the standard ROS installation. Simply type rosmake perception\_oru.
