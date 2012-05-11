#include "../../include/icr.h"
#include <iostream>
#include <sys/time.h>
#include <time.h>

using namespace ICR;

int main()
{ 
   //Load a new target object 
   ObjectLoader obj_loader;
   obj_loader.loadObject("../models/beer_can.obj","beer_can");
 
   //Create a list of 5 default finger parameters (default parameters defined in config.h) and a 
   //vector of centerpoint contact id's for the 5-fingered prototype grasp
   FParamList f_parameters;
   FingerParameters parameters;
   for (int i=0;i<5;i++)
     f_parameters.push_back(parameters);
   VectorXui centerpoint_ids(5);
   centerpoint_ids << 1838, 4526, 4362, 1083, 793;

   //Create a prototype grasp and search zones, the parameter alpha is the scale of the largest
   //origin-centered ball contained by the Grasp wrench space of the prototype grasp
   GraspPtr prototype_grasp(new Grasp());
   prototype_grasp->init(f_parameters,obj_loader.getObject(),centerpoint_ids);
   SearchZonesPtr search_zones(new SearchZones(prototype_grasp));
   double alpha=0.5;
   search_zones->computeShiftedSearchZones(alpha);

   //Create and plot the Independent Contact Regions
   IndependentContactRegions icr(search_zones,prototype_grasp);
   icr.computeICR();
   std::cout<<icr;
   //=====================================================================

   //Load a new target object
   obj_loader.loadObject("../models/cup.obj","cup");

   //Modify the centerpoint id's and some of the finger parameters of the prototype-grasp.
   //The third finger utilizies the Multi-Point contact model, patches only contain the center-point
   //and border points
   centerpoint_ids << 850, 305, 2009, 1740, 1701;     
   f_parameters[0].setSoftFingerContact(1,6,0.5,0.5);
   f_parameters[1].setFrictionlessContact(1);
   f_parameters[2].setContactModelType(Multi_Point);
   f_parameters[2].setInclusionRuleFilterPatch(true);

   //Update prototype grasp, search zones and ICR
   prototype_grasp->init(f_parameters,obj_loader.getObject(),centerpoint_ids);
   search_zones->computeShiftedSearchZones(alpha);
   icr.computeICR();
   std::cout<<icr;
  
  // Utility for timing selected parts of the code - uncomment below and put the code to be timed at the marked location
  // struct timeval start, end;
  // double c_time;
  // gettimeofday(&start,0);
  //
  //  Put code to be timed here...
  //  
  // gettimeofday(&end,0);
  // c_time = end.tv_sec - start.tv_sec + 0.000001 * (end.tv_usec - start.tv_usec);
  // std::cout<<"Computation time: "<<c_time<<" s"<<std::endl;

 return 0;
}
