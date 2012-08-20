#include <ndt_visualisation.h>

typedef struct NDT_VISUAL_REPRESENTATION{
	double  loc_x,  // 
			loc_y,  // mean location
			loc_z,  // 

			ori_x,  // 
			ori_y,  // orientation
			ori_z,  // 
			
			sc_x,   //
			sc_y,   // scale
			sc_z,   //
			
			col1_R, //
			col1_G, // classification color
			col1_B, //

			col2_R, //
			col2_G, // distinct cell color
			col2_B; //
} visual_t;

NDTVisualiser::NDTVisualiser() {
	fileFormat_ = ' ';
	aOK = false;
}

NDTVisualiser::NDTVisualiser(const char * filename) {
	aOK = true;
    char * fileFormat = strrchr(filename,'.');

    if(strcmp(fileFormat, ".jff") == 0){
    	fileFormat_ = 'j';
        if(loadJFF(filename) < 0)
        	aOK = false;
    } else if(strcmp(fileFormat, ".wrl") == 0){
    	fileFormat_ = 'v';
        if(loadVRML(filename) < 0)
        	aOK = false;
    } else {
        std::cout << "Invalid file format! Support is only offered for .jff and .wrl\n";
        // exit(1);
    }

}

NDTVisualiser::NDTVisualiser(string filename) {
	aOK = true;
    char * fileFormat = strrchr(filename.c_str(),'.');

    if(strcmp(fileFormat, ".jff") == 0){
    	fileFormat_ = 'j';
        if(loadJFF(filename.c_str()) < 0)
        	aOK = false;
    } else if(strcmp(fileFormat, ".wrl") == 0){
    	fileFormat_ = 'v';
        if(loadVRML(filename.c_str()) < 0)
        	aOK = false;
    } else {
        std::cout << "Invalid file format! Support is only offered for .jff and .wrl\n";
        // exit(1);
    }	
}

int NDTVisualiser::loadJFF(const char * filename){
	NDTMap<pcl::PointXYZ> test(new CellVector<pcl::PointXYZ>(0.2));
	int retval = test.loadFromJFF(filename);
	if(retval == 0){
		NDTMap<pcl::PointXYZ> nd(new CellVector<pcl::PointXYZ>(0.2));
	} else if (retval == -2) {
		NDTMap<pcl::PointXYZ> nd(new OctTree<pcl::PointXYZ>(0.2));
	} else if (retval == -3) {
		NDTMap<pcl::PointXYZ> nd(new LazyGrid<pcl::PointXYZ>(0.2));
	} else {
		cerr << "[ ERROR ] Error loading NDTMap from jff\n";
		return -1;
	}

	double F_ZERO = 10e-5;
	typename SpatialIndex<PointT>::CellVectorItr it = nd.index_->begin();
    while (it != nd.index_->end()) {
		NDTCell<PointT> *cell = dynamic_cast<NDTCell<PointT>*> (*it);
		if(cell!=NULL) {
			visual_t temp;
			temp.loc_x = cell->mean_(0);
			temp.loc_y = cell->mean_(1);
			temp.loc_z = cell->mean_(2);

			//opposite order to get local transforms
			Eigen::Vector3d ori = cell->evecs_.eulerAngles(2,1,0);
			
			// if(isnan(ori(0)) || isnan(ori(1)) || isnan(ori(2))) {
			// 	cerr << "ori is not a number\n";
			// 	return -1;
			// }
			// if(isinf(ori(0)) || isinf(ori(1)) || isinf(ori(2))) {
			// 	cerr << "ori is infinite\n";
			// 	return -1;
			// }

			temp.ori_x = ori(0);
			temp.ori_y = ori(1);
			temp.ori_z = -ori(2);

			// if (sqrt(evals_(0)) < F_ZERO || sqrt(evals_(1)) < F_ZERO || sqrt(evals_(2)) < F_ZERO) {
			// 	cerr << "sqrt(evals) too small\n";
			// 	return -1;
			// }

			temp.sc_x = sqrt(cell->evals_(0));
			temp.sc_y = sqrt(cell->evals_(1));
			temp.sc_z = sqrt(cell->evals_(2));

			Eigen::Vector3d col;
			switch (cell->cl_) {
			    case ROUGH:
				col<<1,0,0;
				break;
			    case HORIZONTAL:
				col<<0,1,0;
				break;
			    case VERTICAL:
				col<<0,1,1;
				break;
			    case INCLINED:
				col<<0,0,1;
				break;
			    default:
				col<<0,0,0;
				break;
			}
			
			temp.col1_R = col(0);
			temp.col1_G = col(1);
			temp.col1_B = col(2);

			temp.col2_R = cell->R;
			temp.col2_G = cell->G;
			temp.col2_B = cell->B;

			data.push_back(temp);

		} else {
			// do nothing
		}
		it++;
	}

	return 0;
}

void loadVRML(const char * filename){
	FILE * file = fopen(filename, "r");


	fclose(file);
	return 0;
}	

void NDTVisualiser::display(){

}






	Eigen::Vector3d ori;
	//opposite order to get local transforms
	ori = evecs_.eulerAngles(2,1,0);
	


	fprintf(fout,"Transform {\n\t");
	//VRML transforms are applied like this:
	//P_new = Trans x Rot x Scale x P_old
	//translation is mean location
	//scale is evals x |evec|%lf %lf %lf\n\t 
	//orientation is from rotation matrix
	//NOTE: scaled 3x for display reasons!
	if(color != Eigen::Vector3d(0,0,0)) {
	    col = color;
	}
	fprintf(fout,"\
		translation %lf %lf %lf \n\t \
		Transform {\n\
		rotation 0 0 1 %lf\n\t \
		Transform {\n\
		rotation 0 1 0 %lf\n\t\t \
		Transform { \n\
		rotation 1 0 0 %lf\n\t\t \
		Transform { \n\
		scale %lf %lf %lf \
		\n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Sphere {radius 1}\n",
    	        //\n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Box {size 1 1 1}\n",

		mean_(0),mean_(1),mean_(2),
		ori(0),ori(1),-ori(2),
		//3*sqrt(evals(0)),3*sqrt(evals(1)),3*sqrt(evals(2))
		sqrt(evals_(0)),sqrt(evals_(1)),sqrt(evals_(2))
	       );
	fprintf(fout,"\t\t\tappearance Appearance {\n\t\t\tmaterial Material \
		{ diffuseColor %lf %lf %lf }\n}\n}\n]\n}}}}}\n", 
		col(0),col(1),col(2)
	       );
