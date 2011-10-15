#include <PointCloudUtils.hh>
#include <Eigen/Eigen>

using namespace std;
using namespace lslgeneric;

pcl::PointCloud<pcl::PointXYZ> lslgeneric::readVRML(const char* fname) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    FILE *vrml = fopen(fname,"r");

    cloud = lslgeneric::readVRML(vrml);

    if(vrml!= NULL) fclose(vrml);
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ> lslgeneric::readVRML(FILE* vrml) {

    pcl::PointCloud<pcl::PointXYZ> cloud;
    char *line = NULL;
    size_t len;
    bool first=true;
    size_t length = 0;
    if(vrml == NULL) {
	cout<<"couldn't process vrml file\n";
	return cloud;
    }
    
    while(getline(&line,&len,vrml) >0 ) {
	if(first) {
	    //look for 'point [' token
	    char *token = strtok(line," ");
	    while(token!=NULL) {
		if(strncmp("point",token,5)==0) {
		    first=false;
		    break;
		} else {
		    token=strtok(NULL," ");
		}
	    }
	} else {	    
	    //read everything until ]
	    char *token = strtok(line," ");
	    if(strncmp("]",token,1)==0) {
		first=true;
		continue;
	    }
	    pcl::PointXYZ pt;
	    if(token == NULL) continue;
	    pt.x = atof(token);
	    token = strtok(NULL," ");
	    if(token == NULL) continue;
	    pt.y = atof(token);
	    token = strtok(NULL," ");
	    if(token == NULL) continue;
	    pt.z = atof(token);
	    cloud.points.push_back(pt);
	}
    }
    length = cloud.points.size();
    cloud.width = length;
    cloud.height = 1;
    return cloud;
}

///with intensity info from the colors
pcl::PointCloud<pcl::PointXYZI> lslgeneric::readVRMLIntensity(const char* fname) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    FILE *vrml = fopen(fname,"r");

    cloud = lslgeneric::readVRMLIntensity(vrml);

    fclose(vrml);
    return cloud;
}

pcl::PointCloud<pcl::PointXYZI> lslgeneric::readVRMLIntensity(FILE* vrml) {

    pcl::PointCloud<pcl::PointXYZI> cloud;
    char *line = NULL;
    size_t len;
    bool first=true;
    bool second=false;
    int ctr=0;

    size_t length = 0;
    if(vrml == NULL) return cloud;
    
    while(getline(&line,&len,vrml) >0 ) {
	if(first) {
	    //look for 'point [' token
	    char *token = strtok(line," ");
	    while(token!=NULL) {
		if(strncmp("point",token,5)==0) {
		    first=false;
		    second=false;
		    break;
		} else if(strncmp("color",token,5)==0) {
		    first=false;
		    second=true;
		    break;
		} else {
		    token=strtok(NULL," ");
		}
	    }
	} else {	    
	    if(!second) {
		//read everything until ]
		char *token = strtok(line," ");
		if(strncmp("]",token,1)==0) {
		    first=true;
		    continue;
		}
		pcl::PointXYZI pt;
		if(token == NULL) continue;
		pt.x = atof(token);
		token = strtok(NULL," ");
		if(token == NULL) continue;
		pt.y = atof(token);
		token = strtok(NULL," ");
		if(token == NULL) continue;
		pt.z = atof(token);
		cloud.points.push_back(pt);
	    } else {
		//we are at second pass, reading color info
		char *token = strtok(line," ");
		if(strncmp("]",token,1)==0) {
		    first=true;
		    second=false;
		    continue;
		}
		if(strncmp("color",token,5)==0) {
		    continue;
		}
		if(ctr<cloud.points.size()) {
		    if(token == NULL) continue;
		    //red channel, skip
		    token = strtok(NULL," ");
		    if(token == NULL) continue;
		    //green channel = intensity
		    cloud.points[ctr].intensity = atof(token);
		    token = strtok(NULL," ");
		    //blue channel, skip
		    ctr++; 
		} else {
		    //error occured, we are at the wrong place
		    first=true;
		    second=false;
		    continue;

		}
	    }
	}
    }
    length = cloud.points.size();
    cloud.width = length;
    cloud.height = 1;
    return cloud;
}

void lslgeneric::writeToVRML(const char* fname, pcl::PointCloud<pcl::PointXYZI> &pc) {
    FILE *out = fopen(fname,"w");
    fprintf(out,"#VRML V2.0 utf8\n");
    writeToVRML(out,pc);
    fclose(out);
}

void lslgeneric::writeToVRML(FILE* fout, pcl::PointCloud<pcl::PointXYZI> &pc, 
	    Eigen::Vector3d col) {
    fprintf(fout,"Shape {\n geometry PointSet {\n coord Coordinate {\n point [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
	pcl::PointXYZI thisPoint = pc.points[pit];
	if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
	fprintf(fout,"%.5lf %.5lf %.5lf\n", thisPoint.x, thisPoint.y, thisPoint.z);
    }

    fprintf(fout,"]\n}\n color Color {\n color [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
	pcl::PointXYZI thisPoint = pc.points[pit];
	if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
	if(col == Eigen::Vector3d(1,1,1)) {
	    fprintf(fout,"%.5f,%.5f,%.5f\n",thisPoint.intensity, thisPoint.intensity, thisPoint.intensity);
	} else {
	    fprintf(fout,"%.2f,%.2f,%.2f\n",col(0),col(1),col(2));
	}
    }
    fprintf(fout,"]\n }\n }\n }\n");

}

void lslgeneric::writeToVRML(const char* fname, pcl::PointCloud<pcl::PointXYZ> &pc, Eigen::Vector3d col) {
    FILE *out = fopen(fname,"w");
    fprintf(out,"#VRML V2.0 utf8\n");
    writeToVRML(out,pc,col);
    fclose(out);
}

void lslgeneric::writeToVRML(FILE* fout, pcl::PointCloud<pcl::PointXYZ> &pc, Eigen::Vector3d col) {
    fprintf(fout,"Shape {\n geometry PointSet {\n coord Coordinate {\n point [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
	pcl::PointXYZ thisPoint = pc.points[pit];
	if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
	fprintf(fout,"%.5lf %.5lf %.5lf\n", thisPoint.x, thisPoint.y, thisPoint.z);
    }

    fprintf(fout,"]\n}\n color Color {\n color [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
	pcl::PointXYZ thisPoint = pc.points[pit];
	if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
	fprintf(fout,"%.2f,%.2f,%.2f\n",col(0),col(1),col(2));
    }
    fprintf(fout,"]\n }\n }\n }\n");

}


pcl::PointCloud<pcl::PointXYZ> lslgeneric::transformPointCloud(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tr, const pcl::PointCloud<pcl::PointXYZ> &pc){
    Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> T = Tr.cast<float>();
    pcl::PointCloud<pcl::PointXYZ> cloud;  
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
	pcl::PointXYZ thisPoint = pc.points[pit];
	Eigen::Map<Eigen::Vector3f> pt((float*)&thisPoint,3);
	pt = T*pt;
	cloud.points.push_back(thisPoint);
    }
    cloud.width = pc.width;
    cloud.height = pc.height;
    return cloud; 
}

pcl::PointCloud<pcl::PointXYZI> lslgeneric::transformPointCloud(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tr, const pcl::PointCloud<pcl::PointXYZI> &pc){
    Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> T = Tr.cast<float>();
    pcl::PointCloud<pcl::PointXYZI> cloud;  
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
	pcl::PointXYZI thisPoint = pc.points[pit];
	Eigen::Map<Eigen::Vector3f> pt((float*)&thisPoint,3);
	pt = T*pt;
	cloud.points.push_back(thisPoint);
    }
    cloud.width = pc.width;
    cloud.height = pc.height;
    return cloud; 
}

void lslgeneric::transformPointCloudInPlace(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tr, pcl::PointCloud<pcl::PointXYZ> &pc) {
    Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> T = Tr.cast<float>();
    for(unsigned int pit=0; pit<pc.points.size(); ++pit) {
	Eigen::Map<Eigen::Vector3f> pt((float*)&pc.points[pit],3);
	pt = T*pt;
    }
}
