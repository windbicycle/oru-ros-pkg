#include <IntensityDetection.hh>

using namespace lslgeneric;
using namespace lsl_id;
using namespace std;

void lsl_id::thresholdIntensities(pcl::PointCloud<pcl::PointXYZI> &testCloud,
	pcl::PointCloud<pcl::PointXYZI> &thresholdedCloud) {

    double T_HIGH, T_LOW;

    T_HIGH = 9.5/32.;  //10k is a good raw indicator
    T_LOW = 2./320.;   //so is stuff <200

    for(int i=0; i<testCloud.points.size(); i++) {
	pcl::PointXYZI pt = testCloud.points[i];
	if(pt.intensity < T_LOW || pt.intensity > T_HIGH) {
	    thresholdedCloud.points.push_back(pt);
	    //cout<<pt.intensity<<endl;
	} 
    }

    cout<<"original points "<<testCloud.points.size()<<endl;
    cout<<"thresholded points "<<thresholdedCloud.points.size()<<endl;

    lslgeneric::writeToVRML("ndt_tmp/thresholded.wrl",thresholdedCloud);

}

void lsl_id::clusterIntensityOutput(pcl::PointCloud<pcl::PointXYZI> &pc,
	pcl::PointCloud<pcl::PointXYZI> &map,
	std::string fn) {

    //write out the reference cloud//
    FILE *fout = fopen("ndt_tmp/intensity_clusters.wrl","w");
    fprintf(fout,"#VRML V2.0 utf8\n");
    lslgeneric::writeToVRML(fout,map);

    //cluster points//
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > pcptr(new pcl::PointCloud<pcl::PointXYZI>());
    *pcptr = pc;
    boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZI> > tree(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    tree->setEpsilon(0.1);
    //tree->setInputCloud(pcptr);
    std::vector<pcl::PointIndices> IDX;

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> cl;
    cl.setInputCloud(pcptr);
    cl.setSearchMethod(tree);
    cl.setClusterTolerance(0.5);
    cl.setMinClusterSize(2);
    cl.extract(IDX);

    Eigen::Vector3d centroid, col;
    //go through clusters and output them
    //maybe do some other checks here?
    for(int i =0 ; i<IDX.size(); i++) {
	cout<<"cluster number "<<i<<" contains "<<IDX[i].indices.size()<<" points\n";
	col<<1,0,0;
	centroid<<0,0,0;
	pcl::PointCloud<pcl::PointXYZI> cluster;
	for(int j=0; j<IDX[i].indices.size(); j++) {
	    cluster.push_back(pc.points[IDX[i].indices[j]]);
	    centroid(0) += cluster.points[j].x;
	    centroid(1) += cluster.points[j].y;
	    centroid(2) += cluster.points[j].z;
	}
	
	centroid /= cluster.points.size();
	cout<<"center of cluster at "<<centroid.transpose()<<endl;

	//output points
	lslgeneric::writeToVRML(fout,cluster,Eigen::Vector3d(1,0,0));

	//output sphere centered at centroid
	//VRML transforms are applied like this:
	//P_new = Trans x Rot x Scale x P_old
	//translation is centroid location
	fprintf(fout,"Transform {\n\t \
		translation %lf %lf %lf \n\t \
		\n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Sphere {radius 0.5}\n",
		centroid(0),centroid(1),centroid(2)
	       );
	fprintf(fout,"\t\t\tappearance Appearance {\n\t\t\tmaterial Material \
		{ diffuseColor %lf %lf %lf }\n}\n}\n]\n}\n", 
		col(0),col(1),col(2)
	       );


    }

    fclose(fout);

}
