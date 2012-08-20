#include <constraint_builder.h>
#include <iostream>
#include <highgui.h>

using namespace std;

void ConstraintBuilder::loadConfigFile(const char *configFile) {
    
    FILE *fin = fopen(configFile,"r");
    char *line = NULL;
    bool first = true, second = false, third = false;
    size_t len;
    size_t length = 0;
    std::string image_path;

    double centerX,centerY,resolutionX,resolutionY,xsizeRobot,ysizeRobot;
    State2D pose;

    if(fin == NULL) {
	cout<<"Error reading file "<<configFile<<endl;
	return;
    }

    while(getline(&line,&len,fin) >0 ) {

	//first line: path to map file
	if(first) {
	    first = false; second = true;
	    image_path = std::string(line);
	    *(image_path.rbegin()) = '\0';
	    continue;
	}
	//second line: map params: centerX centerY resolutionX resolutionY
	if(second) {
	    second = false; third = true;
	    sscanf(line,"%lf %lf %lf %lf",&centerX,&centerY,&resolutionX,&resolutionY); 
	    continue;
	}
	//third line: vehicle params: xsize ysize
	if(third) {
	    third = false;
	    sscanf(line,"%lf %lf",&xsizeRobot,&ysizeRobot); 
	    continue;
	}
	//subsequent lines are path points: x y theta_in_radians linear_velocity angular_velocity
	sscanf(line,"%lf %lf %lf %lf %lf",&pose.poseX,&pose.poseY,&pose.poseT,&pose.linearVelocity,&pose.angularVelocity);
	trajectory.push_back(pose);
    }
    map = new GridMap(centerX,centerY,resolutionX,resolutionY);
    map->loadFromDisk(image_path.c_str());
    map->robot.setRobotParams(xsizeRobot,ysizeRobot);

    fclose(fin);
}

void ConstraintBuilder::processEnvironment() {

    //pre-compute allowed positions, max and min thetas
    map->precomputeAllowedPositions(allowedStates,stateIntervals);

    double theta_increment = 2*M_PI / (double) N_THETA_INCREMENTS;
    double len = 0.5;
    cv::Point pixel;
    size_t x,y;

    //go through all poses in the trajectory
    for( size_t i=0; i<trajectory.size(); i++) {
	//create a polygon with a seed set to the trajectory point
	ConvexPolygon poly;
	poly.seed(0) = trajectory[i].poseX;
	poly.seed(1) = trajectory[i].poseY;
	
	map->world2pixel(poly.seed, pixel);
	if(pixel.x < 0 || pixel.y < 0 || pixel.x >= map->sizeY || pixel.y>= map->sizeX) {
	    cout<<"ERROR in polygon creation - trajectory goes out of map at "<<poly.seed.transpose()<<"\n";
	    return;
	}
	x = pixel.y;
	y = pixel.x;
	cout<<"polygon seed at "<<x<<" "<<y<<endl;

	if(!allowedStates(x,y)) {
	    cout<<"ERROR in polygon creation - trajectory goes through obstacle space at "<<poly.seed.transpose()<<"\n";
	    return;
	}

	int idx = trajectory[i].poseT / theta_increment;	
	if(!stateIntervals(x,y).allowed_theta_phi[idx][0]) {
	    cout<<"ERROR in polygon creation - trajectory goes through obstacle space at "
		<<poly.seed.transpose()<<" with theta wrong at "<<idx<<" \n";
	    return;
	}
	size_t idxmin = (idx == 0) ? 0 : idx-1;
	size_t idxmax = (idx == N_THETA_INCREMENTS-1) ? idx : idx+1;
	
	int nMoves = 0;
	vector<pair<size_t,size_t> > best_endpoints, current_endpoints, eps;
	pair<size_t, size_t> ul (x-5,y-5), ll(x-5,y+5), lr(x+5,y+5), ur(x+5,y-5);
	int cl1, cl2;
	double mindist, mindist2, dist;
	Eigen::Vector2d vec;
	size_t xprev, yprev;
	int MAX_XDIST = 10, MAX_YDIST = 10;

	best_endpoints.push_back(ul);
	best_endpoints.push_back(ll);
	best_endpoints.push_back(lr);
	best_endpoints.push_back(ur);
	
	vector<int> indexes_to_consider;
	if(i>=1) {
	    indexes_to_consider.push_back(i-1);
	}
	//search direction towards next
	if(i<trajectory.size()-1) {
	    indexes_to_consider.push_back(i+1);
	}
	
	while (nMoves<50) {
	    current_endpoints = best_endpoints;
	    //search direction towards previous
	    for(int sn =0; sn<indexes_to_consider.size(); sn++) {
		int next = indexes_to_consider[sn];
		vec<<trajectory[next].poseX,trajectory[next].poseY;
		map->world2pixel(vec, pixel);
		xprev = pixel.y;
		yprev = pixel.x;
		//find the two closest points in best endpoints
		mindist=INT_MAX;
		for(int q =0; q<current_endpoints.size(); q++) {
		    dist = sqrt( pow((double)current_endpoints[q].first-xprev,2) 
			         + pow((double)current_endpoints[q].second-yprev,2));
		    //cout<<"q "<<q<<" dist "<<dist<<" to ("<<xprev<<","<<yprev<<")\n";
		    if(dist<mindist) {
			mindist = dist;
			cl1 = q;
		    }	
		}
		int prev_cl1, next_cl1;
		prev_cl1 = cl1 == 0 ? current_endpoints.size()-1 : cl1-1;
		next_cl1 = cl1 == current_endpoints.size()-1 ? 0 : cl1+1;
		double dist1 = sqrt( pow((double)current_endpoints[prev_cl1].first-xprev,2) 
			+ pow((double)current_endpoints[prev_cl1].second-yprev,2));
		double dist2 = sqrt( pow((double)current_endpoints[next_cl1].first-xprev,2) 
			+ pow((double)current_endpoints[next_cl1].second-yprev,2));

		cl2 = dist1 < dist2 ? prev_cl1 : next_cl1; 

		//move them in the direction of xprev yprev
		int dx1,dy1,dx2,dy2,dx3,dy3;
		dx1 = current_endpoints[cl1].first - xprev;// ? -1 : current_endpoints[cl1].first < xprev ?  1 : 0;
		dy1 = current_endpoints[cl1].second - yprev;// ? -1 : current_endpoints[cl1].second < yprev ?  1 : 0;
		dx2 = current_endpoints[cl2].first - xprev;// ? -1 : current_endpoints[cl2].first < xprev ?  1 : 0;
		dy2 = current_endpoints[cl2].second - yprev;// ? -1 : current_endpoints[cl2].second < yprev ?  1 : 0;
		
		dx1 = dx1 > 0 ? -1 : 1;
		dy1 = dy1 > 0 ? -1 : 1;
		dx2 = dx2 > 0 ? -1 : 1;
		dy2 = dy2 > 0 ? -1 : 1;

		current_endpoints[cl1].first += dx1;
		current_endpoints[cl1].second += dy1;
		current_endpoints[cl2].first += dx2;
		current_endpoints[cl2].second += dy2;
//		cout<<"d: "<<dx1<<" "<<dy1<<", "<<dx2<<" "<<dy2<<" eps "<<cl1<<" "<<cl2<<endl;
		dx3 = current_endpoints[cl1].first - current_endpoints[cl2].first;
		dy3 = current_endpoints[cl1].second - current_endpoints[cl2].second;
		
		if(abs(dx3) < MAX_XDIST) {
		    //move away from each other
		    if(dx3 > 0) {
			dx1 =2;
			dx2 =-2;
		    } else {
			dx2 =2;
			dx1 =-2;
		    }
		}
		if(abs(dy3) < MAX_YDIST) {
		    //move away from each other
		    if(dy3 > 0) {
			dy1 =2;
			dy2 =-2;
		    } else {
			dy2 =2;
			dy1 =-2;
		    }
		}
		//cout<<"d: "<<dx1<<" "<<dy1<<", "<<dx2<<" "<<dy2<<" eps "<<cl1<<" "<<cl2<<endl;
		current_endpoints[cl1].first += dx1;
		current_endpoints[cl1].second += dy1;
		current_endpoints[cl2].first += dx2;
		current_endpoints[cl2].second += dy2;

	    }

	    bool doGrow = true;
	    //trace polygon as well
	    for(int q = 1; q<current_endpoints.size(); q++) {
		//cout<<current_endpoints[q-1].first<<" "<<current_endpoints[q-1].second<<" to "<<current_endpoints[q].first<<" "<<current_endpoints[q].second;
		if(!traceLine(current_endpoints[q-1].first,current_endpoints[q-1].second,
			current_endpoints[q].first,current_endpoints[q].second,idxmin,idxmax)) {
		    doGrow = false;
		    break;
		}
		//cout<<" OK\n";
	    }
	    if(!doGrow) break;
	    //cout<<current_endpoints.back().first<<" "<<current_endpoints.back().second<<" to "<<current_endpoints.front().first<<" "<<current_endpoints.front().second;
	    if(!traceLine(current_endpoints.back().first,current_endpoints.back().second,
			current_endpoints.front().first,current_endpoints.front().second,idxmin,idxmax)) {
		break;
	    }
	    //cout<<" OK\n";

	    best_endpoints = current_endpoints;
	    nMoves++;
	}
	 
	//dummy 0.5 m in each direction
/*	Eigen::Vector2d s;
	Eigen::Vector2d ul, ll, lr, ur;
	ul<<-len/2,len/2;
	ll<<-len/2,-len/2;
	lr<<len/2,-len/2;
	ur<<len/2,len/2;
	s = poly.seed + ul;
	poly.endpoints.push_back(s);
	s = poly.seed + ll;
	poly.endpoints.push_back(s);
	s = poly.seed + lr;
	poly.endpoints.push_back(s);
	s = poly.seed + ur;
	poly.endpoints.push_back(s);
*/
	//go through best_endpoints, compute vector position and add in polygon
	for(int q=0; q<best_endpoints.size(); q++) {
	    pixel.x = best_endpoints[q].second;
	    pixel.y = best_endpoints[q].first;
	    map->pixel2world(pixel,vec);
	    poly.endpoints.push_back(vec);
	}
	poly.theta_min = -1;
	poly.theta_max = 1;
	poly.phi_min = -1;
	poly.phi_max = 1;
	poly.v_min = -1;
	poly.v_max = 1;
	poly.w_min = -1;
	poly.w_max = 1;
	poly.calculateMatrixForm();
	polygons.push_back(poly);
	//cout<<"added poly at "<<poly.seed<<endl;
    }

}

void ConstraintBuilder::savePolygonFile(const char*outputFile) {
    for(size_t i = 0; i < polygons.size(); i++) {
	char fname[500];
	snprintf(fname,499,"%s%03d.poly",outputFile,i);
	FILE *fout = fopen(fname,"w");
	polygons[i].writeToPolyFile(fout);
	fclose(fout);
    }	
}

void ConstraintBuilder::savePolygonPicture(const char*outputFile) {
    if(map==NULL) return;
    cv::Mat image;
    map->getCvImage(image);

    if(image.data == NULL) {
	cout<<"probrem!\n";
	return;
    }

    for(size_t i = 0; i < polygons.size(); i++) {
	polygons[i].drawInImage(image,map);
    }	

    cout<<"image size is "<<image.size().width<<"x"<<image.size().height<<endl;
    cv::imwrite(outputFile,image);
}
	
void ConvexPolygon::calculateMatrixForm() {

    //every two adjacent endpoints define a line -> inequality constraint
    //first, resize A and b. x and b are column vectors
    this->A = Eigen::MatrixXd (this->endpoints.size(),2);
    this->b = Eigen::VectorXd (this->endpoints.size());
    Eigen::Vector2d normal;

    for(size_t i=1; i<endpoints.size(); i++) {
	//to define the correct line direction, we also need a point on the inside of the constraint - the seed
	normal(0) = endpoints[i-1](1) - endpoints[i](1);
	normal(1) = endpoints[i](0) - endpoints[i-1](0);
	if(normal.dot(seed) > 0) { //we want the outward pointing normal, so n.dot(s) < 0
	    normal = -normal;
	}
	normal.normalize();
	b(i-1) = -endpoints[i].dot(normal); //endpoints[i];
	A(i-1,0) = normal(0);
	A(i-1,1) = normal(1);
    }
    normal(0) = endpoints.back()(1) - endpoints.front()(1);
    normal(1) = endpoints.front()(0) - endpoints.back()(0);
    if(normal.dot(seed) > 0) { //we want the outward pointing normal, so n.dot(s) < 0
	normal = -normal;
    }
    normal.normalize();
    b(endpoints.size()-1) = -endpoints.front().dot(normal); //endpoints[i];
    A(endpoints.size()-1,0) = normal(0);
    A(endpoints.size()-1,1) = normal(1);
}

void ConvexPolygon::writeToPolyFile(FILE* polyfile) {

    if(polyfile == NULL) return;
    fprintf(polyfile,"#polygon\n");
    fprintf(polyfile,"%d\n",endpoints.size());
    for(size_t i=0; i<endpoints.size(); i++) {
	fprintf(polyfile,"%lf %lf %lf\n", A(i,0), A(i,1), b(i));
    }
    fprintf(polyfile,"%lf %lf\n\n#theta\n%lf %lf\n\n#phi\n%lf %lf\n\n#v\n%lf %lf\n\n#w\n%lf%lf",
	    seed(0),seed(1),theta_min,theta_max,phi_min,phi_max,v_min,v_max,w_min,w_max);

}

void ConvexPolygon::drawInImage(cv::Mat &image, GridMap *map) {
    if (map == NULL) return;
    int npts = endpoints.size();
    cv::Point *pts = new cv::Point[npts];
    cv::Scalar col(1);
    for(size_t i=0; i<npts; i++) {
	map->world2pixel(endpoints[i],pts[i]);
    }
    polylines(image,(const cv::Point**)&pts,(const int*)&npts,1,true,col);

}

bool ConstraintBuilder::traceLine(size_t x1, size_t y1, size_t x2, size_t y2, size_t idx_min, size_t idx_max) {
    size_t x,y;
    x = x1; y = y1;
    int dx, dy;
    while(x!=x2 && y!=y2) {
	if(!allowedStates(x,y)) {
	    //cout<<"PASSES THROUGH OBST\n";
	    return false;
	}
	for(size_t i=idx_min; i<idx_max; i++) {
	    if(!stateIntervals(x,y).allowed_theta_phi[i][0]) {
		//cout<<"PASSES THROUGH WRONG THETA \n";
		return false;
	    }
	}
	/*
	cout<<"xy: "<<x<<" "<<y<<" t ";
	for(size_t i=0; i<N_THETA_INCREMENTS; i++) {
	    cout<<stateIntervals(x,y).allowed_theta_phi[i][0]<<".";
	}
	cout<<endl;
	*/
	dx = x2-x;
	dy = y2-y;
	dx = dx > 0 ? 1 : dx < 0 ? -1 : 0; 
	dy = dy > 0 ? 1 : dy < 0 ? -1 : 0; 
	x = x + dx;
	y = y + dy;
    }	

    return true;
}

bool ConstraintBuilder::isConvex(size_t x1, size_t y1, size_t x2, size_t y2, size_t px, size_t py, size_t cx, size_t cy) {
    //find normal of line from x1,y1 to x2,y2
    int normal_x, normal_y;
    normal_x = y2-y1;
    normal_y = x1-x2;
    return ((px*normal_x + py*normal_y)*(cx*normal_x + cy*normal_y)>0);
}
