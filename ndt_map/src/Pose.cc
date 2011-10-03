#include <Pose.hh>
#include <cstdio>
#include <Debug.hh>

using namespace std;
using namespace lslgeneric;

/**
  */
Pose3::Pose3() {
    pos<<0,0,0;
    rot.setIdentity();
}

/**
  */
Pose3::Pose3(const Pose3& other) {
    pos = other.pos;
    rot = other.rot;
}

/**
  */
Pose3::Pose3(Eigen::Vector3d &_pos, Eigen::Quaternion<double> &_rot) {
    pos = _pos;
    rot = _rot;
}

/**
  */
Pose3::Pose3(double x, double y, double z, double yaw) {
    pos<<x,y,z;
    rot = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

/**
  */
Pose3 Pose3::from2d(double x, double y, double yaw) {
     return Pose3(x,y,0,yaw);
}

/**
  */
Pose3 Pose3::from2d(const Eigen::Vector3d &v) {
     return Pose3(v(0), v(1), 0, v(2));
}


/// The other is a relative pose towards this.
Pose3
Pose3::add(const Pose3 &other) {
     Pose3 ret;
     ret.pos = this->rot*other.pos+this->pos;
     ret.rot = (this->rot*other.rot).normalized();
     return ret;
}

/// This and the other is given in 'global' cooridinats. The returned value is the relative pose how to reach 'other' from 'this'.
/*
Pose3
Pose3::sub(const Pose3 &other) {

     Pose3 ret;
     Eigen::Quaterniond q;
     ret.rot = this->rot.inverse()*(other.rot);
     q = ret.rot*(Quaternion(this->pos))*(ret.rot.inverse());
     ret.pos = other.pos - q.getIm();

     return ret;

}
*/
/**
Pose3Cov::Pose3Cov() {
    cov = Matrix(6,6);
}

Pose3Cov::Pose3Cov(const Pose3Cov &other) {
    mean = other.mean;
    cov = other.cov;
}

Pose3Cov::Pose3Cov(Pose3 _mean) {
    mean = _mean;
    cov = Matrix(6,6);
}

Pose3Cov::Pose3Cov(Pose3 _mean, Matrix _cov) {
    mean = _mean;
    cov = _cov;
}

  */
void Pose3::writeToVRML(const char* filename) {

    if(filename == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }

    FILE *fout = fopen(filename,"w");
    if(fout == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }
    fprintf(fout,"#VRML V2.0 utf8\n");
    writeToVRML(fout);
    fclose(fout);
}

/**
  */
void Pose3::writeToVRML(FILE* fout) {
    if(fout == NULL) {
	ERR("problem outputing to vrml\n");
	return;
    }
    double BASE_SIZE = 0.6, LENGTH = 1.2;
    //we presenta a pose as a cone, major axis alligned with orientation
    Eigen::Vector3d ori;
    Eigen::Matrix3d tr;
    tr = rot.toRotationMatrix();
    ori = tr.eulerAngles(0,1,2);

    fprintf(fout,"Transform {\n\t\
	    translation %lf %lf %lf \n\t \
	    Transform {\n\
	    rotation 0 0 1 %lf\n\t \
	    Transform {\n\
	    rotation 0 1 0 %lf\n\t\t \
	    Transform { \n\
	    rotation 1 0 0 %lf\n\t\t \
	    Transform { \n\
	    \n\t rotation 0 0 1 -1.5708\
	    \n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Cone { \n\t\t\tbottomRadius %lf \n\t\t\t height %lf }\n",

	    pos(0),pos(1),pos(2),
	    ori(0),ori(1),ori(2),
	    BASE_SIZE, LENGTH
	   );
    //color is fixed for now
    fprintf(fout,"\t\t\tappearance Appearance {\n\t\t\tmaterial Material \
	    { diffuseColor %lf %lf %lf }\n}\n}\n]\n}}}}}\n",
	    1.0,0.0,0.0
	   );
}
/*
Pose3Motion::Pose3Motion() {
}

Pose3Motion::Pose3Motion(const Pose3Motion &other) {
	instantPose		= other.instantPose;
	instantVelocity		= other.instantVelocity;
	instantAngularVelocity	= other.instantAngularVelocity;
	instantAccel		= other.instantAccel;
	instantAngularAccel	= other.instantAngularAccel;
}

Pose3Motion::Pose3Motion(Pose3 iPose) {
	instantPose		= iPose;
}

Pose3Motion::Pose3Motion(Pose3 iPose, Vector3 iVel, Vector3 iAVel) {
	instantPose		= iPose;
	instantVelocity		= iVel;
	instantAngularVelocity	= iAVel;
}

Pose3Motion::Pose3Motion(Pose3 iPose, Vector3 iVel, Vector3 iAVel, 
	Vector3 iAccel, Vector3 iAAccel) {
	
	instantPose		= iPose;
	instantVelocity		= iVel;
	instantAngularVelocity	= iAVel;
	instantAccel		= iAccel;
	instantAngularAccel	= iAAccel;
}
*/
