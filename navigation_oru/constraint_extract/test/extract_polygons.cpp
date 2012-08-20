#include <constraint_builder.h>

using namespace std;

int main(int argc, char** argv) {

    if(argc!=2) {
	cout<<"usage: \n"<<argv[0]<<" configFile \n";
	return -1;
    }

    ConstraintBuilder cb;

    cb.loadConfigFile(argv[1]);
    cb.processEnvironment();
    cb.savePolygonFile("output.poly");
    cb.savePolygonPicture("output.png");

}
