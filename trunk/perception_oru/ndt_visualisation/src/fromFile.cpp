#include <iostream>
#include <boost/bind.hpp>
#include <pangolin/pangolin.h>
#include <ndt_visualisation.h>

using namespace std;
using namespace lslgeneric;

int main( int argc, char* argv[] )
{
    if(argc == 2)
    {
        NDTVisualiser vis(argv[1]);
        vis.display();
    }
    else if(argc == 3)
    {
        NDTVisualiser vis(argv[1], argv[2]);
        vis.display();
    }
    else
    {
        cerr << "[ USAGE ] ./ndt_visualiser file_path <config_file_path>" << endl;
        return -1;
    }
    return 0;
}