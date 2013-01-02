#include <iostream>
#include <boost/bind.hpp>
#include <pangolin/pangolin.h>
#include <ndt_visualisation.h>

using namespace std;
using namespace lslgeneric;
using namespace pangolin;

int main( int argc, char* argv[] )
{
    // NDTVisualiser vis("/home/administrator/ros_workspace/oru_ros_pkg/perception_oru/ndt_map/testdata/LazyGrid.jff");
    // NDTVisualiser vis("/home/administrator/ros_workspace/oru_ros_pkg/perception_oru/ndt_map/testdata/test_wrl.wrl");
    // NDTVisualiser vis("/home/administrator/ros_workspace/oru_ros_pkg/perception_oru/ndt_map/testdata/test_jff.wrl");
    NDTVisualiser vis("/home/administrator/ros_workspace/oru_ros_pkg/perception_oru/ndt_map/testdata/visualiser_test.wrl");
    vis.display();
    return 0;
}
