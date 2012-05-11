#include "ros/ros.h"
#include "std_msgs/String.h"
#include "icr/finger_tips.h"

#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<icr::finger_tips>("/finger_tips", 1000);

  ros::Rate loop_rate(10);


  int count = 0;
  while (ros::ok())
  {
    std::vector<uint8_t> contact(5,1);
    std::vector<geometry_msgs::Point> points;
    for (uint i=0; i<5;i++) {
      geometry_msgs::Point a;
      a.x = rand()*100;
      a.y = rand()*100;
      a.z = rand()*100;
      points.push_back(a);
    }
    icr::finger_tips msg;
    msg.in_contact = contact;
    msg.points = points;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
