#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>

int i = 0;

void chatterCallback(const std_msgs::Float32MultiArray::Ptr & msg)
{
  std::cout << (*msg).data[0] << " " << (*msg).data[1] << " " << (*msg).data[2] << " " << (*msg).data[3] << " " << (*msg).data[4] << " " << (*msg).data[5] << " " << i++ << '\n';
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
