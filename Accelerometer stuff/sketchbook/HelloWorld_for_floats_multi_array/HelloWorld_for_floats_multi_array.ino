/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;

std_msgs::Float32MultiArray str_msg;
ros::Publisher chatter("chatter", &str_msg);

float hello[3] = {9.87654321,1.111111111,2.2222222222};

void setup()
{
  nh.initNode();
  str_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  str_msg.layout.dim[0].label = "hello";
  str_msg.layout.dim[0].size = 3;
  str_msg.layout.dim[0].stride = 3;
  str_msg.layout.data_offset = 0;
  str_msg.data = (float *)malloc(sizeof(float)*3);
  //str_msg.layout.dim_length = 1;
  str_msg.data_length = 3;
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
