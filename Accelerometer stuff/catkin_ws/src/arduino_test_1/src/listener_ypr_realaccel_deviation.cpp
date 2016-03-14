#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <iomanip> 
#include <time.h>

time_t starttime;
float xaccel[10] = {0,0,0,0,0,0,0,0,0,0};
float yaccel[10] = {0,0,0,0,0,0,0,0,0,0};
float zaccel[10] = {0,0,0,0,0,0,0,0,0,0};
int accelindex = 0;

int i = 0;
float initialData[6] = {0,0,0,0,0,0};
bool initialRun=0;

void chatterCallback(const std_msgs::Float32MultiArray::Ptr & msg)
{

double xaverage = 0;
double yaverage = 0;
double zaverage = 0;

if (!initialRun)
{
	for (int j = 0; j < 6; j++)
	{
		initialData[j] = (*msg).data[j];
	}
	initialRun = 1;
}

xaccel[accelindex]=(initialData[3]-(*msg).data[3])/10.0/16384.0;
yaccel[accelindex]=(initialData[4]-(*msg).data[4])/10.0/16384.0;
zaccel[accelindex]=(initialData[5]-(*msg).data[5])/10.0/16384.0;

accelindex=(accelindex+1)%10;

for (int i = 0; i < 10; i++)
{
	xaverage += xaccel[i];
	yaverage += yaccel[i];
	zaverage += zaccel[i];
}

printf("ypr deviation: %8.3f %8.3f %8.3f   xyzaccel deviation: %9.6f %9.6f %9.6f  %.0f \n", (initialData[0]-(*msg).data[0]), (initialData[1]-(*msg).data[1]), (initialData[2]-(*msg).data[2]), (initialData[3]-(*msg).data[3])/16384.0, (initialData[4]-(*msg).data[4])/16384.0, (initialData[5]-(*msg).data[5])/16384.0, difftime(time(NULL),starttime));

printf("xyzaccel average: %9.6f %9.6f %9.6f\n", xaverage, yaverage, zaverage);

}

int main(int argc, char **argv)
{

time(&starttime);

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();



  return 0;
}
