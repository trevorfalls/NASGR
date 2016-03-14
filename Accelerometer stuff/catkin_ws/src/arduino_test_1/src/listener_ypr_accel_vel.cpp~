#include "ros/ros.h"
//this program shows the current ypr, xyzaccl, xyzvel, and time the program has been running
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <iomanip> 
#include <time.h>

bool initialRun=0;
clock_t t1,t2;
float xyzvel [3] = {0,0,0};
int initialData[3] = {0,0,0};

void chatterCallback(const std_msgs::Float32MultiArray::Ptr & msg)
{
	t2=clock();
	float diff ((float)t2-(float)t1);//gets time the program has been running for
	if (!initialRun)// gets initial reading from accelerometer
	{
		for (int j = 0; j < 6; j++)
		{
			initialData[j] = (*msg).data[j];
		}
		initialRun = 1;
	}
	std::cout << std::setprecision(3) << std::fixed << "ypr: " << (*msg).data[0] << " " << (*msg).data[1] << " " << (*msg).data[2];
	std::cout << std::setprecision(5) << std::fixed << " xyzaccel: " << (*msg).data[3]/2048.0 << " " << (*msg).data[4]/2048.0 << " " << (*msg).data[5]/2048.0 << " ";
	std::cout << std::setprecision(2) << " "/*placeholder for velocity output << '\n'*/;
	std::cout << std::setprecision(5) << std::fixed << ((diff/CLOCKS_PER_SEC)*512)-3 << '\n';
}

int main(int argc, char **argv)
{
t1=clock();
ros::init(argc, argv, "listener");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
ros::spin();
return 0;
}
