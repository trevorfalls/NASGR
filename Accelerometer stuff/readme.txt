This program measures the orientation of the MPU-9250 and outputs it through serial. 

-------------------CHANGELOG------------------------------------------------
2/26/2016
Created program to read raws from MPU-9250 which include xyz accelerations, gyroscopic accelerations, and magnetometer data. 
No ROS programming integrated yet. Plan on figuring out how to get orientations and velocities before or while integrating ROS.
Program called MPU_9250_Serial_Reader_2

v3.0 uploaded 3 November 2015 
	- V2 and V1 didn't work
V5.0 uploaded 5 November 2015
	-V4 didn't work
V5.0 modified 
	-Tried to fix lost syc issue. It's slightly better, can run for ~12 minutes before stopping.

1/16/2016
V6.0 is up and running. 
	-Must continuously gather data from accelerometer buffer, but pushes data to ROS every 1/10 second. Don't know if it'll interfere with other stuff.
	-If not continuously taking data from accelerometer buffer, accelerometer crashes.

1/18/2016
	-Lowered sampling rate on accelerometer so the Arduino can be doing other things. Before, more of the processing power was needed to keep the FIFO buffer from overflowing.

1/21/2016
	-Included accelerations in data sent through ROS. Started working on a complementary filter, still needs tweaking.
	
4/7/2016
	-Cleaned up a lot of stuff, added bash script to automate launch of various programs.
	-Cleaned up Cmakelist so ros program names match the name needed to launch program. 
	-Commented a lot of stuff
--------------------HOW TO USE---(outdated)---------------------------------
To set up, connect the MPU to an Arduino Mega or Uno, connect VCC to 3.3V, GND to GND, SCL to SCL, SDA to SDA, and INT to digital pin 2.

----------------MUST COMPLETE FUTURE GOALS------------------------------------------

-change toggle to toggle when a certain button from a game controller is pressed
	-Will provide data needed to maintain the robot's orientation
-integrate data to with robot maneuvering systems to maintain the robot's orientation

-----------------TENTATIVE FUTURE GOALS---------------------------------

-make robot follow pilots input more exactly e.g. if going forward, will compensate for drift and will only move forward
