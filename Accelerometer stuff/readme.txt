This program measures the orientation of the MPU-6050 accelerometer and outputs it through ros. 

-------------------CHANGELOG------------------------------------------------
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
--------------------HOW TO USE------------------------------------
To set up the connect the MPU-6050 to an Arduino Mega, connect VCC to 3.3V, GND to GND, SCL to SCL, SDA to SDA, and INT to Digital I/O #2.

Upload the program. Run roscore and the other ros stuff to make stuff visible. Will add more specifics later

----------------MUST COMPLETE FUTURE GOALS------------------------------------------

-change toggle to toggle when a certain button from a game controller is pressed
	-Will provide data needed to maintain the robot's orientation
-integrate data to with robot maneuvering systems to maintain the robot's orientation

-----------------TENTATIVE FUTURE GOALS---------------------------------

-make robot follow pilots input more exactly e.g. if going forward, will compensate for drift and will only move forward
