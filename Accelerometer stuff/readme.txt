This program measures the orientation of the MPU-6050 accelerometer and outputs it through ros. 

-------------------CHANGELOG------------------------------------------------
v3.0 uploaded 3 November 2015 
	- V2 and V1 were horrible failures.
V5.0 uploaded 5 November 2015
	-V4 died like a man dying because of cancer of a tapeworm living inside of him
--------------------HOW TO USE------------------------------------
To set up the connect the MPU-6050 to an Arduino Mega, connect VCC to 3.3V, GND, SCL, SDA, and INT to Digital I/O #2.

Upload the program. Run roscore and the other ros stuff to make stuff visible.

----------------FUTURE GOALS------------------------------------------

-change toggle to toggle when a certain button from a game controller is pressed
	-Will provide data needed to maintain the robot's orientation
-integrate data to with robot maneuvering systems to maintain the robot's orientation
-figure out ROS and publish accelerometer data. Will let PC do the calculations for offset and maintaining robot orientation. (COMPLETED)
