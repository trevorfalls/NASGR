This program measures the offset of the MPU-6050 accelerometer using serial input. 

-------------------CHANGELOG------------------------------------------------
v3.0 uploaded 3 November 2015 
	- V2 and V1 were horrible failures.
--------------------HOW TO USE------------------------------------
To set up the connect the MPU-6050 to an Arduino Mega, connect VCC to 3.3V, GND, SCL, SDA, and INT to Digital I/O #2.

Upload the program and then open the serial window. A prompt should tell you to enter something through the serial window. The serial window will then show the yaw, pitch, and roll of the accelerometer at any time. The values will stabilize in about 10 seconds. To toggle measuring offset, enter anything to the serial window. 
