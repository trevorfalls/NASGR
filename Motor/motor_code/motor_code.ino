//Controls the motors

#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo motorFR;
Servo motorFL;
Servo motorBR;
Servo motorBL;
Servo motorFV;
Servo motorBV;

void pushValues(const std_msgs::UInt8MultiArray& values) {
  motorFR.write(values.data[0]);
  motorFL.write(values.data[1]);
  motorBR.write(values.data[2]);
  motorBL.write(values.data[3]);
  motorFV.write(values.data[4]);
  motorBV.write(values.data[5]);
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("motorValues",&pushValues);

void setup() {
  // put your setup code here, to run once:
  motorFR.attach(9);
  motorFL.attach(10);
  motorBR.attach(11);
  motorBL.attach(12);
  motorFV.attach(13);
  motorBV.attach(14);
  nh.initNode();
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(15);
}
