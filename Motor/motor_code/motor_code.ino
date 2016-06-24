//Controls the motors

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo motorFR;
Servo motorFL;
Servo motorBR;
Servo motorBL;
Servo motorFV;
Servo motorBV;
Servo claw;

void pushValues(const std_msgs::UInt16MultiArray& values) {
  motorFR.writeMicroseconds(values.data[0]);
  motorFL.writeMicroseconds(values.data[1]);
  motorBR.writeMicroseconds(values.data[2]);
  motorBL.writeMicroseconds(values.data[3]);
  motorFV.writeMicroseconds(values.data[4]);
  motorBV.writeMicroseconds(values.data[5]);
  claw.writeMicroseconds(values.data[6]);
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("motorValues",&pushValues);

void setup() {
    nh.initNode();
  // put your setup code here, to run once:
  motorFR.attach(10);
  motorFL.attach(11);
  motorBR.attach(5);
  motorBL.attach(3);
  motorFV.attach(9);
  motorBV.attach(2);
  claw.attach(13);
  motorFR.writeMicroseconds(1500);
  motorFL.writeMicroseconds(1500);
  motorBR.writeMicroseconds(1500);
  motorBL.writeMicroseconds(1500);
  motorFV.writeMicroseconds(1500);
  motorBV.writeMicroseconds(1500);
  //claw.writeMicroseconds(1500);
  delay(5000);
  nh.subscribe(sub);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
  delay(10);
}
