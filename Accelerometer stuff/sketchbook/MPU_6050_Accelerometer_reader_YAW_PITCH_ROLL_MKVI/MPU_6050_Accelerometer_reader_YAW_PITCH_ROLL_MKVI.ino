

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

ros::NodeHandle  nh;

std_msgs::Float32MultiArray str_msg;
ros::Publisher chatter("chatter", &str_msg);

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprInitial[3];    //initial read of ypr in order to calculate offset
boolean button(0), readOffset(0);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

float hello[3] = {
  9.87654321,1.111111111,2.2222222222};

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
int ledPin = 13;
void setup()
{
  pinMode(ledPin,OUTPUT); 
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz) CHANGE BACK TO 24 IF ANY OTHER VALUE IS BREAKING
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  //--------------------------------------------ROS STUFF---------------
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

  // if programming failed, don't try to do anything
  // if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  //  while (!mpuInterrupt && fifoCount < packetSize) ;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.resetFIFO();//TEST. REMOVE IF STUFF MESSES UP
                    //SHOULD BE FINE.
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //if (readOffset)//Sends stuff through ros now
    //--------------ROS STUFF
    for (int i = 0; i < 3; i++)
    {
      ypr[i]=ypr[i]*180/M_PI;
    }    
    

    digitalWrite(ledPin, !digitalRead(ledPin));
    str_msg.data = ypr;
    chatter.publish( &str_msg );
    nh.spinOnce();
    nh.spinOnce();
    //delay(100);
    for (int i = 0; i < 10; i++)
    {
      str_msg.data = ypr;
      chatter.publish( &str_msg );
      nh.spinOnce();
      nh.spinOnce();
    }
  }
}

