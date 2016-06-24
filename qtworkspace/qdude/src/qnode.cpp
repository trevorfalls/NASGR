/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>
#include <topic_tools/MuxSelect.h>
#include <sensor_msgs/Joy.h>
#include <sstream>
#include "../include/qdude/qnode.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <QDebug>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Implementation
*****************************************************************************/

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 * left Horizontal (lh)= yaw
 * left Vertical (lv)= y
 * right Horizontal (rh)= x
 * right Vertical (rv)= pitch
 * right Trigger (rt)= negative z
 * left trigger (lt)= positive z
 *
 * variables come in between -1 and 1
 *
 * 1          2
 *  /--------\
 *  |        |
 *5 |        | 6
 *  |        |
 *  \--------/
 * 3          4
 *
 * 5 and 6 are vertical thrusters, 5 is the front of the robot
 * 1-4 are horizontal thrusters, angled at 45 degrees around a square in the center of the robot
 */
void calculateVector(float lh, float lv, float rh, float rv, float rt, float lt) {
    //Let's do the easy ones first
    u_int8_t motorOne;
    u_int8_t motorTwo;
    u_int8_t motorThree;
    u_int8_t motorFour;
    u_int8_t motorFive; //0 is full reverse, 255 is full forward
    u_int8_t motorSix;



}

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qdude");
    //qDebug() << "init1";
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    ros::NodeHandle in;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_publisher = n.advertise<std_msgs::String>("/gui_cmd", 1000);
    motorValues_publisher = n.advertise<std_msgs::UInt16MultiArray>("/motorValues",6);
    camToggle_client = n.serviceClient<topic_tools::MuxSelect>("mux_usb_cam/select");
    joy_subscriber = n.subscribe<sensor_msgs::Joy>("joy",10,&QNode::joyCallback,this);
    image_transport::ImageTransport rt_(in);
    it_ = &rt_; //you're going to be confused by this, I tried to do something that didn't work and I didn't remove unused variables
    image_sub_ = it_->subscribe("/displayCam", 1, &QNode::imageCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
    //qDebug() << "init2";
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qdude");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    ros::NodeHandle in;
    // Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_publisher = n.advertise<std_msgs::String>("/gui_cmd", 1000);
    motorValues_publisher = n.advertise<std_msgs::UInt16MultiArray>("/motorValues",6);
    camToggle_client = n.serviceClient<topic_tools::MuxSelect>("mux_usb_cam/select");
    joy_subscriber = n.subscribe<sensor_msgs::Joy>("joy",10,&QNode::joyCallback,this);
    image_transport::ImageTransport rt_(in);
    it_ = &rt_;
    image_sub_ = it_->subscribe("/displayCam", 1, &QNode::imageCallback, this);
	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(1);
	int count = 0;
    //qDebug() << "started to run";
	while ( ros::ok() ) {

        /*std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
        ++count;*/
        ros::spin();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    static int camToggle = 0;
    static int activeCam = 0;
    Q_EMIT buttonAPressed(joy->buttons[0]);
    Q_EMIT buttonBPressed(joy->buttons[1]);
    Q_EMIT buttonXPressed(joy->buttons[2]);
    Q_EMIT buttonYPressed(joy->buttons[3]);
    Q_EMIT leftTrigger(map(joy->axes[5],-1,1,0,100));
    Q_EMIT rightTrigger(map(joy->axes[4],-1,1,0,100));
    Q_EMIT leftControlH(map(joy->axes[0],1,-1,0,100));
    Q_EMIT leftControlV(map(joy->axes[1],-1,1,0,100));
    Q_EMIT rightControlH(map(joy->axes[2],1,-1,0,100));
    Q_EMIT rightControlV(map(joy->axes[3],-1,1,0,100));

    std_msgs::UInt16MultiArray msg;

    //http://diydrones.com/forum/topics/controling-4-rov-thrusters-vectored-configuration-with-arduino?groupUrl=arduboat-user-group&groupId=705844%3AGroup%3A1741386&id=705844%3ATopic%3A2166053&page=1#comments

    int zero=1500;
    float fwdFactor = 1;
    float strafeFactor = 1;
    float yawFactor = 0.14;
    float fwdCmd = map(joy->axes[1],-1,1,-200,200);
    float strafeCmd = map(joy->axes[0],1,-1,-200,200);
    float yawCmd = map(joy->axes[2],1,-1,-200,200);

    if(fwdCmd == 0 && strafeCmd == 0) yawFactor=1;
    
    //input dynamic factor resizing
    //if we are pushing full forward and full yaw, should we let yaw have more precedence?
    //if we are pushing only yaw, should that be higher?
    //this will require a lot of playing around

    u_int16_t fwdRight = zero - fwdFactor*fwdCmd + strafeFactor*strafeCmd + yawFactor*yawCmd;
    u_int16_t fwdLeft = zero - fwdFactor*fwdCmd - strafeFactor*strafeCmd - yawFactor*yawCmd;
    u_int16_t backRight = zero + fwdFactor*fwdCmd + strafeFactor*strafeCmd - yawFactor*yawCmd;
    u_int16_t backLeft = zero + fwdFactor*fwdCmd - strafeFactor*strafeCmd + yawFactor*yawCmd;
    u_int16_t fwdVert = zero;
    u_int16_t backVert = zero;
    /*
    if(joy->axes[5]<1) { //No pitch for now, just to keep things simple and make sure we know values
        backVert = 1500; //1600
        fwdVert = 1500; //1900
    }
    else if(joy->axes[4]<1) {
        backVert = 1500; //1400
        fwdVert = 1500; //1100
    }
    */
    fwdVert = 1500;
    backVert = 1500;
    if(joy->buttons[2])
    {
        fwdVert = 1900;
        backVert = 1600;
    }
    else if(joy->buttons[3])
    {
        fwdVert = 1100;
        backVert = 1400;
    }

    u_int16_t claw = 1500;
    if(joy->buttons[0]) claw = 1400;
    else if(joy->buttons[1]) claw = 1600;

    if(fwdCmd>strafeCmd && strafeCmd >= 0) {
        fwdLeft *= 1.07;
        backLeft *= 1.07;
    }
    else if(fwdCmd<strafeCmd && strafeCmd <= 0) {
        fwdLeft *= 1.07;
        backLeft *= 1.07;
    }

    if(joy->buttons[8]) { //Power button
        fwdRight=zero;
        fwdLeft=zero;
        backRight=zero;
        backLeft=zero;
        fwdVert=zero;
        backVert=zero;
        claw=zero;
    }

    if(yawCmd > 100 || yawCmd < -100 || fwdCmd > 100 || fwdCmd < -100 || strafeCmd > 100 || strafeCmd < -100) {
        /*if(joy->buttons[2] || joy->buttons[3]) {
            fwdRight *= 0.25;
            fwdLeft *= 0.25;
            backRight *= 0.25;
            backLeft *= 0.25;
        }*/
        fwdVert = 1500;
        backVert = 1500;
    }

    /*if(joy->axes[5] > 10) {
        fwdVert = zero + 3*(joy->axes[5]);
    }
    else if(joy->axes[4] > 10) {
        fwdVert = zero - 3*(joy->axes[4]);
    }*/
    if(joy->axes[7]) {
        fwdVert = 1680;
    }
    if(joy->buttons[5]) {
        fwdRight=zero;
        backLeft=zero;
        fwdLeft=1300;
        backRight=1700;
    }

    msg.data.push_back(fwdRight);
    msg.data.push_back(fwdLeft);
    msg.data.push_back(backRight);
    msg.data.push_back(backLeft);
    msg.data.push_back(fwdVert);
    msg.data.push_back(backVert);
    msg.data.push_back(claw);

    motorValues_publisher.publish(msg);

    if(joy->buttons[4]) {
        //image_sub_.shutdown();
        //image_sub_ = it_->subscribe("/usb_cam1/image_raw", 1, &QNode::imageCallback, this);
        camToggle = 1;
    }
    else if(camToggle){
        topic_tools::MuxSelect srv;
        activeCam = (activeCam + 1) % 3;
        if(activeCam==2) {
            srv.request.topic = "usb_cam2/image_raw";
        }
        else if(activeCam==1) {
            srv.request.topic = "usb_cam1/image_raw";
        }
        else if(activeCam==0) {
            srv.request.topic = "usb_cam/image_raw";
        }
        camToggle_client.call(srv);
        Q_EMIT Update_Active_Cam(activeCam);
        camToggle = 0;
    }
}

QImage QNode::cvtCvMat2QImage(const cv::Mat & image)
{
    QImage qtemp;
    if(!image.empty() && image.depth() == CV_8U)
    {
        const unsigned char * data = image.data;
        qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
        for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
        {
            for(int x = 0; x < image.cols; ++x)
            {
                QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
                *p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
            }
        }
    }
    else if(!image.empty() && image.depth() != CV_8U)
    {
        printf("Wrong image format, must be 8_bits\n");
    }
    return qtemp;
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
      try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
    px = QPixmap::fromImage(cvtCvMat2QImage(cv_ptr->image));
    Q_EMIT Update_Image(&px);
}

}  // namespace qdude
