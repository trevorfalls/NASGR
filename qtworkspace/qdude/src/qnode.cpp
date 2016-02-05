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
#include <sensor_msgs/Joy.h>
#include <sstream>
#include "../include/qdude/qnode.hpp"
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
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_publisher = n.advertise<std_msgs::String>("/gui_cmd", 1000);
    joy_subscriber = n.subscribe<sensor_msgs::Joy>("joy",10,&QNode::joyCallback,this);
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
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_publisher = n.advertise<std_msgs::String>("/gui_cmd", 1000);
    joy_subscriber = n.subscribe<sensor_msgs::Joy>("joy",10,&QNode::joyCallback,this);
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

void QNode::magicSlotPressed() {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "yes";
    msg.data = ss.str();
    cmd_publisher.publish(msg);

    ros::spinOnce();
}

void QNode::magicSlotReleased() {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "no";
    msg.data = ss.str();
    cmd_publisher.publish(msg);

    ros::spinOnce();
}

void QNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
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
}

}  // namespace qdude
