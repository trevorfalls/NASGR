/**
 * @file /include/qdude/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qdude_QNODE_HPP_
#define qdude_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Joy.h>
#include <QThread>
#include <QStringListModel>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    QPixmap PixmapModel (){return px;}

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void buttonAPressed(bool);
    void buttonBPressed(bool);
    void buttonXPressed(bool);
    void buttonYPressed(bool);
    void leftTrigger(int);
    void rightTrigger(int);
    void leftControlV(int);
    void leftControlH(int);
    void rightControlV(int);
    void rightControlH(int);
    void Update_Image(const QPixmap* image);

public Q_SLOTS:
    void magicSlotPressed();
    void magicSlotReleased();

private:
	int init_argc;
	char** init_argv;
    image_transport::Subscriber image_sub_;
    QPixmap px;
	ros::Publisher chatter_publisher;
    ros::Publisher cmd_publisher;
    ros::Subscriber joy_subscriber;
    QImage cvtCvMat2QImage(const cv::Mat & image);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    QStringListModel logging_model;
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

}  // namespace qdude

#endif /* qdude_QNODE_HPP_ */
