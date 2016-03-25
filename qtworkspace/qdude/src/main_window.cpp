/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/qdude/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qdude {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(QNode *node, QWidget *parent)
	: QMainWindow(parent)
    , qnode(node)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
    //ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
    //ui.view_logging->setModel(qnode.loggingModel());
    ui.imageLabel->setPixmap(qnode->PixmapModel());
    QObject::connect(qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    //XBOX
    QObject::connect(qnode,SIGNAL(buttonAPressed(bool)),ui.checkA,SLOT(setChecked(bool)));
    QObject::connect(qnode,SIGNAL(buttonBPressed(bool)),ui.checkB,SLOT(setChecked(bool)));
    QObject::connect(qnode,SIGNAL(buttonXPressed(bool)),ui.checkX,SLOT(setChecked(bool)));
    QObject::connect(qnode,SIGNAL(buttonYPressed(bool)),ui.checkY,SLOT(setChecked(bool)));
    QObject::connect(qnode,SIGNAL(leftTrigger(int)),ui.leftTrigger,SLOT(setValue(int)));
    QObject::connect(qnode,SIGNAL(rightTrigger(int)),ui.rightTrigger,SLOT(setValue(int)));
    QObject::connect(qnode,SIGNAL(leftControlV(int)),ui.leftControlV,SLOT(setValue(int)));
    QObject::connect(qnode,SIGNAL(leftControlH(int)),ui.leftControlH,SLOT(setValue(int)));
    QObject::connect(qnode,SIGNAL(rightControlV(int)),ui.rightControlV,SLOT(setValue(int)));
    QObject::connect(qnode,SIGNAL(rightControlH(int)),ui.rightControlH,SLOT(setValue(int)));
    QObject::connect(qnode,SIGNAL(Update_Image(const QPixmap*)),this,SLOT(updatePixmap(const QPixmap*)));
    QObject::connect(qnode,SIGNAL(Update_Active_Cam(int)),this,SLOT(updateCam(int)));
    ui.activeCam->setText("0");

    if(!qnode->init()) {
        showNoMasterMessage();
    }

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        //ui.view_logging->scrollToBottom();
}

/*void MainWindow::on_magicButton_pressed() {
    ui.lineEdit->setText("Yes");
}

void MainWindow::on_magicButton_released() {
    ui.lineEdit->setText("No");
}*/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About"),tr("<h2>NASGR Pilot Program Alpha 0.02</h2><p>Copyright Nobody</p><p>I mean, probably we have copyright, who knows...</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qdude");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    //ui.line_edit_master->setText(master_url);
    //ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    //ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    //ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    //    ui.line_edit_master->setEnabled(false);
    //    ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qdude");
    //settings.setValue("master_url",ui.line_edit_master->text());
    //settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    //settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    //settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::updatePixmap(const QPixmap* image)
{
    ui.imageLabel->setPixmap(*image);
}

void MainWindow::updateCam(int cam) {
    ui.activeCam->setText(QString::number(cam));
}

}  // namespace qdude

