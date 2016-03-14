/**
 * @file /include/qdude/main_window.hpp
 *
 * @brief Qt based gui for qdude.
 *
 * @date November 2010
 **/
#ifndef qdude_MAIN_WINDOW_H
#define qdude_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qdude {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(QNode *node, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
    void on_actionAbout_triggered();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    //void on_magicButton_pressed();
    //void on_magicButton_released();
    void updatePixmap(const QPixmap* image);
    void updateCam(int cam);

private:
	Ui::MainWindowDesign ui;
    QNode *qnode;
};

}  // namespace qdude

#endif // qdude_MAIN_WINDOW_H
