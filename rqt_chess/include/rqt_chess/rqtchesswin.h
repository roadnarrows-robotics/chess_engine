////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Node:      rqt_chess
//
// File:      rqtchesswin.h
//
/*! \file
 *
 * \brief Qt based gui for rqt_chess.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016-2017  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 *
 * \par License:
 * MIT
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RQT_CHESS_H
#define _RQT_CHESS_H

#include <QMainWindow>

#include "chess_engine/ceChess.h"

#include "rqt_chess/qnode.h"
#include "rqt_chess/chessscene.h"

namespace Ui
{
  class RqtChessWin;
}

class RqtChessWin : public QMainWindow
{
  Q_OBJECT

public:
  explicit RqtChessWin(QNode *node, QWidget *parent = 0);

  ~RqtChessWin();

protected:
   void showNoMasterMessage();

private:
  Ui::RqtChessWin *ui;
  QNode           *m_node;
  ChessScene      *m_chessScene;

  void createConnections();
};

#if 0 // OLD ROS QT
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rqt_chess {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
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
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace rqt_chess
#endif // OLD ROS QT

#endif // _RQT_CHESS_H
