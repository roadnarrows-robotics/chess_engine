////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Node:      rqt_chess
//
// File:      main.cpp
//
/*! \file
 *
 * \brief The rqt_chess main.
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

#include <QApplication>

#include "rqt_chess/rqtchesswin.h"
#include "rqt_chess/rosqthread.h"

int main(int argc, char *argv[])
{
  QApplication  app(argc, argv);
  RosQThread    qros(argc, argv);
  RqtChessWin   win(&qros);

  win.show();

  return app.exec();
}
