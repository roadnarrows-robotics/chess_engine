////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Node:      rqt_chess
//
// File:      rosqthread.h
//
/*! \file
 *
 * \brief The ROS Qt thread provides the interface between ROS and the Qt UI.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 * (C) 2017  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 *
 * \par License:
 * MIT
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _ROS_THREAD_H
#define _ROS_THREAD_H

#include <string>

#include <QThread>
#include <QStringListModel>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include "rqt_chess/qnode.h"
#endif // Q_MOC_RUN

class RosQThread : public QNode
{
public:
  RosQThread(int argc, char *argv[]);

  virtual ~RosQThread();

  virtual void ros_comms_init();

  virtual void run();

Q_SIGNALS:

protected:
  void advertiseServices(ros::NodeHandle &nh);

  void clientServices(ros::NodeHandle &nh);

  void advertisePublishers(ros::NodeHandle &nh);

  void subscribeToTopics(ros::NodeHandle &nh);

  void publish();
};

#endif // _ROS_THREAD_H
