////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Node:      rqt_chess
//
// File:      rosqthread.cpp
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

#ifndef Q_MOC_RUN
#include "boost/bind.hpp"

#include <ros/ros.h>
#include <ros/console.h>

#include "rqt_chess/qnode.h"
#include "rqt_chess/rosqthread.h"
#endif  // Q_MOC_RUN

using namespace std;

RosQThread::RosQThread(int argc, char *argv[])
  : QNode(argc, argv, "rqt_chess")
{
}

RosQThread::~RosQThread()
{
}

void RosQThread::ros_comms_init()
{
  ros::NodeHandle nh;

  //
  // Advertise services.
  //
  advertiseServices(nh);

  ROS_INFO_STREAM(nodeName() << ": Advertised services registered.");

  //
  // Client services.
  //
  clientServices(nh);

  ROS_INFO_STREAM(nodeName() << ": Created client services.");

  //
  // Advertise publishers.
  //
  advertisePublishers(nh);
  
  ROS_INFO_STREAM(nodeName() << ": Advertised publishers registered.");
  
  //
  // Subscribed to topics.
  //
  subscribeToTopics(nh);
  
  ROS_INFO_STREAM(nodeName() << ": Subscribed topics registered.");
}

void RosQThread::advertiseServices(ros::NodeHandle &nh)
{
  std::string strSvc;

  //strSvc = ServiceNameSetGeofenceAlt;
  //m_services[strSvc] = nh.advertiseService(strSvc,
  //                                        &vSensorCloud::setGeofenceAlt,
  //                                        &(*this));
}

void RosQThread::clientServices(ros::NodeHandle &nh)
{
  //m_clientServices[m_serviceOut] =
  //  nh.serviceClient<mavros_msgs::CommandLong>(m_serviceOut);
}

void RosQThread::advertisePublishers(ros::NodeHandle &nh)
{
  //m_publishers[TopicNameCloud] =
  //        nh.advertise<sensor_msgs::PointCloud2>(TopicNameCloud, 1);
}

void RosQThread::subscribeToTopics(ros::NodeHandle &nh)
{
  //m_subscriptions[strTopic] = m_nh.subscribe(strTopic, 1,
  //                                            &vSensorCloud::cbFcDist,
  //                                            &(*this));
}

void RosQThread::publish()
{
  //m_publishers[TopicNameCloud].publish(m_msgCloud);
}

void RosQThread::run()
{
  double          hz = 1.0; //5.0;

  // set loop rate in Hertz
  ros::Rate loop_rate(hz);

  ROS_INFO_STREAM(nodeName() << ": Ready.");

  //
  // ROS thread loop.
  //
  while( ros::ok() )
  {
    // make any callbacks on pending ROS events
    ros::spinOnce(); 

    // publish all readied advertised topics
    publish();

    // sleep to keep at loop rate
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("Ros shutdown, proceeding to close the gui.");

  // used to signal the gui for a shutdown (useful to roslaunch)
  Q_EMIT rosShutdown();
}
