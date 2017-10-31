/*!
 * qnode.cpp
 *
 * \file
 *
 * \brief Example qt programs, generated from code similar to that used by the
 * roscreate-qt-pkg script and styled on roscpp_tutorials.
 *
 * \par Maintainer Status:
 * maintained
 *
 * \par Maintainer:
 * Daniel Stonier <d.stonier@gmail.com>
 *
 * \author Daniel Stonier
 *
 * \par License:
 * BSD
 *
 * \par URL:
 * - http://wiki.ros.org/qt_tutorials
 * - http://docs.ros.org/hydro/api/qt_tutorials/html/index.html
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include "rqt_chess/qnode.h"
#include <std_msgs/String.h>
#include <sstream>

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, const std::string &name ) :
        init_argc(argc),
        init_argv(argv),
        node_name(name)
{
}

QNode::~QNode()
{
  shutdown();
}

void QNode::shutdown()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::on_init()
{
  ros::init(init_argc,init_argv,node_name);
  if ( ! ros::master::check() )
  {
    return false;
  }
  // our node handles go out of scope, so we want to control shutdown explicitly
  ros::start();
  ros_comms_init();
  start();
  return true;
}

bool QNode::on_init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, node_name);
  if ( ! ros::master::check() )
  {
    return false;
  }
  // our node handles go out of scope, so we want to control shutdown explicitly
  ros::start();
  ros_comms_init();
  start();
  return true;
}
