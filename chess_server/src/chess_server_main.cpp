////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_server_main.cpp
//
/*! \file
 *
 * \brief The ROS node chess_server main.
 *
 * The ROS chess_server node provides services and subscriptions to play 
 * chess using the GNU chess gnuchess as the backend engine.
 *
 * The gnuchess (http://www.gnu.org/software/chess) engine is unmodified
 * (and hence, apt-get installable).
 *
 * The GNU Chess Versions supported and tested are 5.07, 6.1.1.

 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2016  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 *
 * \par Licence:
 * MIT
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

#include "ros/ros.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

#include "chess_server.h"
#include "chess_as.h"

using namespace std;
using namespace chess_server;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

//
// Application exit codes
//
#define APP_EC_OK   0   ///< success
#define APP_EC_INIT 2   ///< initialization fatal error
#define APP_EC_EXEC 4   ///< execution fatal error

const char *NodeName = "chess_server";  ///< this ROS node's registered name


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

/*!
 * \breif Chess server main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line arguments.
 *
 * \return Exits with 0 on success, \>0 on failure.
 */
int main(int argc, char *argv[])
{
  string    strNodeName;
  double    fHz = 10.0;
  int       n;
  int       rc;

  ros::init(argc, argv, NodeName);

  ros::NodeHandle nh(NodeName);

  if( !ros::master::check() )
  {
    return APP_EC_EXEC;
  }

  // read back actual node name
  strNodeName = ros::this_node::getName();

  ROS_INFO("%s: Node started.",  strNodeName.c_str());

  // the chess server work horse
  ChessServer chessServer(nh);

  // initialize chess server and underlining class objects
  if( (rc = chessServer.initialize()) != chess_engine::CE_OK )
  {
    ROS_ERROR_STREAM(strNodeName << ": "
        << "Failed to initialize chess server: "
        << chess_engine::strecode(rc) << "(" << rc << ")");
    return APP_EC_INIT;
  }

  // advertise services
  n = chessServer.advertiseServices();

  ROS_INFO("%s: %d services advertised.", strNodeName.c_str(), n);

  // advertise publishers
  n = chessServer.advertisePublishers();

  ROS_INFO("%s: %d topic publishers advertised.", strNodeName.c_str(), n);

  // subscribe to topics (none for now)
  n = chessServer.subscribeToTopics();

  ROS_INFO("%s: %d topics subscribed.", strNodeName.c_str(), n);

  // this nodes associated action servers
  ChessActionServers actionServers(chessServer);

  // start action servers
  n = actionServers.start();

  ROS_INFO("%s: %d action servers started.", strNodeName.c_str(), n);

  // set loop rate in hertz
  ros::Rate loop_rate(fHz);

  ROS_INFO("%s: Running at %.1lfHz.", strNodeName.c_str(), fHz);

  //
  // The loop.
  //
  while( ros::ok() )
  {
    // make any callbacks on pending ROS services
    ros::spinOnce(); 

    // publish any new data on advertised topics
    chessServer.publish();

    // sleep to keep loop hertz rate
    loop_rate.sleep();
  }

  return APP_EC_OK;
}
