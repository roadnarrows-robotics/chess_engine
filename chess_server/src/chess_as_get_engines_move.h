////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_as_get_engines_move.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Action Server class to get the engine's move class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#ifndef _CHESS_AS_GET_ENGINES_MOVE_H
#define _CHESS_AS_GET_ENGINES_MOVE_H

#include <unistd.h>

#include <boost/bind.hpp>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "actionlib/server/simple_action_server.h"

#include "chess_server/ChessMoveMsg.h"
#include "chess_server/GetEnginesMoveSvc.h"
#include "chess_server/GetEnginesMoveAction.h"

namespace chess_engine
{
  class ASGetEnginesMove
  {
  public:
    ASGetEnginesMove(std::string name, ros::NodeHandle nh):
      as_(nh, name, boost::bind(&ASGetEnginesMove::execute_cb, this, _1),false),
      action_name_(name)
    {
      // register the goal and feeback callbacks
      as_.registerPreemptCallback(
        boost::bind(&ASGetEnginesMove::preempt_cb, this));
      as_.start();
    }

    virtual ~ASGetEnginesMove(void)
    {
    }

    void execute_cb(const chess_server::GetEnginesMoveGoalConstPtr &goal);

    void preempt_cb();

  protected:
    std::string action_name_;                         ///< action name
    actionlib::SimpleActionServer<chess_server::GetEnginesMoveAction> as_;
                                                      ///< action server
    chess_server::GetEnginesMoveFeedback feedback_;   ///< progress feedback
    chess_server::GetEnginesMoveResult   result_;     ///< action results
  };

} // namespace chess_engine

#endif // _CHESS_AS_GET_ENGINES_MOVE_H
