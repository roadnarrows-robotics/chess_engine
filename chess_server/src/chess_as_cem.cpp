////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_as_cem.cpp
//
/*! \file
 *
 * \brief Compute engine move action server class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014-2016  RoadNarrows
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

#include <unistd.h>

#include <boost/bind.hpp>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "actionlib/server/simple_action_server.h"

#include "chess_server/ChessMove.h"
#include "chess_server/ComputeEnginesMoveAction.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceRosMsgs.h"

#include "chess_as_cem.h"
#include "chess_server.h"

using namespace std;
using namespace chess_server;

/*!
 * \brief Useful types.
 */
typedef chess_engine::ChessMove chess_move;


ASComputeEnginesMove::ASComputeEnginesMove(std::string name,
                                           ChessServer &chessServer) :
    action_name_(name),
    as_(nh_,
        name,                                           // action name
        boost::bind(&ASComputeEnginesMove::cbExecute, this, _1),
                                                        // execute callback
        false),                                         // don't auto-start
    chess_server_(chessServer)
{
  // register action preempt callback
  as_.registerPreemptCallback(boost::bind(&ASComputeEnginesMove::cbPreempt,
                                          this));
}

void ASComputeEnginesMove::cbExecute(const cem_goal_ptr &goal)
{
  uint16_t    uTicks;
  chess_move  move;
  int         rc;
  bool        bSuccess;

  ROS_INFO("%s: Execute.", action_name_.c_str());

  uTicks = 0;

  ros::Rate r(5.0);

  move.m_ePlayer = chess_server_.whoseTurn();

  //
  // Get engine's move. This can take up to 30 seconds, depending on the
  // difficulty setting.
  //
  rc = chess_server_.asyncComputeEnginesMove();

  bSuccess = rc == chess_engine::CE_OK? true: false;

  if( !bSuccess )
  {
    move.m_eResult = chess_engine::rcToMoveResult(rc);
    chess_engine::copyMoveToMsg(move, result_.move);
    as_.setAborted(result_); // abort action on error

    ROS_ERROR("%s: Compute engine's move failed: %s(%d).",
        action_name_.c_str(), chess_engine::strecode(rc).c_str(), rc);
  }

  while( bSuccess && chess_server_.isMoveInProgress() )
  {
    //
    // Action was preempted.
    //
    if( as_.isPreemptRequested() || !ros::ok() )
    {
      //chess_server_.abortMove("Action preempted");

      move.m_eResult=chess_engine::rcToMoveResult(chess_engine::CE_ECODE_INTR);
      chess_engine::copyMoveToMsg(move, result_.move);

      as_.setPreempted(result_); // set the action state to preempted
      bSuccess = false;

      ROS_INFO("%s: Execution preempted.", action_name_.c_str());
      break;
    }

    feedback_.thinking = uTicks++;
    as_.publishFeedback(feedback_);

    r.sleep();
  }

  if( bSuccess )
  {
    rc = chess_server_.asyncGetLastMove(move);
    move.m_eResult = chess_engine::rcToMoveResult(rc);
    chess_engine::copyMoveToMsg(move, result_.move);

    if( rc == chess_engine::CE_OK )
    {
      as_.setSucceeded(result_); // set the action state to succeeded
      ROS_INFO("%s: Execution succeeded", action_name_.c_str());
    }
    else
    {
      as_.setAborted(result_); // abort action on error
      ROS_INFO("%s: Execution failed", action_name_.c_str());
    }
  }
}

void ASComputeEnginesMove::cbPreempt()
{
  ROS_INFO("%s: Preempt.", action_name_.c_str());
  //chess_.getEngine().abortRead();
  as_.setPreempted();
  //as_.acceptNewGoal();  // does return from execution autoset this state?
}
