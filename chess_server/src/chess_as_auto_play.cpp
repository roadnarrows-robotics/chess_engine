////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_as_auto_play.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Autoplay the next n moves action server class implementation.
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

#include <unistd.h>

#include <boost/bind.hpp>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "actionlib/server/simple_action_server.h"

#include "chess_server/ChessMoveMsg.h"

#include "chess_engine/ceMove.h"
#include "chess_engine/ceError.h"

#include "chess_as_auto_play.h"

using namespace std;
using namespace chess_engine;


void ASAutoPlay::execute_cb(const chess_server::AutoPlayGoalConstPtr &goal)
{
  uint_t    numMoves;
  double    hz;
  uint_t    n;
  Move      move;
  bool      success;
  int       rc;

  ROS_INFO("%s: Execute.", action_name_.c_str());

  numMoves  = (uint_t)goal->num_moves;
  hz        = (double)goal->hz;
  n         = 0;
  success   = true;
  rc        = CE_OK;

  ros::Rate r(hz);

  while( chess_.getGame().isPlaying() && ((numMoves == 0) || (n < numMoves)) )
  {
    //
    // Action was preempted.
    //
    if( as_.isPreemptRequested() || !ros::ok() )
    {
      ROS_INFO("%s: Execution preempted", action_name_.c_str());
      result_.rc = rc;
      as_.setPreempted(result_); // set the action state to preempted
      success = false;
      break;
    }

    // engine make move and update game state
    rc = chess_.getEngine().getEnginesMove(move, true);
    rc = chess_.getGame().sync(move);

    ROS_DEBUG_STREAM(action_name_ << ": " << move << endl);

    //
    // Action resulted in an error.
    //
    if( rc != CE_OK )
    {
      ROS_INFO("%s: Execution error: %s(%d)",
        action_name_.c_str(), strerror(rc).c_str(), rc);
      result_.rc = rc;
      as_.setAborted(result_); // abort action on error
      success = false;
      break;
    }

    //
    // Move succeeded.
    //
    else
    {
      // convert move to result.
      chess_.toMsgMove(move, feedback_.move);

      // publish feedback
      as_.publishFeedback(feedback_);

      // advance number of moves only after black's move
      if( move.m_player == Black )
      {
        ++n;
      }
    }

    r.sleep();
  }

  if( success )
  {
    // result_.sequence = feedback_.sequence;
    ROS_INFO("%s: Exectution succeeded", action_name_.c_str());
    result_.rc = CE_OK;
    as_.setSucceeded(result_); // set the action state to succeeded
  }
}

void ASAutoPlay::preempt_cb()
{
  ROS_INFO("%s: Preempt.", action_name_.c_str());
  chess_.getEngine().abortRead();
  as_.setPreempted();
  //as_.acceptNewGoal();  // does return from execution autoset this state?
}
