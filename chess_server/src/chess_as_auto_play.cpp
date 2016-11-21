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
 * \brief Autoplay the next n moves action server class implementation.
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
 * appear in all copies of the source code and (2) redi  <url type="website">http://github.com/roadnarrows-robotics/chess_engine</url>
stributions
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

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceError.h"

#include "chess_as_auto_play.h"
#include "chess_server.h"

using namespace std;
using namespace chess_server;

/*!
 * \brief Helpful typedefs
 */
typedef chess_engine::ChessColor              chess_color;
typedef chess_engine::ChessColor              chess_player;
typedef chess_engine::ChessMove               chess_move;

ASAutoPlay::ASAutoPlay(std::string name, ChessServer &chessServer) :
      action_name_(name),
      as_(nh_,
          name,                     // action name
          boost::bind(&ASAutoPlay::cbExecute, this, _1),
                                    // execute callback
          false),                   // don't auto-start
      chess_server_(chessServer)
{
  // register action preempt callback
  as_.registerPreemptCallback( boost::bind(&ASAutoPlay::cbPreempt, this));
}

void ASAutoPlay::cbExecute(const chess_server::AutoPlayGoalConstPtr &goal)
{
  int    numMoves;
  double    hz;
  int    n;
  chess_move      move;
  bool      success;
  int       rc;

  ROS_INFO("%s: Execute.", action_name_.c_str());

  numMoves  = (int)goal->num_moves;
  hz        = (double)goal->hz;
  n         = 0;
  success   = true;
  rc        = chess_engine::CE_OK;

  ros::Rate r(hz);

  //while( chess_server_.getGame().isPlaying() && ((numMoves == 0) || (n < numMoves)) )
  while( true )
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
    //rc = chess_server_.getEngine().getEnginesMove(move, true);
    //rc = chess_server_.getGame().sync(move);

    ROS_DEBUG_STREAM(action_name_ << ": " << move << endl);

    //
    // Action resulted in an error.
    //
    if( rc != chess_engine::CE_OK )
    {
      ROS_INFO("%s: Execution error: %s(%d)",
        action_name_.c_str(), chess_engine::strecode(rc).c_str(), rc);
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
      //chess_server_.toMsgMove(move, feedback_.move);

      // publish feedback
      as_.publishFeedback(feedback_);

      // advance number of moves only after black's move
      if( move.m_ePlayer == chess_engine::Black )
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
    result_.rc = chess_engine::CE_OK;
    as_.setSucceeded(result_); // set the action state to succeeded
  }
}

void ASAutoPlay::cbPreempt()
{
  ROS_INFO("%s: Preempt.", action_name_.c_str());
  //chess_server_.getEngine().abortRead();
  as_.setPreempted();
  //as_.acceptNewGoal();  // does return from execution autoset this state?
}
