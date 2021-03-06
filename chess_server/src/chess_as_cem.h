////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_as_cem.h
//
/*! \file
 *
 * \brief Compute engine's move action server class interface.
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

#ifndef _CHESS_AS_CEM_H
#define _CHESS_AS_CEM_H

#include <unistd.h>

#include <boost/bind.hpp>

#include "ros/ros.h"

#include "std_msgs/String.h"

#include "actionlib/server/simple_action_server.h"

#include "chess_server/ChessMove.h"
#include "chess_server/ComputeEnginesMoveAction.h"

namespace chess_server
{
  //
  // Forward declarations.
  //
  class ChessServer;

  /*!
   * \brief Compute chess engine's move action server class.
   */
  class ASComputeEnginesMove
  {
  public:
    //
    // Useful de-uglified types
    //
    typedef ComputeEnginesMoveAction        cem_action;   ///< action
    typedef actionlib::SimpleActionServer<cem_action> action_server;
                                                          ///< action server
    typedef ComputeEnginesMoveGoalConstPtr  cem_goal_ptr; ///< goal pointer
    typedef ComputeEnginesMoveFeedback      cem_feedback; ///< progress feedback
    typedef ComputeEnginesMoveResult        cem_result;   ///< action results

    /*!
     * \brief Initialization constructor.
     *
     * \param name    Action server name.
     * \param chess   Node-specific class instance.
     */
    ASComputeEnginesMove(std::string name, ChessServer &chessServer);

    /*!
     * \brief Destructor.
     */
    virtual ~ASComputeEnginesMove()
    {
    }

    /*!
     * \brief Start the action server.
     */
    void start()
    {
      as_.start();
    }

    /*!
     * \brief ROS callback to execute action.
     *
     * The callback is executed in a separate ROS-created thread so that it
     * can block.Typically, the callback is invoked within a ROS spin.
     */
    void cbExecute(const cem_goal_ptr &goal);

    /*!
     * \brief ROS callback to preempt action.
     *
     * This is only needed if actions are required outside of the blocking
     * execution callback thread.
     */
    void cbPreempt();

  protected:
    ros::NodeHandle   nh_;        ///< keep first (ROS magic)

    std::string   action_name_;   ///< action name
    action_server as_;            ///< action server
    cem_feedback  feedback_;      ///< progress feedback
    cem_result    result_;        ///< action results

    ChessServer   &chess_server_; ///< chess server reference
  };

} // namespace chess_server

#endif // _CHESS_AS_CEM_H
