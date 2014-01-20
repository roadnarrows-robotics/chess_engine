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
 * \brief Action Server class to get the engine's move class implementation.
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
#include "chess_server/GetEnginesMoveSvc.h"

#include "chess_as_get_engines_move.h"

using namespace std;
using namespace chess_engine;


void ASGetEnginesMove::execute_cb(const chess_server::GetEnginesMoveGoalConstPtr &goal)
{
  int rc;

  ROS_INFO("Executing GetEnginesMove action - please standby.");

#if 0
  pRobot->calibrateAsync(goal->force_recalib? true: false);

    while( pRobot->getAsyncState() == HekAsyncTaskStateWorking )
    {
      if( as_.isPreemptRequested() )
      {
        break;
      }

      updateOpState(names_, feedback_);
      as_.publishFeedback(feedback_);
      sleep(1);
    }

    rc = pRobot->getAsyncRc();
    result_.rc = rc;
    
    if( rc == HEK_OK )
    {
      ROS_INFO("Calibration complete.");
      as_.setSucceeded(result_);
      as_.publishFeedback(feedback_);
    }
    else if( rc != -HEK_ECODE_INTR )
    {
      ROS_ERROR("Calibration aborted with error code %d.", rc);
      as_.setAborted(result_);
    }
#endif
}

void ASGetEnginesMove::preempt_cb()
{
  ROS_INFO("GetEnginesMove action cancelled.");
//  pRobot->cancelAsyncTask();
//  as_.setPreempted();
//  as_.acceptNewGoal();
}
