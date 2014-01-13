////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_server.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The ROS chess_server node class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013  RoadNarrows
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

#ifndef _CHESS_SERVER_H
#define _CHESS_SERVER_H

#include <pthread.h>

#include <string>

#include "ros/ros.h"

#include "chess_server/StartNewGameSvc.h"
#include "chess_server/MakeAMoveSvc.h"
#include "chess_server/MakeAMoveSANSvc.h"
#include "chess_server/GetEnginesMoveSvc.h"
#include "chess_server/ResignSvc.h"
#include "chess_server/AutoPlaySvc.h"
#include "chess_server/SetDifficultySvc.h"
#include "chess_server/GetPlayHistorySvc.h"
#include "chess_server/GetBoardStateSvc.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

#include "chess_engine_gnu.h"

namespace chess_engine
{
  class ChessServer
  {
  public:
    /*!
     * \brief Asynchronous task state.
     */
    enum ActionState
    {
      ActionStateIdle     = 0,    ///< idle, no actions running
      ActionStateWorking  = 1     ///< action(s) running
    };

    ChessServer()
    {
    }

    virtual ~ChessServer()
    {
      disconnect();
    }

    virtual int connect(const std::string &strChessApp="gnuchess")
    {
      return m_engine.openConnection(strChessApp);
    }

    virtual int disconnect()
    {
      return m_engine.closeConnection();
    }

    virtual void advertiseServices(ros::NodeHandle &nh);

    virtual void advertisePublishers(ros::NodeHandle &nh);

    virtual void subscribeToTopics(ros::NodeHandle &nh);

    virtual void bindActionServers(ros::NodeHandle &nh);

    virtual void publish();

    const ChessEngineGnu &getEngine()
    {
      return m_engine;
    }

  protected:
    ChessEngineGnu  m_engine;     // chess backend engine
    Game            m_game;       // chess game state
    std::map<std::string, ros::ServiceServer> m_services;
                                  // chess_server services
    std::map<std::string, ros::Publisher> m_publishers;

    // asynchronous task control
    ActionState m_eActionState;  ///< asynchronous task state
    bool m_bInAutoPlay;
    bool m_bInWaitForEngine;
    bool              m_eAsyncTaskId;     ///< asynchronous task id
    void             *m_pAsyncTaskArg;    ///< asynchronous argument
    pthread_t         m_threadAction;     ///< async pthread identifier 

    //
    // Service callbacks
    //
    bool startNewGame(chess_server::StartNewGameSvc::Request  &req,
                      chess_server::StartNewGameSvc::Response &rsp);

    bool makeAMoveSAN(chess_server::MakeAMoveSANSvc::Request  &req,
                      chess_server::MakeAMoveSANSvc::Response &rsp);

    bool makeAMove(chess_server::MakeAMoveSvc::Request  &req,
                   chess_server::MakeAMoveSvc::Response &rsp);

    bool getEnginesMove(chess_server::GetEnginesMoveSvc::Request  &req,
                        chess_server::GetEnginesMoveSvc::Response &rsp);

    bool resign(chess_server::ResignSvc::Request  &req,
                chess_server::ResignSvc::Response &rsp);

    bool autoplay(chess_server::AutoPlaySvc::Request  &req,
                  chess_server::AutoPlaySvc::Response &rsp);

    bool setDifficulty(chess_server::SetDifficultySvc::Request  &req,
                       chess_server::SetDifficultySvc::Response &rsp);

    bool getPlayHistory(chess_server::GetPlayHistorySvc::Request  &req,
                        chess_server::GetPlayHistorySvc::Response &rsp);

    bool getBoardState(chess_server::GetBoardStateSvc::Request  &req,
                       chess_server::GetBoardStateSvc::Response &rsp);

    //
    // Action (and service thread
    //

    //
    // Support
    //
    void toMsgMove(const Move &move, chess_server::ChessMoveMsg &msgMove);
  };

} // namespace chess_engine


#endif // _CHESS_SERVER_H
