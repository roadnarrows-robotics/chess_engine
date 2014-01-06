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

#include <string>
#include "ros/ros.h"

#include "chess_server/StartNewGame.h"
#include "chess_server/MakeAMove.h"
#include "chess_server/MakeAMoveSAN.h"
#include "chess_server/GetEnginesMove.h"
#include "chess_server/Resign.h"
#include "chess_server/AutoPlay.h"
#include "chess_server/SetDifficulty.h"
#include "chess_server/GetPlayHistory.h"
#include "chess_server/GetBoardState.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

#include "chess_engine_gnu.h"

namespace chess_engine
{
  class ChessServer
  {
  public:
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

    const ChessEngineGnu &getEngine()
    {
      return m_engine;
    }

  protected:
    ChessEngineGnu  m_engine;
    Game            m_game;
    std::map<std::string, ros::ServiceServer> m_services;

    //
    // Service callbacks
    //
    bool startNewGame(chess_server::StartNewGame::Request  &req,
                      chess_server::StartNewGame::Response &rsp);

    bool makeAMoveSAN(chess_server::MakeAMoveSAN::Request  &req,
                      chess_server::MakeAMoveSAN::Response &rsp);

    bool makeAMove(chess_server::MakeAMove::Request  &req,
                   chess_server::MakeAMove::Response &rsp);

    bool getEnginesMove(chess_server::GetEnginesMove::Request  &req,
                        chess_server::GetEnginesMove::Response &rsp);

    bool resign(chess_server::Resign::Request  &req,
                chess_server::Resign::Response &rsp);

    bool autoplay(chess_server::AutoPlay::Request  &req,
                  chess_server::AutoPlay::Response &rsp);

    bool setDifficulty(chess_server::SetDifficulty::Request  &req,
                       chess_server::SetDifficulty::Response &rsp);

    bool getPlayHistory(chess_server::GetPlayHistory::Request  &req,
                        chess_server::GetPlayHistory::Response &rsp);

    bool getBoardState(chess_server::GetBoardState::Request  &req,
                       chess_server::GetBoardState::Response &rsp);

    //
    // Support
    //
    void toMsgMove(const Move &move, chess_server::ChessMove &msgMove);
  };

} // namespace chess_engine


#endif // _CHESS_SERVER_H
