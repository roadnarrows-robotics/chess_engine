////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_service_srv.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The ROS chess_server node supported services.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

#include "ros/ros.h"

#include "chess_server/StartNewGame.h"
#include "chess_server/MakeAMove.h"
#include "chess_server/MakeAMoveSAN.h"
#include "chess_server/GetEnginesMove.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

#include "rnr/rnrconfig.h"

#include "chess_engine_gnu.h"
#include "chess_server.h"

using namespace std;
using namespace chess_engine;


void ChessServer::advertiseServices(ros::NodeHandle &nh)
{
  string  strSvc;

  strSvc = "start_new_game";
  m_services[strSvc] = nh.advertiseService(strSvc,
                                          &ChessServer::startNewGame,
                                          &(*this));

  strSvc = "make_a_move_san";
  m_services[strSvc] = nh.advertiseService(strSvc,
                                          &ChessServer::makeAMoveSAN,
                                          &(*this));

  strSvc = "make_a_move";
  m_services[strSvc] = nh.advertiseService(strSvc,
                                          &ChessServer::makeAMove,
                                          &(*this));

  strSvc = "get_engines_move";
  m_services[strSvc] = nh.advertiseService(strSvc,
                                          &ChessServer::getEnginesMove,
                                          &(*this));
}

/*!
 *  \brief Request calibrate.
 */
bool ChessServer::startNewGame(chess_server::StartNewGame::Request  &req,
                               chess_server::StartNewGame::Response &rsp)
{
  ChessColor  colorPlayer;
  int         rc;

  ROS_DEBUG("start_new_game");

  colorPlayer = (ChessColor)req.player.color;

  if( (rc = m_engine.startNewGame(colorPlayer)) == CE_OK )
  {
    m_game.setupBoard();
    ROS_INFO_STREAM("start_new_game: player=" << nameOfColor(colorPlayer));
  }

  else
  {
    ROS_ERROR_STREAM("start_new_game: "
        << chess_engine::strerror(rc) << "(" << rc << ")");
  }

  rsp.rc = (int8_t)rc;

  return rc == CE_OK? true: false;
}

bool ChessServer::makeAMoveSAN(chess_server::MakeAMoveSAN::Request  &req,
                               chess_server::MakeAMoveSAN::Response &rsp)
{
  Move  move;
  int   rc;

  ROS_DEBUG("make_a_move_san");

  move.fromSAN(req.SAN);

  rc = m_engine.makePlayersMove(move);

  ROS_DEBUG_STREAM("e: " << move << endl);

  if( rc == CE_OK )
  {
    rc = m_game.sync(move);
    ROS_DEBUG_STREAM("g: " << move << endl);
  }

  toMsgMove(move, rsp.move);

  if( rc == CE_OK )
  {
    ROS_INFO_STREAM(move.m_nMove << ". " << nameOfColor(move.m_player) << ": "
        << move.m_strAN);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("make_a_move: "
        << chess_engine::strerror(rc) << "(" << rc << ")");
    return false;
  }
}


bool ChessServer::makeAMove(chess_server::MakeAMove::Request  &req,
                            chess_server::MakeAMove::Response &rsp)
{
  Move  move;
  int   rc;

  ROS_DEBUG("make_a_move");

  move.m_sqFrom.m_file = req.src.file;
  move.m_sqFrom.m_rank = req.src.rank;
  move.m_sqTo.m_file   = req.dst.file;
  move.m_sqTo.m_rank   = req.dst.rank;

  rc = m_engine.makePlayersMove(move);

  ROS_DEBUG_STREAM("e: " << move << endl);

  if( rc == CE_OK )
  {
    rc = m_game.sync(move);
    ROS_DEBUG_STREAM("g: " << move << endl);
  }

  toMsgMove(move, rsp.move);

  if( rc == CE_OK )
  {
    ROS_INFO_STREAM(move.m_nMove << ". " << nameOfColor(move.m_player) << ": "
        << move.m_strAN);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("make_a_move: "
        << chess_engine::strerror(rc) << "(" << rc << ")");
    return false;
  }
}

bool ChessServer::getEnginesMove(chess_server::GetEnginesMove::Request  &req,
                                 chess_server::GetEnginesMove::Response &rsp)
{
  Move  move;
  int   rc;

  ROS_DEBUG("get_engines_move");

  rc = m_engine.getEnginesMove(move);

  ROS_DEBUG_STREAM("e: " << move << endl);

  if( rc == CE_OK )
  {
    rc = m_game.sync(move);
    ROS_DEBUG_STREAM("g: " << move << endl);
  }

  toMsgMove(move, rsp.move);

  if( rc == CE_OK )
  {
    ROS_INFO_STREAM(move.m_nMove << ". " << nameOfColor(move.m_player) << ": "
        << move.m_strAN);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("get_engines_move: "
        << chess_engine::strerror(rc) << "(" << rc << ")");
    return false;
  }
}

void ChessServer::toMsgMove(const Move &move, chess_server::ChessMove &msgMove)
{
  msgMove.move_num         = move.m_nMove;
  msgMove.player.color     = move.m_player;
  msgMove.AN               = move.m_strAN;
  msgMove.moved.piece      = move.m_piece;
  msgMove.src.file         = move.m_sqFrom.m_file;
  msgMove.src.rank         = move.m_sqFrom.m_rank;
  msgMove.dst.file         = move.m_sqTo.m_file;
  msgMove.dst.rank         = move.m_sqTo.m_rank;
  msgMove.captured.piece   = move.m_captured;
  msgMove.en_passant       = move.m_en_passant;
  msgMove.castle.side      = move.m_castle;
  msgMove.aux_at.file      = move.m_sqAuxAt.m_file;
  msgMove.aux_at.rank      = move.m_sqAuxAt.m_rank;
  msgMove.aux_to.file      = move.m_sqAuxTo.m_file;
  msgMove.aux_to.rank      = move.m_sqAuxTo.m_rank;
  msgMove.promotion.piece  = move.m_promotion;
  msgMove.check            = move.m_check;
  msgMove.winner.color     = move.m_winner;
  msgMove.result.code      = move.m_result;
}

