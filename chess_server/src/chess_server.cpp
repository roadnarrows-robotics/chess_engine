////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_server.cpp
//
/*! \file
 *
 * \brief The ROS chess_server node supported services.
 *
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

#include "chess_server/ChessNewGameStatus.h"
#include "chess_server/ChessMoveStamped.h"
#include "chess_server/ChessEndGameStatus.h"

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
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

#include "rnr/rnrconfig.h"

#include "chess_engine_gnu.h"
#include "chess_server.h"

using namespace std;
using namespace chess_engine;


//------------------------------------------------------------------------------
// ChessServer Class
//------------------------------------------------------------------------------

ChessServer::ChessServer(ros::NodeHandle &nh) : m_nh(nh)
{
  m_bPubNewGame = false;
  m_nPubLastPly = 0;
  m_bPubEndGame = false;

#ifdef INC_ACTION_THREAD
  createActionThread();
#endif // INC_ACTION_THREAD
}

ChessServer::~ChessServer()
{
#ifdef INC_ACTION_THREAD
  destroyActionThread();
#endif // INC_ACTION_THREAD

  disconnect();
}


//..............................................................................
// Services
//..............................................................................

void ChessServer::advertiseServices()
{
  string  strSvc;

  strSvc = "start_new_game";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::startNewGame,
                                          &(*this));

  strSvc = "make_a_move_san";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::makeAMoveSAN,
                                          &(*this));

  strSvc = "make_a_move";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::makeAMove,
                                          &(*this));

  strSvc = "get_engines_move";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::getEnginesMove,
                                          &(*this));

  strSvc = "resign";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::resign,
                                          &(*this));

  strSvc = "autoplay";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::autoplay,
                                          &(*this));

  strSvc = "set_difficulty";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::setDifficulty,
                                          &(*this));

  strSvc = "get_play_history";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::getPlayHistory,
                                          &(*this));

  strSvc = "get_board_state";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &ChessServer::getBoardState,
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

    m_bPubNewGame = true;
    m_nPubLastPly = 0;
    m_bPubEndGame = false;

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
    if( m_game.getWinner() != NoColor )
    {
      m_bPubEndGame = true;
    }

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

  move.m_posFrom.m_file = req.src.file;
  move.m_posFrom.m_rank = req.src.rank;
  move.m_posTo.m_file   = req.dst.file;
  move.m_posTo.m_rank   = req.dst.rank;

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
    if( m_game.getWinner() != NoColor )
    {
      m_bPubEndGame = true;
    }

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
    if( m_game.getWinner() != NoColor )
    {
      m_bPubEndGame = true;
    }

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

bool ChessServer::resign(chess_server::Resign::Request  &req,
                         chess_server::Resign::Response &rsp)
{
  ROS_DEBUG("resign");

  m_engine.resign();

  m_game.stopPlaying(Resign, opponent(m_engine.getPlayersColor()));

  ROS_INFO_STREAM(nameOfColor(m_engine.getPlayersColor()) << " resigned.");

  m_bPubEndGame = true;

  rsp.rc = CE_OK;

  return true;
}

bool ChessServer::autoplay(chess_server::AutoPlay::Request  &req,
                           chess_server::AutoPlay::Response &rsp)
{
  ROS_DEBUG("autoplay");

  return true;
}

bool ChessServer::setDifficulty(chess_server::SetDifficulty::Request  &req,
                                chess_server::SetDifficulty::Response &rsp)
{
  float   difficulty;

  ROS_DEBUG("set_difficulty");

  difficulty = req.difficulty;

  m_engine.setGameDifficulty(difficulty);

  ROS_INFO("%s: Game difficulty set to %4.1f.",
      ros::this_node::getName().c_str(), difficulty);

  rsp.rc = CE_OK;

  return true;
}

bool ChessServer::getPlayHistory(chess_server::GetPlayHistory::Request  &req,
                                 chess_server::GetPlayHistory::Response &rsp)
{
  std::vector<Move>           &move = m_game.getGameHistory();
  std::vector<Move>::iterator iter;
  ChessColor                  colorPlayer = White;
  
  ROS_DEBUG("get_play_history");

  for(iter = move.begin(); iter != move.end(); ++iter)
  {
    if( colorPlayer == White )
    {
      rsp.whiteAN.push_back(iter->m_strAN);
    }
    else
    {
      rsp.blackAN.push_back(iter->m_strAN);
    }
    colorPlayer = opponent(colorPlayer);
  }

  ROS_INFO("%s: Retrieved chess play history of %zu moves.",
      ros::this_node::getName().c_str(), move.size());

  return true;
}

bool ChessServer::getBoardState(chess_server::GetBoardState::Request  &req,
                                chess_server::GetBoardState::Response &rsp)
{
  int           file;
  int           rank;
  BoardElem    *pElem;
  ChessPos      pos;

  chess_server::ChessPos    msgSquare;
  chess_server::ChessColor  msgColor;
  chess_server::ChessPiece  msgPiece;

  ROS_DEBUG("get_board_state");

  for(rank = ChessRank1; rank <= ChessRank8; ++rank)
  {
    for(file = ChessFileA; file <= ChessFileH; ++file)
    {
      pElem = m_game.getBoardElem((ChessFile)file, (ChessRank)rank);
      if( pElem != NULL )
      {
        msgSquare.file = (uint8_t)file;
        msgSquare.rank = (uint8_t)rank;
        rsp.squares.push_back(msgSquare);

        msgColor.color = (uint8_t)pElem->m_color;
        rsp.colors.push_back(msgColor);

        msgPiece.piece = (uint8_t)pElem->m_piece;
        rsp.pieces.push_back(msgPiece);
      }
      else
      {
        ROS_ERROR("No chess square found out %c%c.", (char)file, (char)rank);
        return false;
      }
    }
  }

  ROS_INFO("%s: Retrieved chess board state.",
      ros::this_node::getName().c_str());

  return true;
}


//..............................................................................
// Topic Publishers
//..............................................................................

void ChessServer::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "new_game_status";
  m_publishers[strPub] = m_nh.advertise<chess_server::ChessNewGameStatus>(
                                  strPub, nQueueDepth);

  strPub = "move_status";
  m_publishers[strPub] = m_nh.advertise<chess_server::ChessMoveStamped>(
                                  strPub, nQueueDepth);

  strPub = "end_of_game_status";
  m_publishers[strPub] = m_nh.advertise<chess_server::ChessEndGameStatus>(
                                  strPub, nQueueDepth);
}

uint32_t ChessServer::publish(uint32_t seqnum)
{
  // new game
  if( m_bPubNewGame )
  {
    chess_server::ChessNewGameStatus &msg = m_msgNewGameStatus;

    stampHeader(msg.header, msg.header.seq+1, "new_game");
    
    msg.player.color  = m_engine.getPlayersColor();

    m_publishers["new_game_status"].publish(msg);

    m_bPubNewGame = false;
  }

  // moves
  while( m_nPubLastPly < m_game.getNumOfPlies() )
  {
    chess_server::ChessMoveStamped &msg = m_msgMoveStamped;

    stampHeader(msg.header, msg.header.seq+1, "move");

    toMsgMove(m_game[m_nPubLastPly], msg.move);

    m_publishers["move_status"].publish(msg);

    ++m_nPubLastPly;
  }
  
  // end of game
  if( m_bPubEndGame )
  {
    chess_server::ChessEndGameStatus &msg = m_msgEndOfGameStatus;

    stampHeader(msg.header, msg.header.seq+1, "end_of_game");

    msg.winner.color  = m_game.getWinner();
    msg.result.code   = m_game.getEndOfGameReason();

    m_publishers["end_of_game_status"].publish(msg);

    m_bPubEndGame = false;
  }

  return seqnum;
}


//..............................................................................
// Subscribed Topics
//..............................................................................

void ChessServer::subscribeToTopics()
{
}


//..............................................................................
// Action thread
//..............................................................................

#ifdef INC_ACTION_THREAD
int ChessServer::createActionThread()
{
  int   rc;

  m_eActionState  = ActionStateWorking;

  pthread_mutex_init(&m_mutexAction, NULL);
  pthread_cond_init(&m_condAction,   NULL);
  
  rc = pthread_create(&m_threadAction, NULL, actionThread, (void*)this);
 
  if( rc == 0 )
  {
    m_eActionState = ActionStateIdle;
    rc = CE_OK;
  }

  else
  {
    m_eActionState = ActionStateExit;
    pthread_cond_destroy(&m_condAction);
    pthread_mutex_destroy(&m_mutexAction);
    rc = -CE_ECODE_SYS;
    ROS_ERROR("pthread_create()");
  }

  return rc;
}

void ChessServer::destroyActionThread()
{
  m_eActionState = ActionStateExit;

  signalActionThread();

  pthread_cancel(m_threadAction);
  pthread_join(m_threadAction, NULL);

  pthread_cond_destroy(&m_condAction);
  pthread_mutex_destroy(&m_mutexAction);

  ROS_INFO("%s: Node action thread destroyed.",
      ros::this_node::getName().c_str());
}

int ChessServer::execAction(boost::function<void()> execAction)
{
  int   rc;

  lock();

  if( m_eActionState == ActionStateIdle )
  {
    m_execAction = execAction;
    m_eActionState = ActionStateWorking;
    signalActionThread();
    rc = CE_OK;
  }
  else
  {
    rc = -CE_ECODE_NO_EXEC;
  }

  unlock();

  return rc;
}

void ChessServer::signalActionThread()
{
  pthread_cond_signal(&m_condAction);
}

void ChessServer::idleWait()
{
  lock();
  while( m_eActionState == ActionStateIdle )
  {
    pthread_cond_wait(&m_condAction, &m_mutexAction);
  }
  unlock();
}

void *ChessServer::actionThread(void *pArg)
{
  ChessServer *pThis = (ChessServer *)pArg;
  int       oldstate;
  int       rc;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);
  //pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &oldstate);

  ROS_INFO("%s: Node action thread created.",
      ros::this_node::getName().c_str());

  while( pThis->m_eActionState != ActionStateExit )
  {
    pThis->idleWait();

    switch( pThis->m_eActionState )
    {
      case ActionStateWorking:
        pThis->m_execAction();
        pThis->lock();
        pThis->m_eActionState = ActionStateIdle;
        pThis->unlock();
        break;
      case ActionStateIdle:
      case ActionStateExit:
      default:
        break;
    }
  }

  pThis->m_eActionState = ActionStateExit;

  ROS_INFO("%s: Node action thread exited.", ros::this_node::getName().c_str());

  return NULL;
}
#endif // INC_ACTION_THREAD


//..............................................................................
// Support
//..............................................................................

void ChessServer::toMsgMove(const Move              &move,
                            chess_server::ChessMove &msgMove)
{
  msgMove.move_num         = move.m_nMove;
  msgMove.player.color     = move.m_player;
  msgMove.AN               = move.m_strAN;
  msgMove.moved.piece      = move.m_piece;
  msgMove.src.file         = move.m_posFrom.m_file;
  msgMove.src.rank         = move.m_posFrom.m_rank;
  msgMove.dst.file         = move.m_posTo.m_file;
  msgMove.dst.rank         = move.m_posTo.m_rank;
  msgMove.captured.piece   = move.m_captured;
  msgMove.en_passant       = move.m_en_passant;
  msgMove.castle.side      = move.m_castle;
  msgMove.aux_src.file     = move.m_posAuxAt.m_file;
  msgMove.aux_src.rank     = move.m_posAuxAt.m_rank;
  msgMove.aux_dst.file     = move.m_posAuxTo.m_file;
  msgMove.aux_dst.rank     = move.m_posAuxTo.m_rank;
  msgMove.promotion.piece  = move.m_promotion;
  msgMove.check            = move.m_check;
  msgMove.winner.color     = move.m_winner;
  msgMove.result.code      = move.m_result;
}

void ChessServer::stampHeader(std_msgs::Header &header,
                              u32_t             nSeqNum,
                              const string     &strFrameId)
{
  header.seq      = nSeqNum;
  header.stamp    = ros::Time::now();
  header.frame_id = strFrameId;
}
