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

#include <sstream>
#include <string>
#include <map>

#include <boost/bind.hpp>

#include "ros/ros.h"

// published messages
#include "chess_server/ChessNewGameStatus.h"
#include "chess_server/ChessMoveStamped.h"
#include "chess_server/ChessEndGameStatus.h"

// services
#include "chess_server/StartNewGame.h"
#include "chess_server/MakeAMove.h"
#include "chess_server/MakeAMoveAN.h"
#include "chess_server/ComputeEnginesMove.h"
#include "chess_server/MoveCompleted.h"
#include "chess_server/Resign.h"
#include "chess_server/AutoPlay.h"
#include "chess_server/SetDifficulty.h"
#include "chess_server/GetGameState.h"
#include "chess_server/GetBoardState.h"
#include "chess_server/GetPlayHistory.h"

// chess engine library
#include "chess_engine/ceTypes.h"
#include "chess_engine/ceChess.h"
#include "chess_engine/ceRosMsgs.h"

// chess_server
#include "chess_server.h"


using namespace std;
using namespace chess_server;

/*!
 * \brief Advertised service names.
 */
static const string SvcNameStartNewGame("start_new_game");
static const string SvcNameMakeAMove("make_a_move");
static const string SvcNameMakeAMoveAN("make_a_move_an");
static const string SvcNameComputeEnginesMove("compute_engines_move");
static const string SvcNameMarkMoveCompleted("mark_move_completed");
static const string SvcNameResign("resign");
static const string SvcNameAutoPlay("autoplay");
static const string SvcNameSetDifficulty("set_difficulty");
static const string SvcNameGetGameState("get_game_state");
static const string SvcNameGetBoardState("get_board_state");
static const string SvcNameGetPlayHistory("get_play_history");

/*!
 * \brief Advertised published topic names.
 */
static const string PubNameNewGameStatus("new_game_status");
static const string PubNameMoveStatus("move_status");
static const string PubNameEndOfGameStatus("end_of_game_status");

/*!
 * \brief Registered action server names.
 */
static const string AsNameAutoPlay("auto_play_action");
static const string AsNameComputeEnginesMove("compute_engines_move_action");

/*!
 * \brief Helpful typedefs
 */
typedef chess_engine::ChessFile               chess_file;
typedef chess_engine::ChessRank               chess_rank;
typedef chess_engine::ChessPos                chess_pos;
typedef chess_engine::ChessColor              chess_color;
typedef chess_engine::ChessColor              chess_player;
typedef chess_engine::ChessPiece              chess_piece;
typedef chess_engine::ChessSquare             chess_square;
typedef chess_engine::ChessBoard              chess_board;
typedef chess_engine::ChessMove               chess_move;
typedef chess_engine::ChessGame::ChessHistory chess_history;
typedef chess_history::const_iterator         chess_history_iterator;

/*
 * \brief Helpful values.
 */
static const int CS_OK = chess_engine::CE_OK; ///< a okay


//------------------------------------------------------------------------------
// ChessServer Class
//------------------------------------------------------------------------------

ChessServer::ChessServer(ros::NodeHandle &nh) :
  m_nh(nh),
  m_asAutoPlay(AsNameAutoPlay, *this)
{
  initPubVars();

  initSvcSeq();

  initAutoPlay();

#ifdef INC_ACTION_THREAD
  createActionThread();
#endif // INC_ACTION_THREAD
}

ChessServer::~ChessServer()
{
#ifdef INC_ACTION_THREAD
  destroyActionThread();
#endif // INC_ACTION_THREAD
}

int ChessServer::initializeChess()
{
  return m_chess.initialize();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessServer::advertiseServices()
{
  int   cnt = 0;

  m_services[SvcNameStartNewGame] =
            m_nh.advertiseService(SvcNameStartNewGame,
                                  &ChessServer::startNewGame,
                                  &(*this));
  ++cnt;

  m_services[SvcNameMakeAMove] =
            m_nh.advertiseService(SvcNameMakeAMove,
                                  &ChessServer::makeAMove,
                                  &(*this));
  ++cnt;

  m_services[SvcNameMakeAMoveAN] =
            m_nh.advertiseService(SvcNameMakeAMoveAN,
                                  &ChessServer::makeAMoveAN,
                                  &(*this));
  ++cnt;

  m_services[SvcNameComputeEnginesMove] =
            m_nh.advertiseService(SvcNameComputeEnginesMove,
                                  &ChessServer::computeEnginesMove,
                                  &(*this));
  ++cnt;

  m_services[SvcNameMarkMoveCompleted] =
            m_nh.advertiseService(SvcNameMarkMoveCompleted,
                                  &ChessServer::markMoveCompleted,
                                  &(*this));
  ++cnt;

  m_services[SvcNameResign] =
            m_nh.advertiseService(SvcNameResign,
                                  &ChessServer::resign,
                                  &(*this));
  ++cnt;

  m_services[SvcNameAutoPlay] =
            m_nh.advertiseService(SvcNameAutoPlay,
                                  &ChessServer::autoplay,
                                  &(*this));
  ++cnt;

  m_services[SvcNameSetDifficulty] =
            m_nh.advertiseService(SvcNameSetDifficulty,
                                  &ChessServer::setDifficulty,
                                  &(*this));
  ++cnt;

  m_services[SvcNameGetGameState] =
            m_nh.advertiseService(SvcNameGetGameState,
                                  &ChessServer::getGameState,
                                  &(*this));
  ++cnt;

  m_services[SvcNameGetBoardState] =
            m_nh.advertiseService(SvcNameGetBoardState,
                                  &ChessServer::getBoardState,
                                  &(*this));
  ++cnt;

  m_services[SvcNameGetPlayHistory] =
            m_nh.advertiseService(SvcNameGetPlayHistory,
                                  &ChessServer::getPlayHistory,
                                  &(*this));
  ++cnt;

  return cnt;
}

bool ChessServer::startNewGame(StartNewGame::Request  &req,
                               StartNewGame::Response &rsp)
{
  string    strWhite;
  string    strBlack;
  int       rc;

  ROS_DEBUG_STREAM(SvcNameStartNewGame);

  endAutoPlay("new game started");

  rc = m_chess.startNewGame(req.white_name, req.black_name);

  if( rc == CS_OK )
  {
    initPubVars(true);

    initSvcSeq();

    m_chess.getPlayerNames(strWhite, strBlack);

    ROS_INFO_STREAM(SvcNameStartNewGame << ": "
        << "White = \"" << strWhite << "\", "
        << "Black = \"" << strBlack << "\".");
  }

  else
  {
    ROS_ERROR_STREAM(SvcNameStartNewGame << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  rsp.rc = (int8_t)rc;

  return true;  // has valid response regardless of success or failure
}

bool ChessServer::makeAMoveAN(MakeAMoveAN::Request  &req,
                              MakeAMoveAN::Response &rsp)
{
  chess_player  ePlayer;
  chess_move    move;
  int           rc;

  ROS_DEBUG_STREAM(SvcNameMakeAMoveAN);

  endAutoPlay("chess move requested");

  ePlayer = chess_engine::tr(req.player);

  // test if busy with previous move
  if( isMoveSvcBusy() )
  {
    move.m_ePlayer = ePlayer;
    move.m_eResult = chess_engine::Busy;

    rc = -chess_engine::CE_ECODE_BUSY;
  }

  // make the logical move
  else
  {
    rc = m_chess.makeAMove(ePlayer, req.AN, move);
  }

  if( rc == CS_OK )
  {
    ROS_INFO_STREAM(SvcNameMakeAMoveAN << ": "
        << move.m_nMoveNum << ". "
        << nameOfColor(move.m_ePlayer) << " "
        << move.m_strSAN << ".");

    beginMoveSvcSeq(ePlayer);

    if( !m_chess.isPlayingAGame() )
    {
      m_pub.m_bPubEndOfGame = true;

      endMoveSvcSeq();
    }
  }
  else
  {
    ROS_ERROR_STREAM(SvcNameMakeAMoveAN << ": "
        << nameOfColor(ePlayer) << " "
        << req.AN << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  chess_engine::copyMoveToMsg(move, rsp.move);

  return true;  // has valid response regardless of success or failure
}

bool ChessServer::makeAMove(MakeAMove::Request  &req,
                            MakeAMove::Response &rsp)
{
  chess_player  ePlayer;
  chess_pos     posSrc;
  chess_pos     posDst;
  chess_piece   ePiecePromoted;
  chess_move    move;
  int           rc;

  ROS_DEBUG_STREAM(SvcNameMakeAMove);

  endAutoPlay("chess move requested");

  ePlayer = chess_engine::tr(req.player);

  chess_engine::copyMsgToPos(req.src, posSrc);
  chess_engine::copyMsgToPos(req.dst, posDst);

  ePiecePromoted = chess_engine::tr(req.promoted);

  // test if busy with previous move
  if( isMoveSvcBusy() )
  {
    move.m_ePlayer        = ePlayer;
    move.m_posSrc         = posSrc;
    move.m_posDst         = posDst;
    move.m_ePiecePromoted = ePiecePromoted;
    move.m_eResult        = chess_engine::Busy;

    rc = -chess_engine::CE_ECODE_BUSY;
  }

  // make the logical move
  else
  {
    rc = m_chess.makeAMove(ePlayer, posSrc, posDst, ePiecePromoted, move);
  }

  if( rc == CS_OK )
  {
    ROS_INFO_STREAM(SvcNameMakeAMove << ": "
        << move.m_nMoveNum << ". "
        << nameOfColor(move.m_ePlayer) << " "
        << move.m_strSAN << ".");

    beginMoveSvcSeq(ePlayer);

    if( !m_chess.isPlayingAGame() )
    {
      m_pub.m_bPubEndOfGame = true;

      endMoveSvcSeq();
    }
  }
  else
  {
    ROS_ERROR_STREAM(SvcNameMakeAMove << ": "
        << nameOfColor(ePlayer) << " "
        << posSrc << posDst << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  copyMoveToMsg(move, rsp.move);

  return true;  // has valid response regardless of success or failure
}

bool ChessServer::computeEnginesMove(ComputeEnginesMove::Request  &req,
                                     ComputeEnginesMove::Response &rsp)
{
  chess_player  ePlayer;
  chess_move    move;
  int           rc;

  ROS_DEBUG_STREAM(SvcNameComputeEnginesMove);

  endAutoPlay("compute engine's move requested");

  ePlayer = m_chess.whoseTurn();

  // test if busy with previous move
  if( isMoveSvcBusy() )
  {
    move.m_ePlayer = ePlayer;
    move.m_eResult = chess_engine::Busy;

    rc = -chess_engine::CE_ECODE_BUSY;
  }

  // compute the logical next move
  else
  {
    rc = m_chess.computeEnginesMove(move);
  }

  if( rc == CS_OK )
  {
    ROS_INFO_STREAM(SvcNameComputeEnginesMove << ": "
        << move.m_nMoveNum << ". "
        << nameOfColor(move.m_ePlayer) << " "
        << move.m_strSAN << ".");

    beginMoveSvcSeq(ePlayer);

    if( !m_chess.isPlayingAGame() )
    {
      m_pub.m_bPubEndOfGame = true;

      endMoveSvcSeq();
    }
  }
  else
  {
    ROS_ERROR_STREAM(SvcNameComputeEnginesMove << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  chess_engine::copyMoveToMsg(move, rsp.move);

  return true;  // has valid response regardless of success or failure
}

bool ChessServer::markMoveCompleted(MoveCompleted::Request  &req,
                                    MoveCompleted::Response &rsp)
{
  chess_player  ePlayer;
  int           rc;

  ROS_DEBUG_STREAM(SvcNameMarkMoveCompleted);

  ePlayer = chess_engine::tr(req.player);

  if( m_svcseq.m_ePlayerMakingMove == ePlayer )
  {
    endMoveSvcSeq();

    ROS_INFO_STREAM(SvcNameMarkMoveCompleted << ": "
        << chess_engine::nameOfColor(ePlayer) << " "
        << "completed move.");

    rc = CS_OK;
  }
  else
  {
    rc = -chess_engine::CE_ECODE_CHESS_OUT_OF_TURN;

    ROS_ERROR_STREAM(SvcNameMarkMoveCompleted << ": "
        << chess_engine::nameOfColor(ePlayer) << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  rsp.rc = rc;

  return true;
}

bool ChessServer::resign(Resign::Request  &req,
                         Resign::Response &rsp)
{
  chess_player  ePlayer;
  int           rc;

  ROS_DEBUG_STREAM(SvcNameResign);

  endAutoPlay("player resigned");

  ePlayer = chess_engine::tr(req.player);

  rc = m_chess.resign(ePlayer);

  if( rc == CS_OK )
  {
    m_pub.m_bPubEndOfGame = true;

    endMoveSvcSeq();

    ROS_INFO_STREAM(SvcNameResign << ": " << nameOfColor(ePlayer) << ".");
  }

  else
  {
    ROS_ERROR_STREAM(SvcNameResign << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  rsp.rc = (int8_t)rc;

  return true;  // has valid response regardless of success or failure
}

bool ChessServer::autoplay(AutoPlay::Request  &req,
                           AutoPlay::Response &rsp)
{
  int   rc = CS_OK;

  ROS_DEBUG_STREAM(SvcNameAutoPlay);

  // test if busy with previous move
  if( isMoveSvcBusy() )
  {
    rc = -chess_engine::CE_ECODE_BUSY;
  }

  // no game
  else if( !m_chess.isPlayingAGame() )
  {
    rc = -chess_engine::CE_ECODE_CHESS_NO_GAME;
  }

  if( rc == CS_OK )
  {
    if( req.run )
    {
      ROS_INFO_STREAM(SvcNameAutoPlay << ": "
        << "run = " << chess_engine::nameOfBool(req.run) << ", "
        << "num_moves = " << req.num_moves << ", "
        << "delay = " << req.delay << ".");

      beginAutoPlay((int)req.num_moves, (double)req.delay);
    }
    else
    {
      ROS_INFO_STREAM(SvcNameAutoPlay << ": "
        << "run = " << chess_engine::nameOfBool(req.run) << ".");

      endAutoPlay("autoplay stopped");
    }
  }
  else
  {
    ROS_ERROR_STREAM(SvcNameAutoPlay << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  rsp.rc = rc;

  return true;
}

bool ChessServer::setDifficulty(SetDifficulty::Request  &req,
                                SetDifficulty::Response &rsp)
{
  float   fDifficulty;
  int     rc;

  ROS_DEBUG_STREAM(SvcNameSetDifficulty);

  fDifficulty = req.difficulty;

  rc = m_chess.setGameDifficulty(fDifficulty);

  if( rc == CS_OK )
  {
    ROS_INFO_STREAM(SvcNameSetDifficulty << ": "
        << "diffuculty = " << fDifficulty);
  }
  else
  {
    ROS_ERROR_STREAM(SvcNameSetDifficulty << ": "
        << "diffuculty = " << fDifficulty << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  rsp.rc = rc;

  return true;  // has valid response regardless of success or failure
}

bool ChessServer::getGameState(GetGameState::Request  &req,
                               GetGameState::Response &rsp)
{
  ROS_DEBUG_STREAM(SvcNameGetGameState);

  m_chess.getPlayerNames(rsp.white_name, rsp.black_name);

  rsp.num_moves   = m_chess.getNumOfMoves();
  rsp.game_winner = chess_engine::msg( m_chess.getWinner() ); 
  rsp.play_state  = chess_engine::msg( m_chess.getPlayState() );

  ROS_INFO_STREAM(SvcNameGetGameState << ": retrieved game state.");

  return true;
}

bool ChessServer::getBoardState(GetBoardState::Request  &req,
                                GetBoardState::Response &rsp)
{
  chess_board &board  = m_chess.getBoard();
  chess_file  file_a  = chess_engine::ChessFileA;
  chess_file  no_file = chess_engine::NoFile;
  chess_rank  rank_1  = chess_engine::ChessRank1;
  chess_rank  no_rank = chess_engine::NoRank;

  chess_file  file;
  chess_rank  rank;

  ROS_DEBUG_STREAM(SvcNameGetBoardState);

  for(rank = rank_1; rank != no_rank; rank = chess_board::nextRank(rank))
  {
    for(file = file_a; file != no_file; file = chess_board::nextFile(file))
    {
      chess_square &sq = board.at(file, rank);

      rsp.squares.push_back( chess_engine::msg(sq.getPos()) );
      rsp.colors.push_back( chess_engine::msg(sq.getPieceColor()) );
      rsp.pieces.push_back( chess_engine::msg(sq.getPieceType()) );
      rsp.ids.push_back( sq.getPieceId() );
    }
  }

  ROS_INFO_STREAM(SvcNameGetBoardState << ": retrieved board state.");

  return true;
}

bool ChessServer::getPlayHistory(GetPlayHistory::Request  &req,
                                 GetPlayHistory::Response &rsp)
{
  const chess_history     &history = m_chess.getGameHistory();
  chess_history_iterator  iter;
  
  ROS_DEBUG_STREAM(SvcNameGetPlayHistory);

  for(iter = history.begin(); iter != history.end(); ++iter)
  {
    const chess_move &move = *iter;

    rsp.player.push_back( chess_engine::msg(move.m_ePlayer) );
    rsp.SAN.push_back(move.m_strSAN);
  }

  ROS_INFO_STREAM(SvcNameGetPlayHistory << ": retrieved play history, "
        << "history_size = " << history.size() << ".");

  return true;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Topic Publishers
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessServer::advertisePublishers(int nQueueDepth)
{
  int   cnt = 0;

  m_publishers[PubNameNewGameStatus] =
      m_nh.advertise<ChessNewGameStatus>( PubNameNewGameStatus, nQueueDepth);
  ++cnt;

  m_publishers[PubNameMoveStatus] =
      m_nh.advertise<ChessMoveStamped>(PubNameMoveStatus, nQueueDepth);
  ++cnt;

  m_publishers[PubNameEndOfGameStatus] =
      m_nh.advertise<ChessEndGameStatus>(PubNameEndOfGameStatus, nQueueDepth);
  ++cnt;

  return cnt;
}

void ChessServer::initPubVars(const bool bNewGame)
{
  m_pub.m_bPubNewGame   = bNewGame;
  m_pub.m_nPubLastPly   = 0;
  m_pub.m_bPubEndOfGame = false;
}

void ChessServer::publish()
{
  int nNumOfPlies;

  // number of plies to publish except any busy move
  nNumOfPlies = isMoveSvcBusy()? m_svcseq.m_nPlyNum - 1: m_svcseq.m_nPlyNum;

  // publish new game status
  if( m_pub.m_bPubNewGame )
  {
    ChessNewGameStatus &msg = m_pub.m_msgNewGameStatus;

    stampHeader(msg.header, msg.header.seq+1, PubNameNewGameStatus);
    
    m_chess.getPlayerNames(msg.white_name, msg.black_name);

    m_publishers[PubNameNewGameStatus].publish(msg);

    m_pub.m_bPubNewGame = false;
  }

  // publish all new fully completed move(s)
  while( m_pub.m_nPubLastPly < nNumOfPlies )
  {
    ChessMoveStamped &msg = m_pub.m_msgMoveStamped;

    stampHeader(msg.header, msg.header.seq+1, PubNameMoveStatus);

    const chess_move &move=m_chess.getGame().getHistoryAt(m_pub.m_nPubLastPly);

    chess_engine::copyMoveToMsg(move, msg.move);

    m_publishers[PubNameMoveStatus].publish(msg);

    ++m_pub.m_nPubLastPly;
  }
  
  // publish end of game status
  if( m_pub.m_bPubEndOfGame )
  {
    ChessEndGameStatus &msg = m_pub.m_msgEndOfGameStatus;

    stampHeader(msg.header, msg.header.seq+1, PubNameEndOfGameStatus);

    msg.winner.color  = m_chess.getWinner();
    msg.result.code   = m_chess.getPlayState();

    m_publishers[PubNameEndOfGameStatus].publish(msg);

    m_pub.m_bPubEndOfGame = false;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessServer::subscribeToTopics()
{
  return 0;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Action Servers
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessServer::startActionServers()
{
  int   cnt = 0;

  m_asAutoPlay.start();

  ++cnt;
#if 0 // RDK
  ASGetEnginesMove m_asGetEnginesMove("get_engines_move_action", chessServer);
#endif // RDK

  return cnt;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Service Sequence State
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessServer::initSvcSeq()
{
  m_svcseq.m_bBusy              = false;
  m_svcseq.m_ePlayerMakingMove  = chess_engine::NoColor;
  m_svcseq.m_nPlyNum            = 0;
}

void ChessServer::beginMoveSvcSeq(chess_player ePlayer)
{
  m_svcseq.m_bBusy              = true;
  m_svcseq.m_ePlayerMakingMove  = ePlayer;
  m_svcseq.m_nPlyNum            = m_chess.getNumOfPlies();
}

void ChessServer::endMoveSvcSeq()
{
  m_svcseq.m_bBusy = false;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Autoplay
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessServer::initAutoPlay()
{
  m_autoplay.m_bRun         = false;
  m_autoplay.m_nNumMoves    = 0;
  m_autoplay.m_nLastMoveNum = 0;
  m_autoplay.m_fDelay       = 0.0;
}

void ChessServer::beginAutoPlay(int nNumMoves, double fDelay)
{
  m_autoplay.m_timer.stop();

  m_autoplay.m_bRun         = true;
  m_autoplay.m_nNumMoves    = nNumMoves;
  m_autoplay.m_nLastMoveNum = m_chess.getNumOfMoves() + nNumMoves;
  m_autoplay.m_fDelay       = fDelay;

  ROS_INFO_STREAM("Autoplay started.");

  execAutoPlay();
}

void ChessServer::endAutoPlay(const string &strReason)
{
  if( m_autoplay.m_bRun )
  {
    m_autoplay.m_bRun = false;

    m_autoplay.m_timer.stop();

    ROS_INFO_STREAM("Autoplay stopped: " << strReason << ".");
  }
}

void ChessServer::execAutoPlay()
{
  static double tMin = 0.01;

  double      tStart, tMove, tDelta, tDelay;
  chess_move  move;
  int         rc;

  if( m_autoplay.m_bRun )
  {
    tStart = ros::WallTime::now().toSec();

    rc = m_chess.computeEnginesMove(move);

    tMove = ros::WallTime::now().toSec();

    if( m_chess.isPlayingAGame() )
    {
      tDelta = tMove - tStart;
      tDelay = m_autoplay.m_fDelay - tDelta;

      if( tDelay < tMin )
      {
        tDelay = tMin;
      }

      if( (m_autoplay.m_nNumMoves == 0) ||
          (m_autoplay.m_nLastMoveNum < m_chess.getNumOfMoves()) )
      {

        m_autoplay.m_timer = m_nh.createWallTimer(
                    ros::WallDuration(tDelay),
                    boost::bind(&ChessServer::cbAutoPlay, this, _1),
                    true);
      }
      else
      {
        stringstream ss;
        ss << m_autoplay.m_nNumMoves << " moves autoplayed";
        endAutoPlay(ss.str());
      }
    }

    else
    {
      endAutoPlay("game ended");
    }
  }
}

void ChessServer::cbAutoPlay(const ros::WallTimerEvent& event)
{
  if( m_autoplay.m_bRun )
  {
    execAutoPlay();
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Action thread
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
    rc = CS_OK;
  }

  else
  {
    m_eActionState = ActionStateExit;
    pthread_cond_destroy(&m_condAction);
    pthread_mutex_destroy(&m_mutexAction);
    rc = -chess_engine::CE_ECODE_SYS;
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
    rc = CS_OK;
  }
  else
  {
    rc = -chess_engine::CE_ECODE_NO_EXEC;
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


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Support
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessServer::stampHeader(std_msgs::Header &header,
                              uint32_t         nSeqNum,
                              const string     &strFrameId)
{
  header.seq      = nSeqNum;
  header.stamp    = ros::Time::now();
  header.frame_id = strFrameId;
}
