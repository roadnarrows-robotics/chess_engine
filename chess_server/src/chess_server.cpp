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
#include "chess_thread.h"
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
    m_threadTask(m_chess)
{
  initPubVars();

  initSvcSeq();

  initAutoPlay();
}

ChessServer::~ChessServer()
{
}

int ChessServer::initialize()
{
  int   rc;

  if( (rc =  m_chess.initialize()) == CS_OK )
  {
    rc = m_threadTask.createThread();
  }
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

  endAutoPlay("New game started", -chess_engine::CE_ECODE_INTR);

  endMoveSvcSeq(true);

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

  //
  // Pull out fields from request.
  //
  ePlayer = chess_engine::tr(req.player);

  //
  // Fill move with known information.
  //
  move.m_ePlayer = ePlayer;
  move.m_strAN   = req.AN;

  //
  // Make a synchronous chess move.
  //
  rc = makeAMove(SvcNameMakeAMoveAN, move, true);

  //
  // Copy move to response.
  //
  chess_engine::copyMoveToMsg(move, rsp.move);

  //
  // Return true since response message is valid. The move constains the
  // (error) result code.
  //
  return true;
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

  //
  // Pull out fields from request.
  //
  ePlayer = chess_engine::tr(req.player);

  chess_engine::copyMsgToPos(req.src, posSrc);
  chess_engine::copyMsgToPos(req.dst, posDst);

  ePiecePromoted = chess_engine::tr(req.promoted);

  //
  // Fill move with known information.
  //
  move.m_ePlayer        = ePlayer;
  move.m_posSrc         = posSrc;
  move.m_posDst         = posDst;
  move.m_ePiecePromoted = ePiecePromoted;
 
  //
  // Make a synchronous chess move.
  //
  rc = makeAMove(SvcNameMakeAMove, move, true);

  //
  // Copy move to response.
  //
  chess_engine::copyMoveToMsg(move, rsp.move);

  //
  // Return true since response message is valid. The move constains the
  // (error) result code.
  //
  return true;
}

bool ChessServer::computeEnginesMove(ComputeEnginesMove::Request  &req,
                                     ComputeEnginesMove::Response &rsp)
{
  chess_player  ePlayer;
  chess_move    move;
  int           rc;

  ROS_DEBUG_STREAM(SvcNameComputeEnginesMove);

  //
  // Fill move with known information.
  //
  move.m_ePlayer = m_chess.whoseTurn();

  //
  // Make a synchronous chess move.
  //
  rc = computeEnginesMove(SvcNameComputeEnginesMove, move, true);

  //
  // Copy move to response.
  //
  chess_engine::copyMoveToMsg(move, rsp.move);

  //
  // Return true since response message is valid. The move constains the
  // (error) result code.
  //
  return true;
}

bool ChessServer::markMoveCompleted(MoveCompleted::Request  &req,
                                    MoveCompleted::Response &rsp)
{
  chess_player  ePlayer;
  int           rc;

  ROS_DEBUG_STREAM(SvcNameMarkMoveCompleted);

  ePlayer = chess_engine::tr(req.player);

  if( m_svcseq.m_eMoveSeqState != MoveSeqStateResult )
  {
    rc = -chess_engine::CE_ECODE_NO_EXEC;
  }

  else if( m_svcseq.m_ePlayerMakingMove == ePlayer )
  {
    rc = -chess_engine::CE_ECODE_CHESS_OUT_OF_TURN;
  }

  else
  {
    markMoveSvcSeq(MoveSeqStateMarked);

    endMoveSvcSeq();

    rc = CS_OK;
  }

  if( rc == CS_OK )
  {
    ROS_INFO_STREAM(SvcNameMarkMoveCompleted << ": "
        << chess_engine::nameOfColor(ePlayer) << " "
        << "marked move as completed.");
  }

  else
  {
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

  endAutoPlay("Player resigned", -chess_engine::CE_ECODE_INTR);

  ePlayer = chess_engine::tr(req.player);

  rc = m_chess.resign(ePlayer);

  if( rc == CS_OK )
  {
    m_pub.m_bPubEndOfGame = true;

    endMoveSvcSeq(true);

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
  bool    bRun;
  int     rc;

  ROS_DEBUG_STREAM(SvcNameAutoPlay);

  bRun = req.run;

  //
  // Begin autoplay
  //
  if( bRun )
  {
    int     nNumPlies = (int)req.num_plies;
    double  fDelay    = (double)req.delay;

    rc = beginAutoPlay(nNumPlies, fDelay);

    if( rc == CS_OK )
    {
      ROS_INFO_STREAM(SvcNameAutoPlay << ": "
        << "run = " << chess_engine::nameOfBool(bRun) << ", "
        << "num_plies = " << nNumPlies << ", "
        << "delay = " << fDelay << ".");

    }
    else
    {
      ROS_ERROR_STREAM(SvcNameAutoPlay << ": "
        << "run = " << chess_engine::nameOfBool(bRun) << ", "
        << chess_engine::strecode(rc) << "(" << rc << ")");
    }
  }

  //
  // End autoplay
  //
  else
  {
    endAutoPlay("Stop requested");

    ROS_INFO_STREAM(SvcNameAutoPlay << ": "
        << "run = " << chess_engine::nameOfBool(bRun) << ".");

    rc = CS_OK;
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

  rsp.whose_turn  = chess_engine::msg( m_chess.whoseTurn() ); 
  rsp.num_moves   = getNumOfMovesPlayed();
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
    rsp.SAN.push_back(move.m_strAN);
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
  nNumOfPlies = isMoveSvcSeqBusy()? m_svcseq.m_nPlyNum - 1: m_svcseq.m_nPlyNum;

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
// Service Sequencing States
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessServer::initSvcSeq()
{
  m_svcseq.m_eMoveSeqState      = MoveSeqStateIdle;
  m_svcseq.m_ePlayerMakingMove  = chess_engine::NoColor;
  m_svcseq.m_nPlyNum            = 0;
}

void ChessServer::beginMoveSvcSeq(chess_player ePlayer)
{
  m_svcseq.m_eMoveSeqState      = MoveSeqStateStart;
  m_svcseq.m_ePlayerMakingMove  = ePlayer;
  m_svcseq.m_nPlyNum            = getNumOfPliesPlayed() + 1;
}

void ChessServer::markMoveSvcSeq(const MoveSeqState eState)
{
  m_svcseq.m_eMoveSeqState = eState;
}

void ChessServer::endMoveSvcSeq(bool bAbort)
{
  // abort todo 
 
  m_svcseq.m_eMoveSeqState = MoveSeqStateIdle;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Asynchronous Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessServer::asyncMakeAMove(const chess_player ePlayer, const string &strAN)
{
  static const string strEvent("AsyncMakeAMove");

  chess_move  move;
  int         rc;

  //
  // Fill move with known information.
  //
  move.m_ePlayer = m_chess.whoseTurn();
  move.m_strAN   = strAN;

  //
  // Make asynchronous chess move.
  //
  rc = makeAMove(strEvent, move, false);

  return rc;
}

int ChessServer::asyncComputeEnginesMove()
{
  static const string strEvent("AsyncComputeEnginesMove");

  chess_move  move;
  int         rc;

  //
  // Fill move with known information.
  //
  move.m_ePlayer = m_chess.whoseTurn();

  //
  // Make asynchronous chess move.
  //
  rc = computeEnginesMove(strEvent, move, false);

  return rc;
}

int ChessServer::asyncGetLastMove(chess_move &move)
{
  return m_threadTask.getLastMove(move);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Autoplay
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessServer::initAutoPlay()
{
  m_autoplay.m_bRun         = false;
  m_autoplay.m_nNumPlies    = 0;
  m_autoplay.m_nLastPlyNum  = 0;
  m_autoplay.m_fDelay       = 0.0;
  m_autoplay.m_rcAutoPlay   = CS_OK;
}

int ChessServer::beginAutoPlay(int nNumPlies, double fDelay)
{
  int   rc;

  m_autoplay.m_timer.stop();

  // make sure there is a game
  if( !isPlayingAGame() )
  {
    rc = -chess_engine::CE_ECODE_CHESS_NO_GAME;
    m_autoplay.m_rcAutoPlay = rc;
  }

  // cannot start autoplay if a move is already in progress
  else if( isMoveInProgress() )
  {
    rc = -chess_engine::CE_ECODE_BUSY;
    m_autoplay.m_rcAutoPlay = rc;
  }

  else
  {
    rc = CS_OK;

    m_autoplay.m_bRun         = true;
    m_autoplay.m_nNumPlies    = nNumPlies;
    m_autoplay.m_nLastPlyNum  = getNumOfPliesPlayed() + nNumPlies;
    m_autoplay.m_fDelay       = fDelay;
    m_autoplay.m_rcAutoPlay   = rc;

    ROS_INFO_STREAM("Autoplay started.");

    execAutoPlay();
  }

  return rc;
}

bool ChessServer::isInAutoPlay()
{
  return m_autoplay.m_bRun;
}

void ChessServer::endAutoPlay(const string &strReason, const int nReasonCode)
{
  if( m_autoplay.m_bRun )
  {
    m_autoplay.m_bRun = false;

    m_autoplay.m_timer.stop();

    m_autoplay.m_rcAutoPlay = nReasonCode <= 0? nReasonCode: -nReasonCode;

    ROS_INFO_STREAM("Autoplay stopped: " << strReason << ".");
  }
}

int ChessServer::getAutoPlayReasonCode()
{
  return m_autoplay.m_rcAutoPlay;
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

    rc = computeEnginesMove("AutoPlay", move, true);

    tMove = ros::WallTime::now().toSec();

    if( isPlayingAGame() )
    {
      tDelta = tMove - tStart;
      tDelay = m_autoplay.m_fDelay - tDelta;

      if( tDelay < tMin )
      {
        tDelay = tMin;
      }

      if( (m_autoplay.m_nNumPlies == 0) ||
          (m_autoplay.m_nLastPlyNum < getNumOfPliesPlayed()) )
      {
        m_autoplay.m_timer = m_nh.createWallTimer(
                    ros::WallDuration(tDelay),
                    boost::bind(&ChessServer::cbAutoPlay, this, _1),
                    true);
      }
      else
      {
        stringstream ss;
        ss  << "Requested "
            << m_autoplay.m_nNumPlies
            << " plies (1/2 moves) autoplayed";
        endAutoPlay(ss.str());
      }
    }

    else
    {
      stringstream ss;
      ss  << "Game ended after "
          << getNumOfPliesPlayed()
          << " plies (1/2 moves)";
      endAutoPlay(ss.str());
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
// Move Execution
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessServer::makeAMove(const string &strEvent,
                           chess_move   &move,
                           bool         bSynchronous)
{
  bool  bIsANMove;
  int   rc;

  endAutoPlay("Make a move requested", -chess_engine::CE_ECODE_INTR);

  // Determine if move specifed by algebraic notation or by source/destination.
  bIsANMove = !move.m_strAN.empty();

  //
  // Make sure there is a game in play
  //
  if( !isPlayingAGame() )
  {
    move.m_eResult = chess_engine::NoGame;
    rc = -chess_engine::CE_ECODE_CHESS_NO_GAME;
  }

  //
  // Make sure no move is already in progress
  //
  else if( isMoveInProgress() )
  {
    move.m_eResult = chess_engine::Busy;
    rc = -chess_engine::CE_ECODE_BUSY;
  }

  //
  // Schedule the move and, optionally, synchronously wait for completion.
  //
  else
  {
    beginMoveSvcSeq(move.m_ePlayer);

    if( bIsANMove )
    {
      rc = m_threadTask.schedMoveToMake(move.m_ePlayer, move.m_strAN);
    }
    else
    {
      rc = m_threadTask.schedMoveToMake(move.m_ePlayer,
                                        move.m_posSrc, move.m_posDst,
                                        move.m_ePiecePromoted);
    }

    switch( rc )
    {
      case CS_OK:
        if( bSynchronous )
        {
          if( (rc = m_threadTask.waitForMove(move)) == CS_OK )
          {
            markMoveSvcSeq(MoveSeqStateResult);
          }
        }
        break;
      case -chess_engine::CE_ECODE_BUSY:
        move.m_eResult = chess_engine::Busy;
        break;
      case -chess_engine::CE_ECODE_NO_EXEC:
      default:
        move.m_eResult = chess_engine::GameFatal;
        break;
    }

    if( rc != CS_OK )
    {
      endMoveSvcSeq(true);
    }
  }

  //
  // Success
  //
  if( rc == CS_OK )
  {
    // move can result in the end of the game
    if( !isPlayingAGame() )
    {
      m_pub.m_bPubEndOfGame = true;
      endMoveSvcSeq();
    }

    ROS_INFO_STREAM(strEvent << ": "
        << move.m_nMoveNum << ". "
        << nameOfColor(move.m_ePlayer) << " "
        << move.m_strAN << ".");
  }

  //
  // Failure
  //
  else
  {
    ROS_ERROR_STREAM(strEvent << ": "
        << nameOfColor(move.m_ePlayer) << " "
        << move.m_strAN << " "
        << move.m_posSrc << move.m_posDst << ": "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  return rc;
}

int ChessServer::computeEnginesMove(const string &strEvent,
                                    chess_move   &move,
                                    bool         bSynchronous)
{
  int   rc;

  endAutoPlay("Compute engine's move requested", -chess_engine::CE_ECODE_INTR);

  //
  // Make sure there is a game in play
  //
  if( !isPlayingAGame() )
  {
    move.m_eResult = chess_engine::NoGame;
    rc = -chess_engine::CE_ECODE_CHESS_NO_GAME;
  }

  //
  // Make sure no move is already in progress
  //
  else if( isMoveInProgress() )
  {
    move.m_eResult = chess_engine::Busy;
    rc = -chess_engine::CE_ECODE_BUSY;
  }

  //
  // Schedule the move and synchronously wait for completion
  //
  else
  {
    beginMoveSvcSeq(move.m_ePlayer);

    rc = m_threadTask.schedMoveToCompute(move.m_ePlayer);

    switch( rc )
    {
      case CS_OK:
        if( bSynchronous )
        {
          if( (rc = m_threadTask.waitForMove(move)) == CS_OK )
          {
            markMoveSvcSeq(MoveSeqStateResult);
          }
        }
        break;
      case -chess_engine::CE_ECODE_BUSY:
        move.m_eResult = chess_engine::Busy;
        break;
      case -chess_engine::CE_ECODE_NO_EXEC:
      default:
        move.m_eResult = chess_engine::GameFatal;
        break;
    }

    if( rc != CS_OK )
    {
      endMoveSvcSeq(true);
    }
  }

  //
  // Success
  //
  if( rc == CS_OK )
  {
    // move can end the game
    if( !isPlayingAGame() )
    {
      m_pub.m_bPubEndOfGame = true;
      endMoveSvcSeq();
    }

    ROS_INFO_STREAM(strEvent << ": "
        << move.m_nMoveNum << ". "
        << nameOfColor(move.m_ePlayer) << " "
        << move.m_strAN << ".");
  }

  //
  // Failure
  //
  else
  {
    ROS_ERROR_STREAM(strEvent << ": "
        << nameOfColor(move.m_ePlayer) << " "
        << chess_engine::strecode(rc) << "(" << rc << ")");
  }

  return rc;
}


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
