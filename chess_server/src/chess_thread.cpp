////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_thread.cpp
//
/*! \file
 *
 * \brief The ROS chess_server chess thread.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows
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

// chess engine library
#include "chess_engine/ceTypes.h"
#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"

// chess_server
#include "chess_thread.h"

using namespace std;
using namespace chess_server;

/*!
 * \brief Helpful typedefs
 */
typedef chess_engine::ChessPos    chess_pos;
typedef chess_engine::ChessColor  chess_color;
typedef chess_engine::ChessColor  chess_player;
typedef chess_engine::ChessPiece  chess_piece;
typedef chess_engine::ChessMove   chess_move;

static const string ThreadName("Task thread");


//------------------------------------------------------------------------------
// ChessThread Class
//------------------------------------------------------------------------------

ChessThread::ChessThread(chess_engine::Chess &chess) : m_chess(chess)
{
  m_eState = ThreadStateUninit;
}

ChessThread::~ChessThread()
{
  destroyThread();
}

int ChessThread::createThread()
{
  int   rc;

  m_eState  = ThreadStateUninit;

  pthread_mutex_init(&m_mutex, NULL);
  pthread_cond_init(&m_cond,   NULL);
  
  rc = pthread_create(&m_threadChess, NULL, threadMain, (void*)this);
 
  if( rc == 0 )
  {
    rc = chess_engine::CE_OK;
  }

  else
  {
    m_eState = ThreadStateUninit;
    pthread_cond_destroy(&m_cond);
    pthread_mutex_destroy(&m_mutex);
    rc = -chess_engine::CE_ECODE_SYS;
    ROS_ERROR("pthread_create(ChessThread)");
  }

  return rc;
}

void ChessThread::destroyThread()
{
  signalStateChange(ThreadStateExiting);

  pthread_cancel(m_threadChess);
  pthread_join(m_threadChess, NULL);

  pthread_cond_destroy(&m_cond);
  pthread_mutex_destroy(&m_mutex);

  m_eState = ThreadStateUninit;

  ROS_INFO_STREAM(ros::this_node::getName() << ": " << ThreadName
      << " destroyed.");
}

int ChessThread::schedMoveToMake(const chess_player ePlayer,
                                 const string       &strAN)
{
  int   rc;

  lock();

  m_argsMove.m_ePlayer  = ePlayer;
  m_argsMove.m_strAN    = strAN;

  rc = schedTask(boost::bind(&ChessThread::execMakeAMoveAN, this));

  unlock();

  return rc;
}

int ChessThread::schedMoveToMake(const chess_player ePlayer,
                                 const chess_pos    &posSrc,
                                 const chess_pos    &posDst,
                                 const chess_piece  ePiecePromoted)
{
  int   rc;

  lock();

  m_argsMove.m_ePlayer        = ePlayer;
  m_argsMove.m_posSrc         = posSrc;
  m_argsMove.m_posDst         = posDst;
  m_argsMove.m_ePiecePromoted = ePiecePromoted;

  rc = schedTask(boost::bind(&ChessThread::execMakeAMove, this));

  unlock();

  return rc;
}

int ChessThread::schedMoveToCompute(const chess_player ePlayer)
{
  int   rc;

  lock();

  m_argsMove.m_ePlayer  = ePlayer;

  rc = schedTask(boost::bind(&ChessThread::execComputeEnginesMove, this));

  unlock();

  return rc;
}

int ChessThread::schedUserTask(boost::function<void()> task)
{
  int   rc;

  lock();

  rc = schedTask(task);

  unlock();

  return rc;
}

int ChessThread::schedTask(boost::function<void()> task)
{
  int   rc;

  switch( m_eState )
  {
    case ThreadStateIdle:
      m_execTask = task;
      signalStateChange(ThreadStateWorking);
      rc = chess_engine::CE_OK;
      break;
    case ThreadStateWorking:
      rc = -chess_engine::CE_ECODE_BUSY;
      break;
    case ThreadStateUninit:
    case ThreadStateExiting:
    default:
      rc = -chess_engine::CE_ECODE_NO_EXEC;
      break;
  }

  return rc;
}

int ChessThread::waitForMove(chess_move &move)
{
  int   rc;

  lock();

  while( m_eState == ThreadStateWorking )
  {
    pthread_cond_wait(&m_cond, &m_mutex);
  }

  move = m_move;
  rc   = m_rcMove;

  unlock();

  return rc;
}

int ChessThread::getLastMove(chess_move &move)
{
  int   rc;

  lock();

  move = m_move;
  rc   = m_eState != m_rcMove? ThreadStateWorking: -chess_engine::CE_ECODE_BUSY;

  unlock();

  return rc;
}

void ChessThread::signalStateChange(ThreadState eNewState)
{
  m_eState = eNewState;
  pthread_cond_signal(&m_cond);
}

void ChessThread::idleWait()
{
  lock();
  while( m_eState == ThreadStateIdle )
  {
    pthread_cond_wait(&m_cond, &m_mutex);
  }
  unlock();
}

void *ChessThread::threadMain(void *pArg)
{
  ChessThread *pThis = (ChessThread *)pArg;
  int         oldstate;
  int         rc;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);
  //pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &oldstate);

  ROS_INFO_STREAM(ros::this_node::getName() << ": " << ThreadName
      << " created.");

  pThis->m_eState = ThreadStateIdle;

  while( pThis->m_eState != ThreadStateExiting )
  {
    pThis->idleWait();

    switch( pThis->m_eState )
    {
      case ThreadStateWorking:
        pThis->m_execTask();
        pThis->signalStateChange(ThreadStateIdle);
        break;

      case ThreadStateUninit:
        pThis->signalStateChange(ThreadStateExiting);
        break;

      case ThreadStateIdle:
      case ThreadStateExiting:
      default:
        break;
    }
  }

  ROS_INFO_STREAM(ros::this_node::getName() << ": " << ThreadName << "exited.");

  return NULL;
}

void ChessThread::execMakeAMoveAN()
{
  m_rcMove = m_chess.makeAMove(m_argsMove.m_ePlayer,
                               m_argsMove.m_strAN,
                               m_move);
}

void ChessThread::execMakeAMove()
{
  m_rcMove = m_chess.makeAMove(m_argsMove.m_ePlayer,
                               m_argsMove.m_posSrc,
                               m_argsMove.m_posDst,
                               m_argsMove.m_ePiecePromoted,
                               m_move);
}

void ChessThread::execComputeEnginesMove()
{
  m_rcMove = m_chess.computeEnginesMove(m_move);
}
