////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceChess.cpp
//
/*! \file
 *
 * \brief Top level chess class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 *
 * \par License:
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

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/console.h>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceEngine.h"
#include "chess_engine/ceParser.h"
#include "chess_engine/ceGame.h"
#include "chess_engine/ceChess.h"

using namespace std;
using namespace chess_engine;

// ----------------------------------------------------------------------------
// Class Chess
// ----------------------------------------------------------------------------

Chess::Chess()
{
}

Chess::~Chess()
{
}

int Chess::initialize()
{
  return m_engine.openConnection();
}

int Chess::startNewGame(const string &strWhite, const string &strBlack)
{
  int   rc;

  if( (rc = m_engine.startNewGame()) == CE_OK )
  {
    rc = m_game.startNewGame(strWhite, strBlack);
  }

  return rc;
}

int Chess::endCurrentGame(ChessResult eReason, ChessColor eWinner)
{
  int   rc;

  if( (rc = m_engine.endCurrentGame(eReason, eWinner)) == CE_OK )
  {
    rc = m_game.endCurrentGame(eReason, eWinner);
  }

  return rc;
}

int Chess::resign(const ChessColor ePlayer)
{
  int rc;

  if( (rc = m_engine.resign(ePlayer)) == CE_OK )
  {
    rc = m_game.endCurrentGame(Resign, opponent(ePlayer));
  }

  return rc;
}

int Chess::makeAMove(const ChessColor ePlayer,
                     const ChessPos   &posSrc,
                     const ChessPos   &posDst,
                     const ChessPiece ePiecePromoted,
                     ChessMove        &move)
{
  string  strCAN;

  strCAN = ChessMove::CAN(posSrc, posDst, ePiecePromoted);

  return makeAMove(ePlayer, strCAN, move);
}

int Chess::makeAMove(const ChessColor ePlayer,
                     const string     &strAN,
                     ChessMove        &move)
{
  static  string strFunc("Chess::makeAMove");

  int     nMoveNum;
  string  strSAN(strAN);
  int     rc;

  // start fresh
  move.clear();

  // Move number. Get this value before making a move.
  nMoveNum = m_engine.getMoveNumInPlay();

  // Make player's move. On success strSAN is updated.
  rc = m_engine.makePlayersMove(ePlayer, strSAN);

  // Fill in critical move fields.
  move.m_nMoveNum = nMoveNum;
  move.m_ePlayer  = ePlayer;
  move.m_strSAN   = strSAN;
  move.m_eResult  = Ok;

  // Failed backend move.
  if( rc != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc
          << ": makePlayersMove(" << nameOfColor(ePlayer) << ", "
          << strAN << ", SAN): "
          << strerror(rc)
          << ".");
  }

  // Parse SAN and fill in appropriate move fields.
  else if( (rc = m_parser.parse(strSAN, move)) != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc
          << ": parse(" << strSAN
          << ", move): "
          << m_parser.getErrorStr()
          << ".");
  }

  // Qualify move against current game state, setting additional fields.
  else if( (rc = m_game.qualifyMove(move)) != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc << ": qualifyMove(move): " << strerror(rc) << ".");
  }

  // Now execute the move on the mirrored game state.
  if( (rc = m_game.execMove(move)) != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc << ": execMove(move): " << strerror(rc) << ".");
  }

  if( rc != CE_OK )
  {
    move.m_eResult = rcToMoveResult(rc);
  }

  // Finalize move/game result.
  move.m_eResult = finalizeResult(move.m_eResult);

  return rc;
}

int Chess::computeEnginesMove(ChessMove &move)
{
  static  string strFunc("Chess::getEnginesMove");

  int         nMoveNum;
  ChessColor  eMoveColor;
  string      strSAN;
  int         rc;

  // start fresh
  move.clear();

  // Move number. Get this value before making a move.
  nMoveNum = m_engine.getMoveNumInPlay();

  // Compute engine's move.
  rc = m_engine.computeEnginesMove(eMoveColor, strSAN);

  // Fill in critical move fields.
  move.m_nMoveNum = nMoveNum;
  move.m_ePlayer  = eMoveColor;
  move.m_strSAN   = strSAN;
  move.m_eResult  = Ok;

  if( rc != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc
          << ": getEnginesMove(color, SAN): "
          << strerror(rc)
          << ".");
  }

  // Parse SAN and fill in appropriate move fields.
  else if( (rc = m_parser.parse(strSAN, move)) != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc
          << ": parse(" << strSAN
          << ", move): "
          << m_parser.getErrorStr()
          << ".");
  }

  // Qualify move against current game state, setting additional fields.
  else if( (rc = m_game.qualifyMove(move)) != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc << ": qualifyMove(move): " << strerror(rc) << ".");
  }

  // Now execute the move on the mirrored game state.
  else if( (rc = m_game.execMove(move)) != CE_OK )
  {
    ROS_ERROR_STREAM(strFunc << ": execMove(move): " << strerror(rc) << ".");
  }

  if( rc != CE_OK )
  {
    move.m_eResult = rcToMoveResult(rc);
  }

  // Finalize move/game result.
  move.m_eResult = finalizeResult(move.m_eResult);

  return rc;
}

ChessResult Chess::finalizeResult(const ChessResult eCurResult)
{
  ChessResult eEoGResult;
  ChessColor  eEoGWinner;
  ChessResult eFinalResult;

  // get any engine's end of game declaration 
  m_engine.getEnginesEoGDecl(eEoGResult, eEoGWinner);

  eFinalResult = eCurResult;

  // End of game, maybe.
  switch( eEoGResult )
  {
    case Checkmate:
    case Draw:
    case Resign:
    case Disqualified:
      endCurrentGame(eEoGResult, eEoGWinner);
      if( eFinalResult == Ok )
      {
        eFinalResult = eEoGResult; // override
      }
      break;
    case GameFatal:
      endCurrentGame(eEoGResult, NoColor);
      eFinalResult = eEoGResult;  // override
      break;
    default:
      switch( eCurResult )
      {
        case GameFatal:
          endCurrentGame(eCurResult, NoColor);
          break;
        default:
          break;
      }
      break;
  }

  return eFinalResult;
}
