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
 * (C) 2016-2017  RoadNarrows
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

#include "rnr/appkit/LogStream.h"

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

int Chess::startNewGame(ChessPlayer *pWhite, ChessPlayer *pBlack)
{
  int   rc;

  if( (rc = m_engine.startNewGame()) == CE_OK )
  {
    rc = m_game.startNewGame(pWhite, pBlack);
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

  int     rc;

  // start fresh
  move.clear();

  // fill in move attempt fields
  move.m_nMoveNum = m_engine.getMoveNumInPlay();
  move.m_ePlayer  = ePlayer;
  move.m_strAN    = strAN;
  move.m_eResult  = Ok;

  //
  // Pre-screen move.
  //

  //
  // Make the player's move.
  //
  if( (rc = m_engine.makePlayersMove(ePlayer, move.m_strAN)) != CE_OK )
  {
    LOGERROR_STREAM(strFunc
          << ": makePlayersMove(" << nameOfColor(ePlayer) << ", "
          << strAN << "): "
          << strecode(rc)
          << ".");
  }

  //
  // Parse AN and fill in appropriate move fields.
  //
  else if( (rc = m_parser.parse(move.m_strAN, move)) != CE_OK )
  {
    LOGERROR_STREAM(strFunc
          << ": parse(" << strAN
          << ", move): "
          << m_parser.getErrorStr()
          << ".");
  }

  //
  // Qualify move against current game state, setting additional fields.
  //
  else if( (rc = m_game.qualifyMove(move)) != CE_OK )
  {
    LOGERROR_STREAM(strFunc << ": qualifyMove(move): " << strecode(rc) << ".");
  }

  //
  // Now execute the move on the mirrored game state.
  //
  else if( (rc = m_game.execMove(move)) != CE_OK )
  {
    LOGERROR_STREAM(strFunc << ": execMove(move): " << strecode(rc) << ".");
  }

  //
  // On error, convert return code to move result.
  //
  if( rc != CE_OK )
  {
    move.m_eResult = rcToMoveResult(rc);
  }

  //
  // Finalize move/game result.
  //
  move.m_eResult = finalizeResult(move.m_eResult);

  return rc;
}

int Chess::computeEnginesMove(ChessMove &move)
{
  static  string strFunc("Chess::getEnginesMove");

  string      strAN;
  int         rc;

  // start fresh
  move.clear();

  // fill in move attempt fields
  move.m_nMoveNum = m_engine.getMoveNumInPlay();
  move.m_ePlayer  = m_engine.whoseTurn();
  move.m_eResult  = Ok;

  //
  // Compute engine's move.
  //
  if( (rc = m_engine.computeEnginesMove(strAN)) == CE_OK )
  {
    move.m_strAN = strAN;
  }
  else
  {
    LOGERROR_STREAM(strFunc
          << ": getEnginesMove(): "
          << strecode(rc)
          << ".");
  }

  if( rc == CE_OK )
  {
    //
    // Parse AN and fill in appropriate move fields.
    //
    if( (rc = m_parser.parse(strAN, move)) != CE_OK )
    {
      LOGERROR_STREAM(strFunc
          << ": parse(" << strAN
          << ", move): "
          << m_parser.getErrorStr()
          << ".");
    }

    //
    // Qualify move against current game state, setting additional fields.
    //
    else if( (rc = m_game.qualifyMove(move)) != CE_OK )
    {
      LOGERROR_STREAM(strFunc << ": qualifyMove(move): "
          << strecode(rc) << ".");
    }

    //
    // Now execute the move on the mirrored game state.
    //
    else if( (rc = m_game.execMove(move)) != CE_OK )
    {
      LOGERROR_STREAM(strFunc << ": execMove(move): " << strecode(rc) << ".");
    }
  }

  //
  // On error, convert return code to move result.
  //
  if( rc != CE_OK )
  {
    move.m_eResult = rcToMoveResult(rc);
  }

  //
  // Finalize move/game result.
  //
  move.m_eResult = finalizeResult(move.m_eResult);

  return rc;
}

ChessResult Chess::finalizeResult(const ChessResult eCurResult)
{
  ChessResult eEoGResult;   // end-of-game result, if any
  ChessColor  eEoGWinner;   // end-of-game winner, if any
  ChessResult eFinalResult; // final result

  // get any engine's end of game declaration 
  m_engine.getEnginesEoGDecl(eEoGResult, eEoGWinner);

  //
  // Finalize result, given the current result and any end-of-game result.
  //
  switch( eEoGResult )
  {
    case Checkmate:
    case Draw:
    case Resign:
    case Disqualified:
      endCurrentGame(eEoGResult, eEoGWinner);
      if( eCurResult == Ok )
      {
        eFinalResult = eEoGResult; // overrides current
      }
      break;
    case GameFatal:
      endCurrentGame(eEoGResult, NoColor);
      eFinalResult = eEoGResult;  // overrides current
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
      eFinalResult = eCurResult;  // keep current
      break;
  }

  return eFinalResult;
}

ChessCheckMod Chess::isInCheck() const
{
  const ChessGame::ChessHistory &history = m_game.getGameHistory();

  if( history.size() == 0 )
  {
    return NoCheckMod;
  }
  else
  {
    return history.back().m_eCheck;
  }
}

ChessResult Chess::getLastMoveResult() const
{
  const ChessGame::ChessHistory &history = m_game.getGameHistory();

  if( history.size() == 0 )
  {
    return NoResult;
  }
  else
  {
    return history.back().m_eResult;
  }
}
