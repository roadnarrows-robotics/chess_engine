////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      cePlayer.cpp
//
/*! \file
 *
 * \brief The chess player class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2017  RoadNarrows LLC
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

#include <stdio.h>
#include <ctype.h>

#include <string>
#include <vector>
#include <map>

#include "boost/assign.hpp"

#include "rnr/color.h"

#include "rnr/appkit/LogStream.h"
#include "rnr/appkit/Time.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/cePlayer.h"

using namespace std;
using namespace rnr;
using namespace rnr::chronos;
using namespace chess_engine;


// -----------------------------------------------------------------------------
// Class PlayerInfo
// -----------------------------------------------------------------------------

PlayerInfo::PlayerInfo()
  : m_id(NoPlayerId), m_eType(PlayerTypeAnon)
{
}

PlayerInfo::PlayerInfo(const ChessPlayerId   id,
                       const string          &strName,
                       const ChessPlayerType eType)
  : m_id(id), m_strName(strName), m_eType(eType)
{
}

PlayerInfo::~PlayerInfo()
{
}

PlayerInfo &PlayerInfo::operator=(const PlayerInfo &rhs)
{
  m_id      = rhs.m_id;
  m_strName = rhs.m_strName;
  m_eType   = rhs.m_eType;

  return *this;
}

void PlayerInfo::clear()
{
  m_id    = NoPlayerId;
  m_strName.clear();
  m_eType = PlayerTypeAnon;
}

ostream &chess_engine::operator<<(ostream &os, const PlayerInfo &obj)
{
  os << "{ "
    << "id = " << obj.m_id << ", "
    << "name = \"" << obj.m_strName << "\", "
    << "type = " << nameOfPlayerType(obj.m_eType)
    << "(" << (char)obj.m_eType << ")"
    << " }";

  return os;
}


// -----------------------------------------------------------------------------
// Class PlayerSummary
// -----------------------------------------------------------------------------

PlayerSummary::PlayerSummary()
{
  clear();
}

PlayerSummary::~PlayerSummary()
{
}

PlayerSummary &PlayerSummary::operator=(const PlayerSummary &rhs)
{
  m_games   = rhs.m_games;
  m_wins    = rhs.m_wins;
  m_losses  = rhs.m_losses;
  m_draws   = rhs.m_draws;

  return *this;
}

void PlayerSummary::clear()
{
  m_games   = 0;
  m_wins    = 0;
  m_losses  = 0;
  m_draws   = 0;
}

ostream &chess_engine::operator<<(ostream &os, const PlayerSummary &obj)
{
  os << "{ "
    << "games = "  << obj.m_games  << ", "
    << "wins = "   << obj.m_wins   << ", "
    << "losses = " << obj.m_losses << ", "
    << "draws = "  << obj.m_draws
    << " }";

  return os;
}


// -----------------------------------------------------------------------------
// Class GameRecord
// -----------------------------------------------------------------------------

GameRecord::GameRecord()
{
  clear();
}

GameRecord::~GameRecord()
{
}

GameRecord &GameRecord::operator=(const GameRecord &rhs)
{
  m_infoPlayer    = rhs.m_infoPlayer;
  m_infoOpponent  = rhs.m_infoOpponent;
  m_eColorPlayed  = rhs.m_eColorPlayed;
  m_timeStart     = rhs.m_timeStart;
  m_timeEnd       = rhs.m_timeEnd;
  m_uNumMoves     = rhs.m_uNumMoves;
  m_eWinner       = rhs.m_eWinner;
  m_eResult       = rhs.m_eResult;

  return *this;
}

void GameRecord::clear()
{
  m_infoPlayer.clear();
  m_infoOpponent.clear();

  m_eColorPlayed = NoColor;

  m_timeStart.clear();
  m_timeEnd.clear();

  m_uNumMoves = 0;
  m_eWinner   = NoColor;
  m_eResult   = NoResult;
}

ostream &chess_engine::operator<<(ostream &os, const GameRecord &obj)
{
  os << "{" << endl;

  os << "  player   = " << obj.m_infoPlayer << endl;
  os << "  opponent = " << obj.m_infoOpponent << endl;
  os << "  played   = " << nameOfColor(obj.m_eColorPlayed)
    << "(" << (char)obj.m_eColorPlayed << ")" << endl;
  os << "  start    = " << obj.m_timeStart.calendarTime() << endl;
  os << "  end      = " << obj.m_timeEnd.calendarTime() << endl;
  os << "  numMoves = " << obj.m_uNumMoves << endl;
  os << "  winner   = " << nameOfColor(obj.m_eWinner)
    << "(" << (char)obj.m_eWinner << ")" << endl;
  os << "  result   = " << nameOfResult(obj.m_eResult)
    << "(" << (char)obj.m_eResult << ")" << endl;

  os << "}";

  return os;
}


// -----------------------------------------------------------------------------
// Class ChessPlayer
// -----------------------------------------------------------------------------

/*! "no player" player */
static ChessPlayer NoPlayer;

ChessPlayer::ChessPlayer()
  : m_eColor(NoColor)
{
}

ChessPlayer::ChessPlayer(const ChessPlayerId   id,
                         const std::string     strName,
                         const ChessPlayerType eType)
  : m_info(id, strName, eType)
{
}

ChessPlayer::ChessPlayer(const PlayerInfo &info)
  : m_info(info)
{
}

ChessPlayer::~ChessPlayer()
{
}

ChessPlayer &ChessPlayer::operator=(const ChessPlayer &rhs)
{
  newPersona(rhs.id(), rhs.name(), rhs.type());
  return *this;
}

void ChessPlayer::clear()
{
  m_info.clear();
  m_eColor = NoColor;
  m_summary.clear();
  m_history.clear();
}

void ChessPlayer::markStartOfGame(const ChessColor &eColor,
                                  const PlayerInfo &infoOpponent,
                                  const Time       &timeStart)
{
  if( isPlaying() )
  {
    LOGERROR_STREAM("Player " << m_info << " already playing a game.");
    return;
  }

  m_eColor = eColor;

  GameRecord  rec;

  rec.m_infoPlayer    = m_info;
  rec.m_infoOpponent  = infoOpponent;
  rec.m_eColorPlayed  = m_eColor;
  rec.m_timeStart     = timeStart;

  m_history.push_back(rec);

  m_summary.games(m_summary.games()+1);
}

void ChessPlayer::markEndOfGame(const ChessColor  eWinner,
                                const ChessResult eResult, 
                                const size_t      uNumMoves,
                                const Time        &timeEnd)
{
  if( !isPlaying() )
  {
    LOGERROR_STREAM("Player " << m_info << " is not playing a game.");
    return;
  }

  GameRecord &rec = m_history.back();

  rec.m_timeEnd   = timeEnd;
  rec.m_uNumMoves = uNumMoves;
  rec.m_eWinner   = eWinner;
  rec.m_eResult   = eResult;

  switch( eResult )
  {
    case BadMove:
    case OutOfTurn:
    case Busy:
    case InPlay:
    case NoResult:
    case NoGame:
    case Ok:
      rec.m_eResult = NoResult;
      LOGWARN_STREAM("Result "
          << nameOfResult(eResult) << "(" << (char)eResult << ") "
          << "is not an end-of-game result.");
      break;

    case Checkmate:
    case Resign:
    case Disqualified:
      if( eWinner == m_eColor )
      {
        m_summary.wins(m_summary.wins()+1);
      }
      else
      {
        m_summary.losses(m_summary.losses()+1);
      }
      break;

    case Draw:
      m_summary.draws(m_summary.draws()+1);
      break;

    case Aborted:
    case GameFatal:
      rec.m_eResult = Aborted;
      break;

    default:
      LOGWARN_STREAM("Result "
          << nameOfResult(eResult) << "(" << (char)eResult << ") "
          << "is unknown.");
      break;
  }

  m_eColor = NoColor;
}

void ChessPlayer::newPersona(const ChessPlayerId   id,
                             const std::string     strName,
                             const ChessPlayerType eType)
{
  clear();

  m_info.m_id       = id;
  m_info.m_strName  = strName;
  m_info.m_eType    = eType;
}

ChessPlayer &ChessPlayer::noplayer()
{
  NoPlayer.clear();   // not constant, so make sure

  return NoPlayer;
}

ostream &chess_engine::operator<<(ostream &os, const ChessPlayer &obj)
{
  os << "{" << endl;

  os << "  info    = " << obj.m_info << endl;
  os << "  playing = " << nameOfColor(obj.m_eColor)
    << "(" << (char)obj.m_eColor << ")" << endl;
  os << "  summary = " << obj.m_summary << endl;
  os << "  history[" << obj.m_history.size() << "] =" << endl;
  os << "  {" << endl;
  for(PlayerHistoryCIter iter = obj.m_history.begin();
      iter != obj.m_history.end();
      ++iter)
  {
    os << *iter << endl;
  }
  os << "  }" << endl;

  os << "}";

  return os;
}
