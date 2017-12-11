////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceGame.cpp
//
/*! \file
 *
 * \brief The chess game state implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2017  RoadNarrows
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
#include <sstream>

#include "rnr/appkit/LogStream.h"
#include "rnr/appkit/Time.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"
#include "chess_engine/ceUtils.h"

using namespace std;
using namespace chess_engine;

static ChessPlayer anonplayer;


// -----------------------------------------------------------------------------
// Chess game class.
// -----------------------------------------------------------------------------

ChessGame::ChessGame()
  : m_anonWhite(AnonWhitePlayerId, "White", PlayerTypeAnon),
    m_anonBlack(AnonBlackPlayerId, "Black", PlayerTypeAnon),
    m_playerWhite(m_anonWhite),
    m_playerBlack(m_anonBlack)
{
  m_bIsPlaying  = false;
  m_ePlayState  = NoGame;
  m_eWinner     = NoColor;

  m_eTurnToMove           = NoColor;
  m_playerName[White]     = "noplayer";
  m_playerName[Black]     = "noplayer";
  m_numPromotions[White]  = 0;
  m_numPromotions[Black]  = 0;

  m_bGraphic = false;
}

ChessGame::~ChessGame()
{
}

int ChessGame::startNewGame(ChessPlayer &white = anon(),
                            ChessPlayer &black = anon())
{
  m_playerWhite = white;
  m_playerBlack = black;

  Time  t;

  t.markNow();

  setupGame();

  m_bIsPlaying  = true;
  m_ePlayState  = InPlay;
  m_eWinner     = NoColor;

  m_eTurnToMove           = White;
  m_playerName[White]     = strWhite.empty()? "Witbier": strWhite;
  m_playerName[Black]     = strBlack.empty()? "Dunkel":  strBlack;
  m_numPromotions[White]  = 0;
  m_numPromotions[Black]  = 0;

  return CE_OK;
}

int ChessGame::endCurrentGame(ChessResult eReason, ChessColor eWinner)
{
  m_bIsPlaying  = false;
  m_ePlayState  = eReason;
  m_eWinner     = eWinner;

  return CE_OK;
}

int ChessGame::qualifyMove(ChessMove &move)
{
  int   rc;

  //
  // ChessGame state sanity checks.
  //
  if( !m_bIsPlaying )
  {
    move.m_eResult = NoGame;
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Cursory check of move's current result
  //
  switch( move.m_eResult )
  {
    case BadMove:
      return -CE_ECODE_CHESS_BAD_MOVE;
    case OutOfTurn:
      return -CE_ECODE_CHESS_OUT_OF_TURN;
    case NoGame:
      endCurrentGame(move.m_eResult);
      return -CE_ECODE_CHESS_SYNC;
    case GameFatal:
      endCurrentGame(move.m_eResult);
      return -CE_ECODE_CHESS_FATAL;
    case Checkmate:
    case Draw:
    case Resign:
    case Disqualified:
      endCurrentGame(move.m_eResult);
      // there is still a move to process
      break;
    case Ok:
    case NoResult:
    default:
      break;
  }

  // Check an out-of-turn condition.
  if( move.m_ePlayer != m_eTurnToMove )
  {
    LOGERROR_STREAM("Player "
        << nameOfColor(move.m_ePlayer)
        << "out of turn.");
    rc = -CE_ECODE_CHESS_SYNC;
  }

  // qualify source and destination of the move
  else if( (rc = qualifyMovePositions(move)) != CE_OK )
  {
    LOGDIAG2("%d = qualifyMovePositions() failed.", rc);
  }

  // verify piece moved
  else if( (rc = qualifyMovePiece(move)) != CE_OK )
  {
    LOGDIAG2("%d = qualifyMovePiece() failed.", rc);
  }

  // verify any capture
  else if( (rc = qualifyMoveCapture(move)) != CE_OK )
  {
    LOGDIAG2("%d = qualifyMoveCapture() failed.", rc);
  }

  // good
  else
  {
    rc = CE_OK;
  }

  if( rc != CE_OK )
  {
    move.m_eResult = GameFatal;
    endCurrentGame(move.m_eResult);
  }

  return rc;
}

int ChessGame::qualifyMovePositions(ChessMove &move)
{
  int   rc;

  // fully qualify source position
  if( move.m_eCastling == NoCastling )
  {
    m_board.findMoveSrcPos(move);
  }

  // fully qualify castling source and destination positions
  else
  {
    ChessMove::getCastlingKingMove(move.m_ePlayer, move.m_eCastling,
                                   move.m_posSrc, move.m_posDst);
  }

  //
  // Verify move destination position.
  //
  if( !ChessBoard::isOnChessBoard(move.m_posDst) )
  {
    LOGERROR_STREAM("Bad destination square "
                    << move.m_posDst
                    << ".");
    rc = -CE_ECODE_CHESS_FATAL;
  }
  
  //
  // Verify move source position.
  //
  else if( !ChessBoard::isOnChessBoard(move.m_posSrc) )
  {
    LOGERROR_STREAM("Bad source square "
                    << move.m_posSrc
                    << ".");
    rc = -CE_ECODE_CHESS_FATAL;
  }

  // good
  else
  {
    rc = CE_OK;
  }

  return rc;
}

int ChessGame::qualifyMovePiece(ChessMove &move)
{
  ChessPiece  ePieceSrc;
  int         rc;

  // chess piece at source position determined by board state
  ePieceSrc = m_board.at(move.m_posSrc).getPieceType();

  // set moved piece, if unset
  if( move.m_ePieceMoved == NoPiece )
  {
    move.m_ePieceMoved = ePieceSrc;
  }

  //
  // Cannot move a non-existent piece.
  //
  if( move.m_ePieceMoved == NoPiece )
  {
    LOGERROR_STREAM("No piece found on source square "
                    << move.m_posSrc
                    << ".");
    rc = -CE_ECODE_CHESS_SYNC;
  }

  //
  // Disagreement about what piece to move - game probably out of sync.
  //
  else if( move.m_ePieceMoved != ePieceSrc )
  {
    LOGERROR_STREAM("Piece "
                  << nameOfPiece(move.m_ePieceMoved)
                  << " != "
                  << nameOfPiece(ePieceSrc)
                  << " found on source square "
                  << move.m_posSrc
                  << ".");
    rc = -CE_ECODE_CHESS_SYNC;
  }

  // good
  else
  {
    rc = CE_OK;
  }

  return rc;
}

int ChessGame::qualifyMoveCapture(ChessMove &move)
{
  ChessPos    posCapture;
  ChessPiece  ePieceCaptured;
  int         rc;

  //
  // Not a capture move.
  //
  if( move.m_ePieceCaptured == NoPiece )
  {
    // but destination square is not empty
    if( m_board.at(move.m_posDst).getPieceType() != NoPiece )
    {
      LOGERROR_STREAM(
        "Not a capture move, but piece found on destination square "
        << move.m_posDst << ".");
      return -CE_ECODE_CHESS_SYNC;
    }
    else
    {
      return CE_OK;
    }
  }

  // find captured piece position from en passant move
  if( move.m_bIsEnPassant )
  {
    ChessMove::getEnPassantCapturedPawnPos(move.m_ePlayer,
                                           move.m_posDst,
                                           posCapture);
  }
  // all other captures are at the destination square
  else
  {
    posCapture = move.m_posDst;
  }

  //
  // Verify capture position.
  //
  if( !ChessBoard::isOnChessBoard(posCapture) )
  {
    LOGERROR_STREAM("Bad capture square " << posCapture << ".");
    return -CE_ECODE_CHESS_FATAL;
  }
  
  // captured chess piece as determined by board state
  ePieceCaptured = m_board.at(posCapture).getPieceType();

  // set captured piece, iff undefined
  if( move.m_ePieceCaptured == UndefPiece )
  {
    move.m_ePieceCaptured = ePieceCaptured;
  }

  //
  // Cannot capture a non-existent piece.
  //
  if( move.m_ePieceCaptured == NoPiece )
  {
    LOGERROR_STREAM("No piece found on capture square " << posCapture << ".");
    rc = -CE_ECODE_CHESS_SYNC;
  }

  //
  // Disagreement about what piece is captured - game probably out of sync.
  //
  else if( move.m_ePieceCaptured != ePieceCaptured )
  {
    LOGERROR_STREAM("Piece "
                  << nameOfPiece(move.m_ePieceCaptured)
                  << " != "
                  << nameOfPiece(ePieceCaptured)
                  << " found on capture square "
                  << posCapture
                  << ".");
    rc = -CE_ECODE_CHESS_SYNC;
  }

  // good
  else
  {
    rc = CE_OK;
  }

  return rc;
}

int ChessGame::execMove(ChessMove &move)
{
  //
  // En passant move.
  //
  if( move.m_bIsEnPassant )
  {
    ChessPos  posCapture;

    // determine captured pawn's position.
    ChessMove::getEnPassantCapturedPawnPos(move.m_ePlayer,
                                           move.m_posDst,
                                           posCapture);

    // remove captured pawn and drop it into the player's bone yard.
    dropInBoneYard(move.m_ePlayer, posCapture);
  }

  //
  // Basic capture.
  //
  else if( move.m_ePieceCaptured != NoPiece )
  {
    // remove captured piece and drop it into the player's bone yard.
    dropInBoneYard(move.m_ePlayer, move.m_posDst);
  }

  //
  // Move the piece.
  //
  m_board.movePiece(move.m_posSrc, move.m_posDst);

  //
  // Castling move.
  // Note:  By the rules of chess, the king (above) must move first,
  //        then the rook.
  //
  if( move.m_eCastling != NoCastling )
  {
    ChessPos posRookSrc, posRookDst;

    // get castling rook's source and destination positions.
    ChessMove::getCastlingRookMove(move.m_ePlayer, move.m_eCastling,
                                   posRookSrc, posRookDst);

    // move the rook
    m_board.movePiece(posRookSrc, posRookDst);
  }

  //
  // Pawn promotion move.
  //
  if( move.m_ePiecePromoted != NoPiece )
  {
    int         nNumPromotions;
    std::string strPieceId;

    // remove the pawn from the board and drop in the player's bone yard.
    dropInBoneYard(move.m_ePlayer, move.m_posDst);

    // incremented players number of promotions
    m_numPromotions[move.m_ePlayer] = m_numPromotions[move.m_ePlayer] + 1;

    nNumPromotions = m_numPromotions[move.m_ePlayer];

    // make the new promoted piece's unique id
    strPieceId = ChessFqPiece::makePieceId(move.m_posDst,
                                           move.m_ePlayer,
                                           move.m_ePiecePromoted,
                                           nNumPromotions);

    // place promoted piece on the board
    m_board.setPiece(move.m_posDst, move.m_ePlayer, move.m_ePiecePromoted,
                      strPieceId);
  }

  // record move
  recordHistory(move);

  // next turn to move
  m_eTurnToMove = opponent(m_eTurnToMove);

  return CE_OK;
}

void ChessGame::setupGame()
{
  m_history.clear();
  m_boneyardWhite.clear();
  m_boneyardBlack.clear();
  m_board.setupBoard();
}

void ChessGame::getPlayerNames(std::string &strWhite,
                               std::string &strBlack) const
{
  strWhite = m_playerName.find(White)->second;
  strBlack = m_playerName.find(Black)->second;
}

int ChessGame::getNumOfPliesPlayed() const
{
  return (int)m_history.size();
}

int ChessGame::getNumOfMovesPlayed() const
{
  return getNumOfPliesPlayed() / 2;
}

int ChessGame::getMoveNumInPlay() const
{
  if( isPlayingAGame() )
  {
    return getNumOfMovesPlayed() + 1;
  }
  else
  {
    return 0;
  }
}

ChessSquare &ChessGame::getBoardSquare(const int file, const int rank)
{
  return m_board.at(file, rank);
}

ChessSquare &ChessGame::getBoardSquare(const ChessPos &pos)
{
  return m_board.at(pos);
}

const ChessMove &ChessGame::getHistoryAt(int nPlyNum) const
{
  static const ChessMove  NoMove;

  if( nPlyNum < m_history.size() )
  {
    return m_history[nPlyNum];
  }
  else
  {
    return NoMove;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Protected
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessGame::recordHistory(ChessMove &move)
{
  m_history.push_back(move);
}

void ChessGame::dropInBoneYard(const ChessColor ePlayer, const ChessPos &pos)
{
  switch( ePlayer )
  {
    case White:
      m_boneyardWhite.push_back(m_board.at(pos).getPiece());
      break;
    case Black:
      m_boneyardBlack.push_back(m_board.at(pos).getPiece());
      break;
    default:
      break;
  }

  m_board.removePiece(pos);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &chess_engine::operator<<(ostream &os, const ChessGame &game)
{
  os << "White: " << game.m_playerName.find(White)->second << endl;
  os << "Black: " << game.m_playerName.find(Black)->second << endl;
  os << game.getNumOfMovesPlayed() << ".  " << nameOfColor(game.m_eTurnToMove);
  os << endl;
  os << game.m_board;

  return os;
}
