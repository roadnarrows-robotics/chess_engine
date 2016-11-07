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
 * (C) 2013-2016  RoadNarrows
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
#include <stringstream>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"
#include "chess_engine/ceUtils.h"

using namespace std;
using namespace chess_engine;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Chess game class.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ChessGame::ChessGame()
{
  m_bIsPlaying  = false;
  m_eEoGReason  = NoResult;
  m_eWinner     = NoColor;

  m_eTurnToMove  = NoColor;

  m_bGui = false;
}

ChessGame::~ChessGame()
{
}

int ChessGame::startNewGame(const string &strWhite, const string &strBlack)
{
  setupGame();

  m_bIsPlaying  = true;
  m_eEoGReason  = NoResult;
  m_eWinner     = NoColor;

  m_eTurnToMove           = White;
  m_playerName[White]     = strWhite;
  m_playerName[Black]     = strBlack;
  m_numPromotions[White]  = 0;
  m_numPromotions[Black]  = 0;

}

int ChessGame::endCurrentGame(ChessResult eReason, ChessColor eWinner)
{
  m_bIsPlaying  = false;
  m_eEoGReason  = eReason;
  m_eWinner     = eWinner;
}

int ChessGame::qualifyMove(ChessMove &move)
{
  ChessPiece  ePieceSrc;

  //
  // ChessGame state sanity checks.
  //
  if( !m_bIsPlaying )
  {
    move.m_result = NoGame;
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Cursory check of move's current result
  //
  switch( move.m_result )
  {
    case BadMove:
      return -CE_ECODE_CHESS_MOVE;
    case OutOfTurn:
      return -CE_ECODE_CHESS_OUT_OF_TURN;
    case NoGame:
      endCurrentGame(move.m_result);
      return -CE_ECODE_CHESS_SYNC;
    case GameFatal:
      endCurrentGame(move.m_result);
      return -CE_ECODE_CHESS_FATAL;
    case Checkmate:
    case Draw:
    case Resign:
    case Disqualified:
      endCurrentGame(move.m_result);
      // there is still a move to process
      break;
    case Ok:
    case NoResult:
    default:
      break;
  }

  if( move.m_ePlayer != m_eTurnToMove )
  {
    LOGERROR();
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Castling special move
  //
  if( move.m_eCastling != NoCastling )
  {
    ChessMove::getCastlingKingMove(move.m_ePlayer, move.m_eCastling,
                                   move.m_posSrc, move.m_posDst);
  }

  //
  // Set source of all other moves. Destination must be fully specified.
  //
  else
  {
    m_board.setMoveSrcPos(move);
  }

  //
  // Source position is not fully specified or is not on the board.
  //
  if( !ChessBoard::isOnChessBoard(move.m_posSrc) )
  {
    printf("ROSLOG: Error: bad 'from' square = %c%c.\n",
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    endCurrentGame(GameFatal);
    return -CE_ECODE_CHESS_FATAL;
  }
  
  //
  // Destination position is not fully specified or is not on the board.
  //
  else if( !ChessBoard::isOnChessBoard(move.m_posDst) )
  {
    printf("ROSLOG: Error: bad 'to' square = %c%c.\n",
        move.m_posTo.m_file, move.m_posTo.m_rank);
    endCurrentGame(GameFatal);
    return -CE_ECODE_CHESS_FATAL;
  }
  
  // chess piece at source position
  ePieceSrc = m_board.at(move.m_posSrc).getPieceType();

  //
  // Set moved piece, if unset
  //
  if( move.m_ePiecedMoved == NoPiece )
  {
    move.m_ePiecedMoved = ePieceSrc;
  }

  //
  // Cannot move a non-existent piece.
  //
  if( move.m_ePiecedMoved == NoPiece )
  {
    printf("ROSLOG: Error: no piece found on 'from' square %c%c.\n",
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    endCurrentGame(GameFatal);
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Disagreement about what piece to move - game probably out of sync.
  //
  else if( move.m_ePiecedMoved = ePieceSrc )
  {
    printf("ROSLOG: Error: piece %c != %c found on 'from' square %c%c.\n",
        move.m_ePiecedMoved, sqSrc.getPieceType(),
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    endCurrentGame(GameFatal);
    return -CE_ECODE_CHESS_SYNC;
  }

  return CE_OK;
}

int ChessGame::execMove(ChessMove &move)
{
  //
  // En passant move.
  //
  if( move.m_en_passant )
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
  else if( move.m_capture )
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
    std::string strPieceId;

    // remove the pawn from the board and drop in the player's bone yard.
    dropInBoneYard(move.m_ePlayer, move.m_posDst);

    // make the new promoted piece's unique id
    str = ChessFqPiece::makePieceId(move.m_ePlayer,
                                    move.m_ePiecePromoted,
                                    move.m_posDst,
                                    m_numPromotions[move.m_ePlayer]);

    // place promoted piece on the board
    m_board.setPiece(move.m_posDst, move.m_ePlayer, move.m_ePiecePromoted,
                      strPieceId);

    // incremented players number of promotions
    m_numPromotions[move.m_ePlayer] = m_numPromotions[move.m_ePlayer] + 1;
  }

  // record move
  recordHistory(move);

  // RDK alternate

  return CE_OK;
}

void ChessGame::setupGame()
{
  m_history.clear();
  m_boneyardWhite.clear();
  m_boneyardBlack.clear();
  m_board.setupBoard();
}

int ChessGame::getNumOfMoves()
{
  return ((int)m_history.size() + 1) / 2;
}

int ChessGame::getNumOfPlies()
{
  return (int)m_history.size();
}

ChessSquare &ChessGame::getBoardSquare(const int file, const int rank)
{
  ChessPos  pos(file, rank);

  return getBoardSquare(pos);
}

ChessSquare &ChessGame::getBoardSquare(const ChessPos &pos)
{
  if( ChessBoard::toRowCol(pos, row, col) == CE_OK )
  {
    return m_board[row][col];
  }
  else
  {
    return ChessNoSquare;
  }
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
      m_boneyardWhite.push_back(m_board.at(pos).getFqPiece());
      break;
    case Black:
      m_boneyardBlack.push_back(m_board.at(pos).getFqPiece());
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
  int         file, rank;       // chess board file,rank
  int         row, col;         // board matrix row,column
  ChessPiece  piece;            // chess piece
  string      strPiece;         // piece one-character name
  ChessColor  colorPiece;       // color of piece (white or black)
  ChessColor  colorSquare;      // color of square (white or black)
  streamsize  w = os.width();   // save default width
  string      strWW;            // white piece on white square
  string      strBW;            // black piece on white square
  string      strWB;            // white piece on black square
  string      strBB;            // black piece on black square
  string      strReset;         // reset colors

  if( game.m_bGui )
  {
    // RDK: fix color.h
    strWW = ANSI_COLOR_PRE "0;30;47m";
    strBW = ANSI_COLOR_PRE "0;30;47m";
    strWB = ANSI_COLOR_PRE "0;30;46m";
    strBB = ANSI_COLOR_PRE "0;30;46m";

    strReset = ANSI_COLOR_RESET;
  }

  for(rank = ChessRank8; rank >= ChessRank1; --rank)
  {
    if( !game.m_bGui )
    {
      os.width(NumOfFiles * 4 + 1);
      os.fill('.');
      os << '.';
      os.width(w);
      os << endl;
    }

    for(file = ChessFileA; file <= ChessFileH; ++file)
    {
      row = game.toRow(rank);
      col = game.toCol(file);

      colorSquare = game.getSquareColor(file, rank);
      piece       = game.m_board[row][col].m_piece;
      colorPiece  = game.m_board[row][col].m_color;

      if( game.m_bGui )
      {
        if( piece == NoPiece )
        {
          //strPiece = "\U00002588";
          strPiece = " ";
          colorPiece  = colorSquare;
        }
        else
        {
          strPiece = figurineOfPiece(colorPiece, piece);
        }
      }
      else
      {
        if( piece == NoPiece )
        {
          strPiece    = " ";
          colorPiece  = colorSquare;
        }
        else if( colorPiece == White )
        {
          strPiece = (char)piece;
        }
        else
        {
          strPiece = (char)tolower(piece);
        }
      }

      if( game.m_bGui )
      {
        if( colorSquare == White )
        {
          if( colorPiece == White )
          {
            os << strWW;
          }
          else
          {
            os << strBW;
          }
        }
        else
        {
          if( colorPiece == White )
          {
            os << strWB;
          }
          else
          {
            os << strBB;
          }
        }

        os << strPiece << " ";
        os << strReset;
      }
      else
      {
        os << "|";
        os << " " << strPiece << " ";
      }
    }

    if( !game.m_bGui )
    {
      os << "|";
    }

    os << endl;
  }

  if( !game.m_bGui )
  {
    os.width(NumOfFiles * 4 + 1);
    os.fill('-');
    os << '-' << endl;
    os.width(w);
  }

  os << endl;

  return os;
}
