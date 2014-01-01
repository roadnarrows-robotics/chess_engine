////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_game.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The chess game state implementation.
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

#include <stdio.h>
#include <ctype.h>

#include <string>
#include <vector>
#include <map>

#include "rnr/color.h"

#include "chess.h"

#include "chess_move.h"
#include "chess_game.h"
#include "chess_server.h"

using namespace std;
using namespace chess_engine;


ChessGame::ChessGame()
{
  setupBoard();

  m_bGui = false;
}

void ChessGame::setupBoard()
{
  int   row, col;

  //
  // Erase board
  //
  for(col=0; col<NumOfFiles; ++col)
  {
    for(row=0; row<NumOfRanks; ++row)
    {
      m_board[row][col] = EmptyElem;
    }
  }

  // 
  // White
  //
 
  // rank 1
  row = toRow(ChessRank1);

  col = toCol(ChessFileA);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = Rook;

  col = toCol(ChessFileB);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = Knight;

  col = toCol(ChessFileC);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = Bishop;

  col = toCol(ChessFileD);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = Queen;

  col = toCol(ChessFileE);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = King;

  col = toCol(ChessFileF);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = Bishop;
  
  col = toCol(ChessFileG);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = Knight;

  col = toCol(ChessFileH);
  m_board[row][col].m_color = White;
  m_board[row][col].m_piece = Rook;
  
  // rank 2
  row = toRow(ChessRank2);

  for(col=0; col<NumOfFiles; ++col)
  {
    m_board[row][col].m_color = White;
    m_board[row][col].m_piece = Pawn;
  }

  // 
  // Black
  //
  
  // rank 7
  row = toRow(ChessRank7);

  for(col=0; col<NumOfFiles; ++col)
  {
    m_board[row][col].m_color = Black;
    m_board[row][col].m_piece = Pawn;
  }
 
  // rank 8
  row = toRow(ChessRank8);

  col = toCol(ChessFileA);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = Rook;

  col = toCol(ChessFileB);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = Knight;

  col = toCol(ChessFileC);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = Bishop;

  col = toCol(ChessFileD);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = Queen;

  col = toCol(ChessFileE);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = King;

  col = toCol(ChessFileF);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = Bishop;
  
  col = toCol(ChessFileG);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = Knight;

  col = toCol(ChessFileH);
  m_board[row][col].m_color = Black;
  m_board[row][col].m_piece = Rook;

  m_bIsPlaying  = true;
  m_endReason   = NoResult;
  m_winner      = NoColor;
}

int ChessGame::sync(ChessMove &move)
{
  ChessBoardElem *pSrc, *pDst, *p3rd;

  pSrc = NULL;
  pDst = NULL;
  p3rd = NULL;

  // sanity check
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
      m_bIsPlaying  = false;
      m_endReason   = move.m_result;
      return -CE_ECODE_CHESS_SYNC;
    case GameFatal:
      m_bIsPlaying  = false;
      m_endReason   = move.m_result;
      return -CE_ECODE_CHESS_FATAL;
    case Checkmate:
    case Draw:
    case Resign:
      m_bIsPlaying  = false;
      m_endReason   = move.m_result;
      m_winner      = move.m_winner;
      // there is still a move to process
      break;
    case Ok:
    case NoResult:
    default:
      break;
  }

  //
  // Move source
  //
  if( (pSrc = elem(move.m_sqFrom)) == NULL )
  {
    printf("ROSLOG: Error: bad 'from' square = %c%c.\n",
        move.m_sqFrom.m_file, move.m_sqFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_FATAL;
  }
  
  //
  // Move destination
  //
  if( (pDst = elem(move.m_sqTo)) == NULL )
  {
    printf("ROSLOG: Error: bad 'to' square = %c%c.\n",
        move.m_sqTo.m_file, move.m_sqTo.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_FATAL;
  }
  
  //
  // Moved piece
  //
  if( move.m_piece == NoPiece )
  {
    move.m_piece = pSrc->m_piece;
  }

  // cannot move a piece that ain't there
  if( move.m_piece == NoPiece )
  {
    printf("ROSLOG: Error: no piece found on 'from' square %c%c.\n",
        move.m_sqFrom.m_file, move.m_sqFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_SYNC;
  }

  // disagreement about what piece to move
  else if( move.m_piece != pSrc->m_piece )
  {
    printf("ROSLOG: Error: piece %c != %c found on 'from' square %c%c.\n",
        move.m_piece, pSrc->m_piece,
        move.m_sqFrom.m_file, move.m_sqFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Pawn Promotion
  //
  if( move.m_promotion != NoPiece )
  {
    // promote here, move is at the end of this function
    pSrc->m_piece = move.m_promotion;
  }

  //
  // Capturing
  //
  if( pDst->m_piece != NoPiece )
  {
    move.m_captured = pDst->m_piece;
    moveToBoneYard(pDst);
  }

  //
  // En Passant (destination must be empty)
  //
  else if( move.m_piece == Pawn )
  {
    // white
    if( move.m_color == White )
    {
      if( (move.m_sqFrom.m_rank == ChessRank5) &&
          (move.m_sqTo.m_rank   == ChessRank6) &&
          (move.m_sqFrom.m_file != move.m_sqTo.m_file) &&
          (pDst->m_piece == NoPiece) )
      {
        move.m_sqAccAt.m_file = move.m_sqTo.m_file;
        move.m_sqAccAt.m_rank = move.m_sqFrom.m_rank;
        p3rd = elem(move.m_sqAccAt);
        if( p3rd->m_piece == Pawn )
        {
          move.m_captured = p3rd->m_piece;
          moveToBoneYard(p3rd);
        }
        else
        {
          move.m_sqAccAt = NoMove;
        }
      }
    }

    // black
    else
    {
      if( (move.m_sqFrom.m_rank == ChessRank4) &&
          (move.m_sqTo.m_rank   == ChessRank3) &&
          (move.m_sqFrom.m_file != move.m_sqTo.m_file) &&
          (pDst->m_piece == NoPiece) )
      {
        move.m_sqAccAt.m_file = move.m_sqTo.m_file;
        move.m_sqAccAt.m_rank = move.m_sqFrom.m_rank;
        p3rd = elem(move.m_sqAccAt);
        if( p3rd->m_piece == Pawn )
        {
          move.m_captured = p3rd->m_piece;
          moveToBoneYard(p3rd);
        }
        else
        {
          move.m_sqAccAt = NoMove;
        }
      }
    }
  }

  //
  // Castling
  //
  else if( move.m_castle == KingSide )
  {
    move.m_sqAccAt.m_file = move.m_sqTo.m_file + 1;
    move.m_sqAccAt.m_rank = move.m_sqTo.m_rank;
    move.m_sqAccTo.m_file = move.m_sqTo.m_file - 1;
    move.m_sqAccTo.m_rank = move.m_sqTo.m_rank;
    movePiece(move.m_sqAccAt, move.m_sqAccTo);
  }
  else if( move.m_castle == QueenSide )
  {
    move.m_sqAccAt.m_file = move.m_sqTo.m_file - 2;
    move.m_sqAccAt.m_rank = move.m_sqTo.m_rank;
    move.m_sqAccTo.m_file = move.m_sqTo.m_file + 1;
    move.m_sqAccTo.m_rank = move.m_sqTo.m_rank;
    movePiece(move.m_sqAccAt, move.m_sqAccTo);
  }

  //
  // Finally, move the piece
  //
  movePiece(pSrc, pDst);

  recordHistory(move);

  return CE_OK;
}

void ChessGame::recordHistory(ChessMove &move)
{
  m_history.push_back(move);
}

void ChessGame::moveToBoneYard(ChessBoardElem *pDeadPiece)
{
  if( pDeadPiece->m_piece != NoPiece )
  {
    if( pDeadPiece->m_color == White )
    {
      m_boneYardWhite.push_back(pDeadPiece->m_piece);
    }
    else if( pDeadPiece->m_color == Black )
    {
      m_boneYardBlack.push_back(pDeadPiece->m_piece);
    }
  }

  *pDeadPiece = EmptyElem;
}

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
