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

#include "rnr/color.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

using namespace std;
using namespace chess_engine;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Chess board square class.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ChessSquare::ChessSquare()
{
  m_eColor = NoColor;
  m_eType  = NoPiece;
}

ChessSquare ChessSquare::operator==(const ChessSquare &rhs)
{
  m_eColor      = rhs.m_eColor;
  m_eType       = rhs.m_eType;
  m_strPieceId  = rhs.m_strPiecedId;

  return *this;
}

void ChessSquare::set(const ChessColor eColor,
                      const ChessPiece eType,
                      const std::string &strPieceId)
{
  m_eColor      = eColor;
  m_eType       = eType;
  m_strPieceId  = strPiecedId;
}

void ChessSquare::get(ChessColor &eColor,
                      ChessPiece &eType,
                      std::string &strPieceId)
{
  eColor      = m_eColor;
  eType       = m_eType;
  strPieceId  = m_strPiecedId;
}

void ChessSquare::copy(ChessSquare &dst)
{
  dst.m_eColor      = m_eColor;
  dst.m_eType       = m_eType;
  dst.m_strPieceId  = m_strPiecedId;
}

void ChessSquare::move(ChessSquare &dst)
{
  copy(dst);
  remove();
}

void ChessSquare::remove()
{
  m_eColor = NoColor;
  m_eType  = NoPiece;
  m_strPieceId.clear();
}

bool ChessSquare::isEmpty()
{
  return m_eType == NoPiece;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Chess game class.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

Game::Game()
{
  setupBoard();

  m_bGui = false;
}

void Game::setupBoard()
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

int Game::sync(Move &move)
{
  BoardElem *pSrc, *pDst, *p3rd;

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
  if( (pSrc = elem(move.m_posFrom)) == NULL )
  {
    printf("ROSLOG: Error: bad 'from' square = %c%c.\n",
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_FATAL;
  }
  
  //
  // Move destination
  //
  if( (pDst = elem(move.m_posTo)) == NULL )
  {
    printf("ROSLOG: Error: bad 'to' square = %c%c.\n",
        move.m_posTo.m_file, move.m_posTo.m_rank);
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
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_SYNC;
  }

  // disagreement about what piece to move
  else if( move.m_piece != pSrc->m_piece )
  {
    printf("ROSLOG: Error: piece %c != %c found on 'from' square %c%c.\n",
        move.m_piece, pSrc->m_piece,
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
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
    if( move.m_player == White )
    {
      if( (move.m_posFrom.m_rank == ChessRank5) &&
          (move.m_posTo.m_rank   == ChessRank6) &&
          (move.m_posFrom.m_file != move.m_posTo.m_file) &&
          (pDst->m_piece == NoPiece) )
      {
        move.m_posAuxAt.m_file = move.m_posTo.m_file;
        move.m_posAuxAt.m_rank = move.m_posFrom.m_rank;
        p3rd = elem(move.m_posAuxAt);
        if( p3rd->m_piece == Pawn )
        {
          move.m_captured = p3rd->m_piece;
          moveToBoneYard(p3rd);
        }
        else
        {
          move.m_posAuxAt = NoPos;
        }
      }
    }

    // black
    else
    {
      if( (move.m_posFrom.m_rank == ChessRank4) &&
          (move.m_posTo.m_rank   == ChessRank3) &&
          (move.m_posFrom.m_file != move.m_posTo.m_file) &&
          (pDst->m_piece == NoPiece) )
      {
        move.m_posAuxAt.m_file = move.m_posTo.m_file;
        move.m_posAuxAt.m_rank = move.m_posFrom.m_rank;
        p3rd = elem(move.m_posAuxAt);
        if( p3rd->m_piece == Pawn )
        {
          move.m_captured = p3rd->m_piece;
          moveToBoneYard(p3rd);
        }
        else
        {
          move.m_posAuxAt = NoPos;
        }
      }
    }
  }

  //
  // Castling
  //
  else if( move.m_castle == KingSide )
  {
    move.m_posAuxAt.m_file = move.m_posTo.m_file + 1;
    move.m_posAuxAt.m_rank = move.m_posTo.m_rank;
    move.m_posAuxTo.m_file = move.m_posTo.m_file - 1;
    move.m_posAuxTo.m_rank = move.m_posTo.m_rank;
    movePiece(move.m_posAuxAt, move.m_posAuxTo);
  }
  else if( move.m_castle == QueenSide )
  {
    move.m_posAuxAt.m_file = move.m_posTo.m_file - 2;
    move.m_posAuxAt.m_rank = move.m_posTo.m_rank;
    move.m_posAuxTo.m_file = move.m_posTo.m_file + 1;
    move.m_posAuxTo.m_rank = move.m_posTo.m_rank;
    movePiece(move.m_posAuxAt, move.m_posAuxTo);
  }

  //
  // Finally, move the piece
  //
  movePiece(pSrc, pDst);

  recordHistory(move);

  return CE_OK;
}

void Game::stopPlaying(ChessResult reason, ChessColor winner)
{
  m_bIsPlaying  = false;
  m_endReason   = reason;
  m_winner      = winner;
}

int Game::getNumOfMoves()
{
  return ((int)m_history.size() + 1) / 2;
}

int Game::getNumOfPlies()
{
  return (int)m_history.size();
}

BoardElem *Game::getBoardElem(ChessFile file, ChessRank rank)
{
  ChessPos  pos;

  pos.m_file = file;
  pos.m_rank = rank;

  return elem(pos);
}

std::vector<ChessPiece> &Game::getBoneYard(ChessColor color)
{
  return color == White? m_boneYardWhite: m_boneYardBlack;
}

int Game::toRow(int rank)
{
  return NumOfRanks - (rank - (int)ChessRank1) - 1;
}

int Game::toCol(int file)
{
  return file - (int)ChessFileA;
}

int Game::toRowCol(const ChessPos &pos, int &row, int &col)
{
  row = toRow(pos.m_rank);
  col = toCol(pos.m_file);
  if( (row < 0) || (row >= NumOfRanks) || (col < 0) || (col >= NumOfFiles) )
  {
    return -CE_ECODE_CHESS_FATAL;
  }
  else
  {
    return CE_OK;
  }
}

ChessFile Game::toFile(int col)
{
  return (ChessFile)((int)ChessFileA + col);
}

ChessRank Game::toRank(int row)
{
  return (ChessRank)((int)ChessRank1 + NumOfRanks - row - 1);
}

ChessColor Game::getSquareColor(int file, int rank)
{
  return ((file - ChessFileA) + (rank - ChessRank1)) % 2 == 0? Black: White;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Protected
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

BoardElem *Game::elem(const ChessPos &pos)
{
  int row, col;

  if( toRowCol(pos, row, col) == CE_OK )
  {
    return &m_board[row][col];
  }
  return NULL;
}

void Game::movePiece(const ChessPos &posFrom, const ChessPos &posTo)
{
  movePiece(elem(posFrom), elem(posTo));
}

void Game::movePiece(BoardElem *pSrc, BoardElem *pDst)
{
  *pDst = *pSrc;
  *pSrc = EmptyElem;
}

void Game::recordHistory(Move &move)
{
  m_history.push_back(move);
}

void Game::moveToBoneYard(BoardElem *pDeadPiece)
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


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &chess_engine::operator<<(ostream &os, const Game &game)
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
