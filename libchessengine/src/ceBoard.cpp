////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceBoard.cpp
//
/*! \file
 *
 * \brief The chess board state implementation.
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

#include <stdio.h>
#include <ctype.h>

#include <string>
#include <vector>
#include <map>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

using namespace std;
using namespace chess_engine;


static ChessNoSquare = ChessSquare();   ///< "no square" square


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Chess game class.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ChessGame::ChessBoard()
{
  initBoard();

  m_bGui = false;
}

void ChessBoard::clearBoard()
{
  for(int col=0; col<NumOfFiles; ++col)
  {
    for(int row=0; row<NumOfRanks; ++row)
    {
      m_board[row][col].removePiece();
    }
  }
}

void ChessBoard::setupBoard()
{
  clearBoard();

  setupBoard(White);
  setupBoard(Black);
}

void ChessBoard::setPiece(const ChessPos    &pos,
                          const ChessColor   eColor,
                          const ChessPiece   ePiece,
                          const std::string &strId)
{
  at(pos).setPiece(eColor, ePiece, strId);
}

void ChessBoard::movePiece(const ChessPos &posSrc, const ChessPos &posDst)
{
  at(posSrc).movePiece(at(posDst));
}

void ChessBoard::removePiece(const ChessPos &pos)
{
  at(pos).removePiece();
}

ChessSquare &ChessBoard::at(const int file, const int rank)
{
  return getBoardSquare(ChessPos(file, rank));
}

ChessSquare &ChessBoard::at(const ChessPos &pos)
{
  if( toRowCol(pos, row, col) == CE_OK )
  {
    return m_board[row][col];
  }
  else
  {
    return ChessNoSquare;
  }
}

bool ChessBoard::setMoveSrcPos(ChessMove &move)
{
  list_of_pos   positions;
  ChessPos     &src = move.m_posSrc;
  size_t        i;

  if( move.m_ePieceMoved == Pawn )
  {
    findPawnSrcMoves(move.m_ePlayer, move.m_posDst, positions);
  }
  else
  {
    findMajorPieceMoves(move.m_ePieceMoved, move.m_posDst, positions);
  }

  for(i = 0; i < positions.size(); ++i)
  {
    ChessSquare &sq = at(positions[i]);

    if( (move.m_ePieceMoved == sq.getPieceType()) &&
        ((src.m_file == NoFile) || (src.m_file == positions[i].m_file)) &&
        ((src.m_rank == NoRank) || (src.m_rank == positions[i].m_rank)) )
    {
      src = positions[i];
      return true;
    }
  }

  return false;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Static Member Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessBoard::toRow(int rank)
{
  return NumOfRanks - (rank - (int)ChessRank1) - 1;
}

int ChessBoard::toCol(int file)
{
  return file - (int)ChessFileA;
}

int ChessBoard::toRowCol(const ChessPos &pos, int &row, int &col)
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

ChessFile ChessBoard::toFile(int col)
{
  if( (cow >= 0) && (cow < NumOfFiles) )
  {
    return (ChessFile)((int)ChessFileA + col);
  }
  else
  {
    return NoFile;
  }
}

ChessRank ChessBoard::toRank(int row)
{
  if( (row >= 0) && (row < NumOfRanks) )
  {
    return (ChessRank)((int)ChessRank1 + NumOfRanks - row - 1);
  }
  else
  {
    return NoRank;
  }
}

ChessFile ChessBoard::nextFile(int file)
{
  return  toFile(toCol(file) + 1);
}

ChessRank ChessBoard::nextRank(int rank)
{
  return  toRank(toRow(rank) + 1);
}

ChessFile ChessBoard::shiftFile(int file, int offset)
{
  return toFile(toCol(file) + offset);
}

ChessFile ChessBoard::shiftRank(int file, int offset)
{
  return toRank(toRow(rank) + offset);
}

bool ChessBoard::isOnChessBoard(const ChessPos &pos)
{
  int   row, col;

  return toRowCol(pos, row, col) == CE_OK? true: fale;
}

bool ChessBoard::isOnChessBoard(const ChessFile file, const ChessRank rank)
{
  return isOnChessBoard(ChessPos(file, rank));
}

ChessColor ChessBoard::getSquareColor(int file, int rank)
{
  return ChessSquare::colorOfSquare(file, rank);
}

void ChessBoard::findMajorPieceMoves(const ChessPiece ePiece,
                                     const ChessPos   &pos,
                                     list_of_pos      &positions)
{
  switch( ePiece )
  {
    case King:
      findKingMoves(pos, positions);
      break;
    case Queen:
      findQueenMoves(pos, positions);
      break;
    case Bishop:
      findBishopMoves(pos, positions);
      break;
    case Knight:
      findKnightMoves(pos, positions);
      break;
    case Rook:
      findRookMoves(pos, positions);
      break;
    case Pawn:
    default:
      break;
  }
}

void ChessBoard::findKingMoves(const ChessPos &pos, list_of_pos &positions)
{
  ChessPos  p;
  size_t    n = 8;
  int       fileOffset[n] = {0, 1, 1,  1,  0, -1, -1, -1};
  int       rankOffset[n] = {1, 1, 0, -1, -1, -1,  0,  1};
  size_t    i;

  // clockwise from 12 o'clock
  for(i = 0; i < n; ++i)
  {
    p.m_file = shiftFile(pos.m_file, fileOffset[i]);
    p.m_rank = shiftRank(pos.m_rank, rankOffset[i]);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }
}

void ChessBoard::findQueenMoves(const ChessPos &pos, list_of_pos &positions)
{
  findBishopMoves(pos, positions);
  findRookMoves(pos, positions);
}

void ChessBoard::findBishopMoves(const ChessPos &pos, list_of_pos &positions)
{
  ChessPiece  p;
  int         row, col;

  // upper right ray
  for(col = toCol(pos.m_file)+1; col < NumOfFiles; ++col)
  {
    p.m_file = toFile(col);
    for(row = toRow(pos.m_rank)+1; row < NumOfRanks; ++row)
    {
      p.m_rank = toRank(row);
      positions.push_back(p);
    }
  }

  // lower right ray
  for(col = toCol(pos.m_file)-1; col >= 0; --col)
  {
    p.m_file = toFile(col);
    for(row = toRow(pos.m_rank)+1; row < NumOfRanks; ++row)
    {
      p.m_rank = toRank(row);
      positions.push_back(p);
    }
  }

  // lower left ray
  for(col = toCol(pos.m_file)-1; col >= 0; --col)
  {
    p.m_file = toFile(col);
    for(row = toRow(pos.m_rank)-1; row >= 0; --row)
    {
      p.m_rank = toRank(row);
      positions.push_back(p);
    }
  }

  // upper left ray
  for(col = toCol(pos.m_file)+1; col < NumOfFiles; ++col)
  {
    p.m_file = toFile(col);
    for(row = toRow(pos.m_rank)-1; row >= 0; --row)
    {
      p.m_rank = toRank(row);
      positions.push_back(p);
    }
  }
}

void ChessBoard::findKnightMoves(const ChessPos &pos, list_of_pos &positions)
{
  ChessPos  p;
  size_t    n = 8;
  int       fileOffset[n] = {-1, 1, 2,  2,  1, -1, -2, -2};
  int       rankOffset[n] = { 2, 2, 1, -1, -2, -2, -1,  1};
  size_t    i;

  // clockwise from 12 o'clock
  for(i = 0; i < n; ++i)
  {
    p.m_file = shiftFile(pos.m_file, fileOffset[i]);
    p.m_rank = shiftRank(pos.m_rank, rankOffset[i]);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }
}

void ChessBoard::findRookMoves(const ChessPos &pos, list_of_pos &positions)
{
  ChessPiece  p;
  int         row, col;

  // up ray
  col       = toCol(pos.m_file);
  p.m_file  = pos.m_file;
  for(row = toRow(pos.m_rank)+1; row < NumOfRanks; ++row)
  {
    p.m_rank = toRank(row);
    positions.push_back(p);
  }

  // right ray
  row       = toRow(pos.m_rank);
  p.m_rank  = pos.m_rank;
  for(col = toCol(pos.m_file)+1; col < NumOfFiles; ++col)
  {
    p.m_file = toFile(col);
    positions.push_back(p);
  }

  // down ray
  col       = toCol(pos.m_file);
  p.m_file  = pos.m_file;
  for(row = toRow(pos.m_rank)-1; row >= 0; --row)
  {
    p.m_rank = toRank(row);
    positions.push_back(p);
  }

  // left ray
  row       = toRow(pos.m_rank);
  p.m_rank  = pos.m_rank;
  for(col = toCol(pos.m_file)-1; col >= 0; --col)
  {
    p.m_file = toFile(col);
    positions.push_back(p);
  }
}

void ChessBoard::findPawnSrcMoves(const ChessColor eColor,
                                  const ChessPos   &pos,
                                  list_of_pos      &positions)
{
  ChessPos  p
  size_t    n = 3;
  int       fileOffset[n] = {-1, 0, 1};
  int       rankOffset = eColor == White? -1: 1;
  size_t    i;

  p.m_rank = shiftRank(pos.m_rank, rankOffset);

  for(i = 0; i < n; ++i)
  {
    p.m_file = shiftFile(pos.m_file, fileOffset[i]);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }

  if( (eColor == White) && (pos.m_rank == ChessRank4) )
  {
    p.m_file = pos.m_file;
    p.m_rank = shiftRank(pos.m_rank, -2);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }
  else if( (eColor == Black) && (pos.m_rank == ChessRank5) )
  {
    p.m_file = pos.m_file;
    p.m_rank = shiftRank(pos.m_rank, 2);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }
}

void ChessBoard::findPawnDstMoves(const ChessColor eColor,
                                  const ChessPos   &pos,
                                  list_of_pos      &positions)
{
  ChessPos  p
  size_t    n = 3;
  int       fileOffset[n] = {-1, 0, 1};
  int       rankOffset = eColor == White? 1: -1;
  size_t    i;

  p.m_rank = shiftRank(pos.m_rank, rankOffset);

  for(i = 0; i < n; ++i)
  {
    p.m_file = shiftFile(pos.m_file, fileOffset[i]);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }

  if( (eColor == White) && (pos.m_rank == ChessRank2) )
  {
    p.m_file = pos.m_file;
    p.m_rank = shiftRank(pos.m_rank, 2);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }
  else if( (eColor == Black) && (pos.m_rank == ChessRank7) )
  {
    p.m_file = pos.m_file;
    p.m_rank = shiftRank(pos.m_rank, -2);
    if( p.isOnBoard() )
    {
      positions.push_back(p);
    }
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Protected
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessBoard::initBoard()
{
  int       row, col;
  ChessPos  pos;

  for(row = 0; row < NumOfRanks; ++row)
  {
    for(col = 0; col < NumOfFiles; ++col)
    {
      pos.m_file = toFile(col);
      pos.m_rank = toRank(row);
      m_board[row][col].setPos(pos);
    }
  }
}

void ChessBoard::setupBoard(ChessColor eColor)
{
  ChessPiece  ePiece;
  string      strId;
  ChessFile   file;
  ChessRank   rank;
  int         row, col;
  
  // -----
  // Back line
  // -----
  rank = eColor == White? ChessRank1: ChessRank8;
  row  = toRow(rank);

  //
  // Pieces from left to right.
  //
  for(file = ChessFileA; file = nextFile(file); file != NoFile)
  {
    col = toCol(file);

    switch( file )
    {
      // rooks
      case ChessFileA:
      case ChessFileH:
        ePiece = Rook;
        break;

      // knights
      case ChessFileB:
      case ChessFileG:
        ePiece = Knight;
        break;

      // bishops
      case ChessFileC:
      case ChessFileF:
        ePiece = Bishop;
        break;

      // queen
      case ChessFileD:
        ePiece = Queen;
        break;

      // king
      case ChessFileE:
        ePiece = King;
        break;
    }

    strId = ChessFqPiece::makePieceId(file, rank, eColor, ePiece);

    m_board[row][col].setPiece(eColor, ePiece, strId);
  }

  // -----
  // Pawns
  // -----
  rank    = eColor == White? ChessRank2: ChessRank7;
  row     = toRow(rank);
  ePiece  = Pawn;

  //
  // Left to right files.
  //
  for(file = ChessFileA; file = nextFile(file); file != NoFile)
  {
    col   = toCol(file);
    strId = ChessFqPiece::makePieceId(file, rank, eColor, ePiece);

    m_board[row][col].set(eColor, ePiece, strId);
  }
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
