////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceSquare.cpp
//
/*! \file
 *
 * \brief The chess board square state implementation.
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

#include "rnr/color.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceBoard.h"

using namespace std;
using namespace chess_engine;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Chess board square class.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ChessSquare::ChessSquare()
{
  m_pos     = NoPos;
  m_eColor  = colorOfSquare(m_pos);
}

void ChessSquare::ChessSquare(const ChessPos     &pos,
                              const ChessFqPiece &fqPiece)
{
  m_pos     = pos;
  m_eColor  = colorOfSquare(pos);
  m_fqPiece = fqPiece;
}

void ChessSquare::ChessSquare(const ChessPos    &pos,
                              const ChessColor  ePieceColor,
                              const ChessPiece  ePieceType,
                              const std::string &strPieceId)
{
  m_pos     = pos;
  m_eColor  = colorOfSquare(pos);
  m_fqPiece.set(ePieceColor, ePieceType, strPiecedId);
}

ChessSquare ChessSquare::operator==(const ChessSquare &rhs)
{
  m_pos     = rhs.m_pos;
  m_eColor  = colorOfSquare(m_pos);
  m_fqPiece = rhs.m_fqPiece;

  return *this;
}

void ChessSquare::setPos(const ChessPos &pos)
{
  m_pos = pos;

  m_eColor = colorOfSquare(m_pos);
}

ChessPos ChessSquare::getPos();
{
  return m_pos;
}

ChessColor ChessSquare::getColor();
{
  return m_eColor;
}

void ChessSquare::setPiece(const ChessFqPiece &fqPiece)
{
  m_fqPiece = fqPiece;
}

void ChessSquare::setPiece(const ChessColor   ePieceColor,
                           const ChessPiece   ePieceType,
                           const std::string &strPieceId)
{
  m_fqPiece.set(ePieceColor, ePieceType, strPiecedId);
}

void ChessSquare::getPiece(ChessFqPiece &fqPiece)
{
  fqPiece = m_fqPiece;
}

void ChessSquare::getPiece(ChessColor  &ePieceColor,
                           ChessPiece  &ePieceType,
                           std::string &strPieceId)
{
  ePieceColor = m_fqPiece.m_ePieceColor;
  ePieceType  = m_fqPiece.m_ePieceType;
  strPieceId  = m_fqPiece.m_strPiecedId;
}

void ChessSquare::copyPiece(ChessSquare &dst)
{
  dst.m_fqPiece = m_fqPiece;
}

void ChessSquare::movePiece(ChessSquare &dst)
{
  copyPiece(dst);
  removePiece();
}

void ChessSquare::removePiece()
{
  m_fqPiece.clear();
}

bool ChessSquare::isEmpty()
{
  return m_fqPiece.m_ePieceType == NoPiece;
}

bool ChessSquare::isOnChessBoard()
{
  return ChessBoard::isOnChessBoard(m_pos);
}

ChessColor ChessSquare::getPieceColor()
{
  return m_fqPiece.m_ePieceColor;
}

ChessPiece ChessSquare::getPieceType()
{
  return m_fqPiece.m_ePieceType;
}

string ChessSquare::getPieceId()
{
  return m_fqPiece.m_strPieceId;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Static Member Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ChessColor ChessSquare::colorOfSquare(const ChessPos &pos)
{
  return ChessSquare::colorOfSquare(pos.m_file, pos.m_rank);
}

ChessColor ChessSquare::colorOfSquare(int file, int rank)
{
  if( ChessBoard::isOnChessBoard(m_pos) )
  {
    return ((file - ChessFileA) + (rank - ChessRank1)) % 2 == 0? Black: White;
  }
  else
  {
    return NoColor;
  }
}
