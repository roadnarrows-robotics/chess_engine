////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceCore.cpp
//
/*! \file
 *
 * \brief Core chess types.
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

#include <boost/algorithm/string.hpp>    

#include <ros/console.h>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceUtils.h"

using namespace std;
using namespace chess_engine;

// ----------------------------------------------------------------------------
// ChessPos Structure
// ----------------------------------------------------------------------------
ChessPos::ChessPos()
{
  m_file = NoFile;
  m_rank = NoRank;
}

ChessPos::ChessPos(const ChessPos &src)
{
  m_file = src.m_file;
  m_rank = src.m_rank;
}

ChessPos::ChessPos(const int &file, const int &rank)
{
  m_file = (ChessFile)file;
  m_rank = (ChessRank)rank;
}

ChessPos ChessPos::operator=(const ChessPos &rhs)
{
  m_file = rhs.m_file;
  m_rank = rhs.m_rank;

  return *this;
}

void ChessPos::clear()
{
  m_file = NoFile;
  m_rank = NoRank;
}

bool ChessPos::isOnChessBoard() const
{
  int col = m_file - (int)ChessFileA;
  int row = NumOfRanks - (m_rank - (int)ChessRank1) - 1;

  return (col >= 0) && (col < NumOfFiles) && (row >= 0) && (col < NumOfRanks);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// ChessPos Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

std::ostream &chess_engine::operator<<(std::ostream &os, const ChessPos &pos)
{
  os << (char)pos.m_file << (char)pos.m_rank;

  return os;
}


// ----------------------------------------------------------------------------
// ChessFqPiece Class
// ----------------------------------------------------------------------------
ChessFqPiece::ChessFqPiece()
{
  clear();
}

ChessFqPiece::~ChessFqPiece()
{
}

ChessFqPiece::ChessFqPiece(const ChessFqPiece &src)
{
  m_ePieceColor = src.m_ePieceColor;
  m_ePieceType  = src.m_ePieceType;
  m_strPieceId  = src.m_strPieceId;
}

ChessFqPiece ChessFqPiece::operator=(const ChessFqPiece &rhs)
{
  m_ePieceColor = rhs.m_ePieceColor;
  m_ePieceType  = rhs.m_ePieceType;
  m_strPieceId  = rhs.m_strPieceId;

  return *this;
}

void ChessFqPiece::set(const ChessColor  ePieceColor,
                       const ChessPiece  ePieceType, 
                       const std::string &strPieceId)
{
  m_ePieceColor = ePieceColor;
  m_ePieceType  = ePieceType;
  m_strPieceId  = strPieceId;
}

void ChessFqPiece::clear()
{
  m_ePieceColor = NoColor;
  m_ePieceType  = NoPiece;
  m_strPieceId.clear();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Static Member Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

string ChessFqPiece::makePieceId(int file, int rank,
                                 ChessColor eColor,
                                 ChessPiece ePiece)
{
  ChessPos pos(file, rank);

  return makePieceId(pos, eColor, ePiece);
}

string ChessFqPiece::makePieceId(const ChessPos &pos,
                                 ChessColor     eColor,
                                 ChessPiece     ePiece)
{
  stringstream  ss;
  string        strSep("-");

  ss  << nameOfColor(eColor) << strSep
      << pos << strSep
      << nameOfPiece(ePiece);

  string strId(ss.str());

  boost::algorithm::to_lower(strId);

  return strId;
}

string ChessFqPiece::makePieceId(const ChessPos &pos,
                                 ChessColor     eColor,
                                 ChessPiece     ePiece,
                                 int            nInstance)
{
  stringstream  ss;
  string        strSep("-");

  ss  << makePieceId(pos, eColor, ePiece) << strSep << nInstance;

  return ss.str();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// ChessFqPiece Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
std::ostream &chess_engine::operator<<(std::ostream       &os,
                                       const ChessFqPiece &fqPiece)
{
  std::string space(" ");

  os  << "{ "
      << nameOfColor(fqPiece.m_ePieceColor) << space
      << nameOfPiece(fqPiece.m_ePieceType) << space
      << "\"" << fqPiece.m_strPieceId << "\""
      << " }";

  return os;
}

