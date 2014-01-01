////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_move.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Chess move class implementation.
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

#include <iostream>
#include <string>

#include <boost/regex.hpp>

#include "chess.h"

#include "chess_move.h"
#include "chess_server.h"

using namespace std;
using namespace chess_engine;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


static string strPiece("[KQRBNP]");
static string strFile("[a-h]");
static string strRank("[1-8]");
static string strRankPenult("[27]");
static string strRankUlt("[18]");

boost::regex reCastleWhiteKingSide("(e1g1|O-O)");
boost::regex reCastleWhiteQueenSide("(e1c1|O-O-O)");
boost::regex reCastleBlackKingSide("(e8g8|O-O)");
boost::regex reCastleBlackQueenSide("(e8c8|O-O-O)");
boost::regex reCastleKingSide("^(e1g1|e8g8|O-O)$");
boost::regex reCastleQueenSide("^(e1c1|e8c8|O-O-O)$");
boost::regex rePawnPromotion("[a-h][27]?[a-h]?[18]=?("+strPiece+")\\S*");
//boost::regex rePawnPromotion("\\w+=?("+strPiece+")$");
boost::regex reSAN("^("+strFile+")("+strRank+")("+strFile+")("+strRank+")$");
boost::regex rePiece("^("+strPiece+").+$");
boost::regex rePawn("^("+strFile+").+$");
boost::regex reCapture(".+(x:).+");
boost::regex reModifier(".+([+#])$");


// -----------------------------------------------------------------------------
// Class ChessMove
// -----------------------------------------------------------------------------

void ChessMove::fromAN(const string &strAN)
{
  boost::cmatch what;
  ChessModifier modifier;

  m_strAN = strAN;

  // king side castle
  if( boost::regex_match(strAN.c_str(), what, reCastleKingSide) )
  {
    m_piece  = King;
    m_castle = KingSide;
  }
  // queen side castle
  else if( boost::regex_match(strAN.c_str(), what, reCastleQueenSide) )
  {
    m_piece  = King;
    m_castle = QueenSide;
  }
  // pawn promotion
  else if( boost::regex_match(strAN.c_str(), what, rePawnPromotion) )
  {
    m_piece     = Pawn;
    m_promotion = (ChessPiece)what[1].str()[0];
  }

  // Standard Algebraic Notation
  else if( boost::regex_match(strAN.c_str(), what, reSAN) )
  {
  }

  // a simple, non-pawn move
  else if( boost::regex_match(strAN.c_str(), what, rePiece) )
  {
    m_piece = (ChessPiece)what[1].str()[0];
  }

  // a simple or en passant pawn move
  else if( boost::regex_match(strAN.c_str(), what, rePawn) )
  {
    m_piece = Pawn;
  }

  // capture
  if( boost::regex_match(strAN.c_str(), what, reCapture) )
  {
    m_bCapture = true;
  }

  // check or checkmate
  if( boost::regex_match(strAN.c_str(), what, reModifier) )
  {
    modifier = (ChessModifier)what[1].str()[0];
    if( modifier == ModCheck )
    {
      m_check = true;
    }
    switch( m_result )
    {
      case NoResult:
      case Ok:
        m_result = modifier == ModCheckmate? Checkmate: Ok;
        break;
      default:
        break;
    }
  }

  if( m_result == NoResult )
  {
    m_result = Ok;
  }
}

string ChessMove::toSAN(const ChessSquare &sqFrom, const ChessSquare &sqTo)
{
  char san[5];

  sprintf(san, "%c%c%c%c", sqFrom.m_file, sqFrom.m_rank,
                           sqTo.m_file,   sqTo.m_rank);

  return string(san);
}

void ChessMove::fromSAN(const string &strSAN)
{
  m_sqFrom = NoMove;
  m_sqTo   = NoMove;

  if( strSAN.size() >= 2 )
  {
    m_sqFrom.m_file = strSAN[0];
    m_sqFrom.m_rank = strSAN[1];
  }
  if( strSAN.size() >= 4 )
  {
    m_sqTo.m_file   = strSAN[2];
    m_sqTo.m_rank   = strSAN[3];
  }
}

void ChessMove::copy(const ChessMove &src)
{
  m_nMove       = src.m_nMove;
  m_color       = src.m_color;
  m_strAN       = src.m_strAN;
  m_piece       = src.m_piece;
  m_sqFrom      = src.m_sqFrom;
  m_sqTo        = src.m_sqTo;
  m_captured    = src.m_captured;
  m_en_passant  = src.m_en_passant;
  m_castle      = src.m_castle;
  m_sqAccAt     = src.m_sqAccAt;
  m_sqAccTo     = src.m_sqAccTo;
  m_promotion   = src.m_promotion;
  m_check       = src.m_check;
  m_winner      = src.m_winner;
  m_result      = src.m_result;

  m_bCapture    = src.m_bCapture;
}

void ChessMove::clear()
{
  m_nMove       = 0;
  m_color       = NoColor;
  m_strAN.clear();
  m_piece       = NoPiece;
  m_sqFrom      = NoMove;
  m_sqTo        = NoMove;
  m_captured    = NoPiece;
  m_en_passant  = false;
  m_castle      = NoCastle;
  m_sqAccAt     = NoMove;
  m_sqAccTo     = NoMove;
  m_promotion   = NoPiece;
  m_check       = false;
  m_winner      = NoColor;
  m_result      = NoResult;

  m_bCapture    = false;
}

ostream &chess_engine::operator<<(ostream &os, const ChessMove &move)
{
  os << move.m_nMove << ":";
  os << " " << nameOfColor(move.m_color);
  os << " " << move.m_sqFrom.m_file << move.m_sqFrom.m_rank
            << move.m_sqTo.m_file << move.m_sqTo.m_rank;
  os << " " << move.m_strAN;
  os << " " << nameOfPiece(move.m_piece);

  if( move.m_captured != NoPiece )
  {
    os << " captured(" << nameOfPiece(move.m_captured) << ")";
  }

  if( move.m_en_passant )
  {
    os << " en_passant(xPawn="
       << move.m_sqAccAt.m_file << move.m_sqAccAt.m_rank << ")";
  }

  if( move.m_castle != NoCastle )
  {
    os << " castle(" << nameOfCastling(move.m_castle) << ", Rook=";
    os << move.m_sqAccAt.m_file << move.m_sqAccAt.m_rank
       << move.m_sqAccTo.m_file << move.m_sqAccTo.m_rank << ")";
  }

  if( move.m_promotion != NoPiece )
  {
    os << " promotion(" << nameOfPiece(move.m_promotion) << ")";
  }

  if( move.m_check )
  {
    os << " check";
  }

  if( move.m_winner != NoColor )
  {
    os << " winner(" << nameOfColor(move.m_winner) << ")";
  }

  if( move.m_result != NoResult )
  {
    os << " " << nameOfResult(move.m_result);
  }

  return os;
}
