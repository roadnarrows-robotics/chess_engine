////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceMove.cpp
//
/*! \file
 *
 * \brief Chess move class implementation.
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

#include <iostream>
#include <string>

#include <boost/regex.hpp>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"

using namespace std;
using namespace chess_engine;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


/*
 * \brief Fixed strings used in regular expressions.
 */
static string strFile("[a-h]");
static string strRank("[1-8]");
static string strRankPenult("[27]");
static string strRankUlt("[18]");

static string strPiece("[KQRBNP]");
static string strPieceMajor("[KQRBN]");
static string strPiecePromo("[QRBN]");

static string strCastlingKingside("0-0|O-O");
static string strCastlingQueenside("0-0-0|O-O-O");

static string strModCapture("x|:");
static string strModEnPassant("e.p");
static string strModPromotion("[=/]");
static string strModCheck("+|++");
static string strModCheckmate("#");


/*
 * \brief Coordinate and Standard Algebraic Notation regular expressions.
 *
 * CAN:
 *
 * 
 */

static boost::regex reCastleWhiteKingSide("(e1g1|O-O)");
static boost::regex reCastleWhiteQueenSide("(e1c1|O-O-O)");
static boost::regex reCastleBlackKingSide("(e8g8|O-O)");
static boost::regex reCastleBlackQueenSide("(e8c8|O-O-O)");
static boost::regex reCastleKingSide("^(e1g1|e8g8|O-O)$");
static boost::regex reCastleQueenSide("^(e1c1|e8c8|O-O-O)$");
static boost::regex rePawnPromotion("[a-h][27]?[a-h]?[18]=?("+strPieceMajor+")\\S*");
//static boost::regex rePawnPromotion("\\w+=?("+strPiece+")$");
static boost::regex reCAN("^("+strFile+")("+strRank+")("
                              +strFile+")("+strRank+")$");
static boost::regex rePiece("^("+strPiece+").+$");
static boost::regex rePawn("^("+strFile+").+$");
static boost::regex reCapture(".+(x:).+");
static boost::regex reModifier(".+([+#])$");


// -----------------------------------------------------------------------------
// Class ChessMove
// -----------------------------------------------------------------------------

ChessMove()
{
  clear();
}

ChessMove(const ChessMove &src)
{
  copy(src);
}

~ChessMove()
{
};

ChessMove operator=(const ChessMove &rhs)
{
  copy(rhs);
  return *this;
}





void ChessMove::fromAN(const string &strAN)
{
  boost::cmatch what;

  // king side castle
  if( boost::regex_match(strAN.c_str(), what, reCANPosPos) )
  {
    m_ePieceMoved  = King;
    m_eCastling = KingSide;
  }
}

void ChessMove::fromSAN(const string &strSAN)
{
  boost::cmatch what;
  ChessModifier modifier;

  m_strSAN = strSAN;

  // king side castle
  if( boost::regex_match(strSAN.c_str(), what, reCastleKingSide) )
  {
    m_ePieceMoved  = King;
    m_eCastling = KingSide;
  }
  // queen side castle
  else if( boost::regex_match(strSAN.c_str(), what, reCastleQueenSide) )
  {
    m_ePieceMoved  = King;
    m_eCastling = QueenSide;
  }
  // pawn promotion
  else if( boost::regex_match(strSAN.c_str(), what, rePawnPromotion) )
  {
    m_ePieceMoved     = Pawn;
    m_ePiecePromoted = (ChessPiece)what[1].str()[0];
  }

  // Standard Algebraic Notation
  else if( boost::regex_match(strSAN.c_str(), what, reCAN) )
  {
  }

  // a simple, non-pawn move
  else if( boost::regex_match(strSAN.c_str(), what, rePiece) )
  {
    m_ePieceMoved = (ChessPiece)what[1].str()[0];
  }

  // a simple or en passant pawn move
  else if( boost::regex_match(strSAN.c_str(), what, rePawn) )
  {
    m_ePieceMoved = Pawn;
  }

  // capture
  if( boost::regex_match(strSAN.c_str(), what, reCapture) )
  {
    m_bCapture = true;
  }

  // check or checkmate
  if( boost::regex_match(strSAN.c_str(), what, reModifier) )
  {
    modifier = (ChessModifier)what[1].str()[0];
    if( modifier == ModCheck )
    {
      m_eCheck = true;
    }
    switch( m_eResult )
    {
      case NoResult:
      case Ok:
        m_eResult = modifier == ModCheckmate? Checkmate: Ok;
        break;
      default:
        break;
    }
  }

  if( m_eResult == NoResult )
  {
    m_eResult = Ok;
  }
}

string ChessMove::toCAN(const ChessPos &posFrom, const ChessPos &posTo)
{
  char san[5];

  sprintf(san, "%c%c%c%c", posFrom.m_file, posFrom.m_rank,
                           posTo.m_file,   posTo.m_rank);

  return string(san);
}

void ChessMove::fromCAN(const string &strCAN)
{
  m_posSrc = NoPos;
  m_posDst   = NoPos;

  if( strCAN.size() >= 2 )
  {
    m_posSrc.m_file = strCAN[0];
    m_posSrc.m_rank = strCAN[1];
  }
  if( strCAN.size() >= 4 )
  {
    m_posDst.m_file = strCAN[2];
    m_posDst.m_rank = strCAN[3];
  }
}

void ChessMove::copy(const ChessMove &src)
{
  m_nMoveNum        = src.m_nMoveNum;
  m_ePlayer         = src.m_ePlayer;
  m_strSAN          = src.m_strSAN;
  m_posSrc          = src.m_posSrc;
  m_posDst          = src.m_posDst;
  m_ePieceMoved     = src.m_ePieceMoved;
  m_ePiecedCaptured = src.m_ePiecedCaptured;
  m_ePiecePromoted  = src.m_ePiecePromoted;
  m_bIsEnPassant    = src.m_bIsEnPassant;
  m_eCastling       = src.m_eCastling;
  m_eCheck          = src.m_eCheck;
  m_eWinner         = src.m_eWinner;
  m_eResult         = src.m_eResult;

  m_bCapture    = src.m_bCapture;
}

void ChessMove::clear()
{
  m_nMoveNum       = 0;
  m_ePlayer      = NoColor;
  m_strSAN.clear();
  m_ePieceMoved       = NoPiece;
  m_posSrc     = NoPos;
  m_posDst       = NoPos;
  m_ePiecedCaptured    = NoPiece;
  m_bIsEnPassant  = false;
  m_eCastling      = NoCastle;
  m_ePiecePromoted   = NoPiece;
  m_eCheck       = false;
  m_eWinner      = NoColor;
  m_eResult      = NoResult;

  m_bCapture    = false;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Static Member Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessMove::getEnPassantCapturedPawnPos(const ChessColor ePlayer,
                                            const ChessPos   &posDst,
                                            ChessPos         &posCapture)
{
  posCapture.m_file = posDst.m_file;

  switch( ePlayer )
  {
    case White:
      posCapture.m_rank = ChessRank5;
      break;
    case Black:
      posCapture.m_rank = ChessRank4;
      break;
    default:
      posCapture.m_rank = NoRank;
  }
}

void ChessMove::getCastlingKingMove(const ChessColor    ePlayer,
                                    const ChessCastling eCastling,
                                    ChessPos            &posSrc,
                                    ChessPos            &posDst)
{
  switch( eCastling )
  {
    case KingSide:
      posSrc.m_file = ChessFileE;
      posDst.m_file = ChessRankG;
      break
    case QueenSide:
      posSrc.m_file = ChessFileE;
      posDst.m_file = ChessRankC;
      break
    default:
      posSrc.m_file = NoFile;
      posDst.m_file = NoFile;
      break
  }

  switch( ePlayer )
  {
    case White:
      posSrc.m_rank = ChessRank1;
      posDst.m_rank = ChessRank1;
      break;
    case Black:
      posSrc.m_rank = ChessRank8;
      posDst.m_rank = ChessRank8;
      break;
    default:
      posSrc.m_rank = NoRank;
      posDst.m_rank = NoRank
  }
}

void ChessMove::getCastlingRookMove(const ChessColor    ePlayer,
                                    const ChessCastling eCastling,
                                    ChessPos            &posSrc,
                                    ChessPos            &posDst)
{
  switch( eCastling )
  {
    case KingSide:
      posSrc.m_file = ChessFileH;
      posDst.m_file = ChessRankF;
      break
    case QueenSide:
      posSrc.m_file = ChessFileA;
      posDst.m_file = ChessRankD;
      break
    default:
      posSrc.m_file = NoFile;
      posDst.m_file = NoFile;
      break
  }

  switch( ePlayer )
  {
    case White:
      posSrc.m_rank = ChessRank1;
      posDst.m_rank = ChessRank1;
      break;
    case Black:
      posSrc.m_rank = ChessRank8;
      posDst.m_rank = ChessRank8;
      break;
    default:
      posSrc.m_rank = NoRank;
      posDst.m_rank = NoRank
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &chess_engine::operator<<(ostream &os, const ChessMove &move)
{
  os << move.m_nMoveNum << ":";
  os << " " << nameOfColor(move.m_ePlayer);
  os << " " << move.m_posSrc.m_file << move.m_posSrc.m_rank
            << move.m_posDst.m_file << move.m_posDst.m_rank;
  os << " " << move.m_strSAN;
  os << " " << nameOfPiece(move.m_ePieceMoved);

  if( move.m_ePiecedCaptured != NoPiece )
  {
    os << " captured(" << nameOfPiece(move.m_ePiecedCaptured) << ")";
  }

  if( move.m_bIsEnPassant )
  {
    os << " en_passant";
  }

  if( move.m_eCastling != NoCastle )
  {
    os << " castle(" << nameOfCastling(move.m_eCastling) << ")";
  }

  if( move.m_ePiecePromoted != NoPiece )
  {
    os << " promotion(" << nameOfPiece(move.m_ePiecePromoted) << ")";
  }

  if( move.m_eCheck != NoCheck )
  {
    os << " " << nameOfCheckMod(move.m_eCheck);
  }

  if( move.m_eWinner != NoColor )
  {
    os << " winner(" << nameOfColor(move.m_eWinner) << ")";
  }

  if( move.m_eResult != NoResult )
  {
    os << " " << nameOfResult(move.m_eResult);
  }

  return os;
}
