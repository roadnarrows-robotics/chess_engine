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
#include <sstream>
#include <string>

#include <boost/regex.hpp>

#include <ros/console.h>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceMove.h"

using namespace std;
using namespace chess_engine;


// -----------------------------------------------------------------------------
// Private Interface 
// -----------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Algebraic Notation Regular Expressions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Regular expression helper macros.
 */
#define SOL "^"                           ///< start of line
#define EOL "$"                           ///< end of line
#define G(expr) string("(" + expr + ")")  ///< match group expression

/*!
 * \brief Helper (marked) sub-expression strings used in regular expressions.
 */
static const string exprFile("[a-h]");
static const string exprRank("[1-8]");
static const string exprRankPenult("[27]");
static const string exprRankUlt("[18]");

static const string exprPiece("[KQRBNP]");
static const string exprPieceMajor("[KQRBN]");
static const string exprPiecePromo("[QRBN]");

static const string exprCastlingKingSide("0-0|O-O");
static const string exprCastlingQueenSide("0-0-0|O-O-O");

static const string exprModCapture("x|:");
static const string exprModEnPassant("e.p");
static const string exprModPromotion("[=/]");
static const string exprModCheck("\\+|\\+\\+|#");

static const string grpDisambig(G(exprFile + "?" + exprRank + "?"));
static const string grpPos(G(exprFile+exprRank));
static const string grpPosPenult(G(exprFile + exprRankPenult));
static const string grpPosUlt(G(exprFile + exprRankUlt));

/*!
 * \brief Coordinate and Standard Algebraic Notation regular expressions.
 */

// 2 groups. "a2a4" "g6a6" ...
static boost::regex reCAN(SOL + grpPos + grpPos + EOL);

// 3 groups. "c7c8=Q" "b2a1/N" "d2d1Q" ...
static boost::regex reCANPromotion1(SOL + grpPosPenult + grpPosUlt
                                        + exprModPromotion
                                        + G(exprPiecePromo)
                                      + EOL);

// 3 groups. "c7c8(Q)" ...
static boost::regex reCANPromotion2(SOL + grpPosPenult + grpPosUlt
                                        + "\\("+G(exprPiecePromo)+"\\)"
                                      + EOL);

// 1 group. 0-0 ...
static boost::regex reCastleKingSide(SOL + G(exprCastlingKingSide) + EOL);

// 1 group. 0-0-0 ...
static boost::regex reCastleQueenSide(SOL + G(exprCastlingQueenSide) + EOL);

// 2 groups. "bxc6e.p" "gxf3e.p" "b5xc6e.p" ...
static boost::regex reEnPassant(SOL + grpDisambig + exprModCapture
                                    + grpPos + exprModEnPassant
                                  + EOL);

// 3 groups. "c8=Q" "bxa1/Q" "d1Q" "axb8=N" ...
static boost::regex rePawnPromotion1(SOL + grpDisambig + ".?" + grpPos
                                         + exprModPromotion + G(exprPiecePromo)
                                      + EOL);

// 3 groups. "c7c8(Q)" "dxc8(R)" ...
static boost::regex rePawnPromotion2(SOL + grpDisambig + ".?" + grpPos
                                        + "\\("+G(exprPiecePromo)+"\\)"
                                      + EOL);

// 2 groups. "a4" "exf5" "a2a4" ...
static boost::regex rePawnMove(SOL + grpDisambig + ".?" + grpPos + EOL);

// 3 groups. "Ba4" "Qxf5" "Ra2a4" ...
static boost::regex reMajorMove(SOL + G(exprPieceMajor)
                                    + grpDisambig + ".?" + grpPos
                                  + EOL);

// 1 group. "x" ...
static boost::regex reModCapture(".+" + G(exprModCapture) + ".+");

// 1 group. "+" ...
static boost::regex reModCheck(".+" + G(exprModCheck) + EOL);

/*!
 * \brief RegEx helper to parse position.
 *
 * \param strExpr   Move (sub)position.
 * \param [out] pos Position.
 */
static void _position(const string &strExpr, ChessPos &pos)
{
  pos.m_file = (ChessFile)strExpr[0];
  pos.m_rank = (ChessRank)strExpr[1];
}

/*!
 * \brief RegEx helper to parse disambiguous position.
 *
 * \param strExpr   Move (sub)position.
 * \param [out] pos Position.
 */
static void _disambiguate(const string &strExpr, ChessPos &pos)
{
  if( strExpr.size() == 2 )
  {
    _position(strExpr, pos);
  }
  else if( (strExpr[0] >= (char)ChessFileA) && 
           (strExpr[0] <= (char)ChessFileH) )
  {
    pos.m_file = (ChessFile)strExpr[0];
  }
  else if( (strExpr[0] >= (char)ChessRank1) && 
           (strExpr[0] <= (char)ChessRank8) )
  {
    pos.m_rank = (ChessRank)strExpr[0];
  }
}


// -----------------------------------------------------------------------------
// Class ChessMove
// -----------------------------------------------------------------------------

ChessMove::ChessMove()
{
  clear();
}

ChessMove::ChessMove(const ChessMove &src)
{
  copy(src);
}

ChessMove::~ChessMove()
{
};

ChessMove ChessMove::operator=(const ChessMove &rhs)
{
  copy(rhs);
  return *this;
}

string ChessMove::CAN()
{
  return CAN(m_posSrc, m_posDst, m_ePiecePromoted);
}

string ChessMove::SAN()
{
  return SAN(*this);
}

void ChessMove::copy(const ChessMove &src)
{
  m_nMoveNum        = src.m_nMoveNum;
  m_ePlayer         = src.m_ePlayer;
  m_strSAN          = src.m_strSAN;
  m_ePieceMoved     = src.m_ePieceMoved;
  m_posSrc          = src.m_posSrc;
  m_posDst          = src.m_posDst;
  m_ePieceCaptured  = src.m_ePieceCaptured;
  m_ePiecePromoted  = src.m_ePiecePromoted;
  m_bIsEnPassant    = src.m_bIsEnPassant;
  m_eCastling       = src.m_eCastling;
  m_eCheck          = src.m_eCheck;
  m_eResult         = src.m_eResult;
}

void ChessMove::clear()
{
  m_nMoveNum        = 0;
  m_ePlayer         = NoColor;
  m_strSAN.clear();
  m_ePieceMoved     = NoPiece;
  m_posSrc.clear();
  m_posDst.clear();
  m_ePieceCaptured  = NoPiece;
  m_ePiecePromoted  = NoPiece;
  m_bIsEnPassant    = false;
  m_eCastling       = NoCastling;
  m_eCheck          = NoCheckMod;
  m_eResult         = NoResult;
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
      posDst.m_file = ChessFileG;
      break;
    case QueenSide:
      posSrc.m_file = ChessFileE;
      posDst.m_file = ChessFileC;
      break;
    default:
      posSrc.m_file = NoFile;
      posDst.m_file = NoFile;
      break;
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
      posDst.m_rank = NoRank;
      break;
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
      posDst.m_file = ChessFileF;
      break;
    case QueenSide:
      posSrc.m_file = ChessFileA;
      posDst.m_file = ChessFileD;
      break;
    default:
      posSrc.m_file = NoFile;
      posDst.m_file = NoFile;
      break;
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
      posDst.m_rank = NoRank;
      break;
  }
}

string ChessMove::CAN(ChessMove &move)
{
  return CAN(move.m_posSrc, move.m_posDst, move.m_ePiecePromoted);
}

string ChessMove::CAN(const ChessPos    &posSrc,
                      const ChessPos    &posDst,
                      const ChessPiece  ePiecePromoted)
{
  stringstream  ss;

  ss << posSrc << posDst;

  if( ePiecePromoted != NoPiece )
  {
    ss << "=" << (char)ePiecePromoted;
  }

  return ss.str();
}

string ChessMove::SAN(ChessMove &move)
{
  return "notsupported";
}

int ChessMove::parseCAN(const string &strCAN, ChessMove &move)
{
  boost::cmatch what;

  // pawn promotion
  if( boost::regex_match(strCAN.c_str(), what, reCANPromotion1) ||
      boost::regex_match(strCAN.c_str(), what, reCANPromotion2) )
  {
    move.m_ePiecePromoted = (ChessPiece)what[3].str()[0];
  }

  // basic coordinate move
  else if( !boost::regex_match(strCAN.c_str(), what, reCAN) )
  {
    return -CE_ECODE_CHESS_PARSE;
  }

  _position(what[1].str(), move.m_posSrc);
  _position(what[2].str(), move.m_posDst);

  return CE_OK;
}

int ChessMove::parseSAN(const string &strSAN, ChessMove &move)
{
  boost::cmatch what;

  // king side castle
  if( boost::regex_match(strSAN.c_str(), what, reCastleKingSide) )
  {
    move.m_ePieceMoved = King;
    move.m_eCastling   = KingSide;
  }

  // queen side castle
  else if( boost::regex_match(strSAN.c_str(), what, reCastleQueenSide) )
  {
    move.m_ePieceMoved = King;
    move.m_eCastling   = QueenSide;
  }

  // en passant
  else if( boost::regex_match(strSAN.c_str(), what, reEnPassant) )
  {
    move. m_ePieceMoved = Pawn;
    _disambiguate(what[1].str(), move.m_posSrc);
    _position(what[2].str(),     move.m_posDst);
  }

  // pawn promotion
  else if( boost::regex_match(strSAN.c_str(), what, rePawnPromotion1) ||
           boost::regex_match(strSAN.c_str(), what, rePawnPromotion2) )
  {
    move.m_ePieceMoved = Pawn;
    _disambiguate(what[1].str(), move.m_posSrc);
    _position(what[2].str(), move.m_posDst);
    move.m_ePiecePromoted = (ChessPiece)what[3].str()[0];
  }

  // pawn move
  else if( boost::regex_match(strSAN.c_str(), what, rePawnMove) )
  {
    move.m_ePieceMoved    = Pawn;
    _disambiguate(what[1].str(), move.m_posSrc);
    _position(what[2].str(), move.m_posDst);
  }

  // major piece move
  else if( boost::regex_match(strSAN.c_str(), what, reMajorMove) )
  {
    move.m_ePieceMoved = (ChessPiece)what[1].str()[0];
    _disambiguate(what[2].str(), move.m_posSrc);
    _position(what[3].str(), move.m_posDst);
  }

  else
  {
    return -CE_ECODE_CHESS_PARSE;
  }

  // capture
  if( boost::regex_match(strSAN.c_str(), what, reModCapture) )
  {
    move.m_ePieceCaptured = move.m_bIsEnPassant? Pawn: UndefPiece;
  }

  // check or checkmate
  if( boost::regex_match(strSAN.c_str(), what, reModCheck) )
  {
    if( what[1].str() == "+" )
    {
      move.m_eCheck = ModCheck;
    }
    else if( what[1].str() == "++" )
    {
      move.m_eCheck = ModDoubleCheck;
    }
    else if( what[1].str() == "#" )
    {
      move.m_eCheck = ModCheckmate;
    }
  }

  return CE_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief The ChessMove output stream operator.
 */
std::ostream &chess_engine::operator<<(std::ostream &os,
                                       const chess_engine::ChessMove &move)
{
  os
    << "{" << std::endl
      << "  move       = " << move.m_nMoveNum << std::endl
      << "  player     = "
        << nameOfColor(move.m_ePlayer)
        << "(" << (char)move.m_ePlayer << ")"
        << std::endl
      << "  SAN        = " << move.m_strSAN << std::endl
      << "  piece      = "
        << nameOfPiece(move.m_ePieceMoved)
        << "(" << (char)move.m_ePieceMoved << ")"
        << std::endl
      << "  srcdst     = " << move.m_posSrc << move.m_posDst << std::endl
      << "  captured   = "
        << nameOfPiece(move.m_ePieceCaptured)
        << "(" << (char)move.m_ePieceCaptured << ")"
        << std::endl
      << "  promoted   = "
        << nameOfPiece(move.m_ePiecePromoted)
        << "(" << (char)move.m_ePiecePromoted << ")"
        << std::endl
      << "  en_passant = "
        << nameOfBool(move.m_bIsEnPassant) << "(" << move.m_bIsEnPassant << ")"
        << std::endl
      << "  castling   = "
        << nameOfCastling(move.m_eCastling)
        << "(" << (char)move.m_eCastling << ")"
        << std::endl
      << "  check      = "
        << nameOfCheckMod(move.m_eCheck)
        << "(" << (char)move.m_eCheck << ")"
        << std::endl
      << "  result     = "
        << nameOfResult(move.m_eResult)
        << "(" << (char)move.m_eResult << ")"
        << std::endl
    << "}" << std::endl
  ;

  return os;
}
