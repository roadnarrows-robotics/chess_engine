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
 * (C) 2013-2017 RoadNarrows LLC
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

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceMove.h"

using namespace std;
using namespace chess_engine;


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
  m_strAN           = src.m_strAN;
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
  m_strAN.clear();
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
      posCapture.m_rank = PawnRankWhiteEPCap;
      break;
    case Black:
      posCapture.m_rank = PawnRankBlackEPCap;
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
  switch( ePlayer )
  {
    case White:
      switch( eCastling )
      {
        case KingSide:
          posSrc = CastlingPosWhiteKSrc;
          posDst = KSidePosWhiteKDst;
          break;
        case QueenSide:
          posSrc = CastlingPosWhiteKSrc;
          posDst = QSidePosWhiteKDst;
          break;
        default:
          posSrc = NoPos;
          posDst = NoPos;
          break;
      }
      break;
    case Black:
      switch( eCastling )
      {
        case KingSide:
          posSrc = CastlingPosBlackKSrc;
          posDst = KSidePosBlackKDst;
          break;
        case QueenSide:
          posSrc = CastlingPosBlackKSrc;
          posDst = QSidePosBlackKDst;
          break;
        default:
          posSrc = NoPos;
          posDst = NoPos;
          break;
      }
      break;
    default:
      posSrc = NoPos;
      posDst = NoPos;
      break;
  }
}

void ChessMove::getCastlingRookMove(const ChessColor    ePlayer,
                                    const ChessCastling eCastling,
                                    ChessPos            &posSrc,
                                    ChessPos            &posDst)
{
  switch( ePlayer )
  {
    case White:
      switch( eCastling )
      {
        case KingSide:
          posSrc = KSidePosWhiteRSrc;
          posDst = KSidePosWhiteRDst;
          break;
        case QueenSide:
          posSrc = QSidePosWhiteRSrc;
          posDst = QSidePosWhiteRDst;
          break;
        default:
          posSrc = NoPos;
          posDst = NoPos;
          break;
      }
      break;
    case Black:
      switch( eCastling )
      {
        case KingSide:
          posSrc = KSidePosBlackRSrc;
          posDst = KSidePosBlackRDst;
          break;
        case QueenSide:
          posSrc = QSidePosBlackRSrc;
          posDst = QSidePosBlackRDst;
          break;
        default:
          posSrc = NoPos;
          posDst = NoPos;
          break;
      }
      break;
    default:
      posSrc = NoPos;
      posDst = NoPos;
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
      << "  AN         = " << move.m_strAN << std::endl
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
    << "}";
  ;

  return os;
}
