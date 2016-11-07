////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceUtils.cpp
//
/*! \file
 *
 * \brief Common chess utilities.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

#include "boost/assign.hpp"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceUtils.h"

using namespace std;
using namespace boost::assign;


namespace chess_engine
{
  /*! name of chess colors */
  static const map<int, string> NameColors = map_list_of
    (NoColor, "nocolor")
    (White,   "white")
    (Black,   "black")
  ;
  
  /*! name of chess pieces */
  static const map<int, string> NamePieces = map_list_of
    (NoPiece, "nopiece")
    (King,    "King")
    (Queen,   "Queen")
    (Rook,    "Rook")
    (Bishop,  "Bishop")
    (Knight,  "Knight")
    (Pawn,    "Pawn")
  ;
  
  /*! unicode of white chess figurines */
  static const map<int, string> FigurineWhitePieces = map_list_of
    (NoPiece,   " ")
    (King,    "\U00002654")
    (Queen,   "\U00002655")
    (Rook,    "\U00002656")
    (Bishop,  "\U00002657")
    (Knight,  "\U00002658")
    (Pawn,    "\U00002659")
  ;
  
  /*! unicode of black chess figurines */
  static const map<int, string> FigurineBlackPieces = map_list_of
    (NoPiece,   " ")
    (King,    "\U0000265A")
    (Queen,   "\U0000265B")
    (Rook,    "\U0000265C")
    (Bishop,  "\U0000265D")
    (Knight,  "\U0000265E")
    (Pawn,    "\U0000265F")
  ;
  
  /*! name of chess castle moves */
  static const map<int, string> NameCastling = map_list_of
    (NoCastling,  "nocastling")
    (KingSide,    "kingside")
    (QueenSide,   "queenside")
  ;
  
  /*! name of chess action results */
  static const map<int, string> NameResults = map_list_of
    (NoResult,      "noresult")
    (Ok,            "ok")
    (BadMove,       "badmove")
    (OutOfTurn,     "outofturn")
    (Checkmate,     "checkmate")
    (Draw,          "draw")
    (Resign,        "resign")
    (Disqualified,  "disqualified")
    (NoGame,        "nogame")
    (GameFatal,     "gamefatal")
  ;
  
  /*! name of chess check modifiers */
  static const std::map<int, std::string> NameCheckModName = map_list_of
    (NoCheckMod,      "nocheck")
    (ModCheck,        "check")
    (ModDoubleCheck,  "doublecheck")
    (ModCheckmate,    "checkmate")
  ;

  /*! name of chess algebras */
  static const std::map<int, std::string> NameAlgebra = map_list_of
    (UnknownAN, "unknown")
    (CAN,       "Coordinate Algebra Notation")
    (SAN,       "Standard Algebra Notation")
  ;

  // name map iterator type
  typedef std::map<int, std::string>::const_iterator name_iter;

  const string nameOfColor(ChessColor color)
  {
    name_iter iter = NameColors.find(color);

    return iter != NameColors.end()? iter->second: NameColors.at(NoColor);
  }
  
  const string nameOfPiece(ChessPiece piece)
  {
    name_iter iter = NamePieces.find(piece);

    return iter != NamePieces.end()? iter->second: NamePieces.at(NoPiece);
  }
  
  const string figurineOfPiece(ChessColor color, ChessPiece piece)
  {
    static const strUnknown = "\U00002047"; // "??"

    name_iter iter;

    switch( color )
    {
      case White:
        iter = FigurineWhitePieces.find(piece);
        return iter != FigurineWhitePieces.end()? iter->second: strUnknown;
      case Black:
        iter = FigurineBlackPieces.find(piece);
        return iter != FigurineBlackPieces.end()? iter->second: strUnknown;
      default:
        return strUnknown;
    }
  }
  
  const string nameOfCastling(ChessCastling side)
  {
    name_iter iter = NameCastling.find(side);

    return iter != NameCastling.end()?  iter->second:
                                        NameCastling.at(NoCastling);
  }
  
  const string nameOfResult(ChessResult result)
  {
    name_iter iter = NameResults.find(result);

    return iter != NameResults.end()? iter->second: NameResults.at(NoResult);
  }

  const string nameOfCheckMod(ChessCheckMod checkmod)
  {
    name_iter iter = NameCheckMod.find(checkmod);

    return iter != NameCheckMod.end()? iter->second:
                                       NameCheckMod.at(NoCheckMod);
  }

  const string nameOfAlgebra(ChessAlgebra algebra)
  {
    name_iter iter = NameAlgebra.find(algebra);

    return iter != NameAlgebra.end()? iter->second: NameAlgebra.at(UnknownAN);
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // ChessPos Structure
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
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

  void ChessPos::clear()
  {
    m_file = NoFile;
    m_rank = NoRank;
  }

  ChessPos::isOnBoard()
  {
    int col = m_file - (int)ChessFileA;
    int row = NumOfRanks - (m_rank - (int)ChessRank1) - 1;

    return (col >= 0) && (col < NumOfFiles) && (row >= 0) && (col < NumOfRanks);
  }

} // namespace chess_engine
