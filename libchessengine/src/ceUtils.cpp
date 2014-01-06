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
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Common chess utilities.
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
  static map<int, string> NameColors = map_list_of
    (NoColor, "nocolor")
    (White,   "white")
    (Black,   "black");
  
  /*! name of chess pieces */
  static map<int, string> NamePieces = map_list_of
    (NoPiece, "nopiece")
    (King,    "King")
    (Queen,   "Queen")
    (Rook,    "Rook")
    (Bishop,  "Bishop")
    (Knight,  "Knight")
    (Pawn,    "Pawn");
  
  /*! unicode of white chess figurines */
  static map<int, string> FigurineWhitePieces = map_list_of
    (NoPiece,   " ")
    (King,    "\U00002654")
    (Queen,   "\U00002655")
    (Rook,    "\U00002656")
    (Bishop,  "\U00002657")
    (Knight,  "\U00002658")
    (Pawn,    "\U00002659");
  
  /*! unicode of black chess figurines */
  static map<int, string> FigurineBlackPieces = map_list_of
    (NoPiece,   " ")
    (King,    "\U0000265A")
    (Queen,   "\U0000265B")
    (Rook,    "\U0000265C")
    (Bishop,  "\U0000265D")
    (Knight,  "\U0000265E")
    (Pawn,    "\U0000265F");
  
  /*! name of chess castle moves */
  static map<int, string> NameCastling = map_list_of
    (NoCastle,  "nocastle")
    (KingSide,  "kingside")
    (QueenSide, "queenside");
  
  /*! name of chess action results */
  static map<int, string> NameResults = map_list_of
    (NoResult,  "noresult")
    (Ok,        "ok")
    (BadMove,   "badmove")
    (OutOfTurn, "outofturn")
    (Checkmate, "checkmate")
    (Draw,      "draw")
    (Resign,    "resign")
    (NoGame,    "nogame")
    (GameFatal, "gamefatal");
  
  string nameOfColor(ChessColor color)
  {
    return NameColors[color];
  }
  
  string nameOfPiece(ChessPiece piece)
  {
    return NamePieces[piece];
  }
  
  string figurineOfPiece(ChessColor color, ChessPiece piece)
  {
    return color==White? FigurineWhitePieces[piece]: FigurineBlackPieces[piece];
  }
  
  string nameOfCastling(ChessCastling side)
  {
    return NameCastling[side];
  }
  
  string nameOfResult(ChessResult result)
  {
    return NameResults[result];
  }

} // namespace chess_engine
