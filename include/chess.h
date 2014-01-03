////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Nodes: chess_server, rqt_chess_viz
//
// File:      chess.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Defined the game of chess.
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

#ifndef _CHESS_H
#define _CHESS_H

#ifndef SWIG
namespace chess_engine
{
#endif // SWIG

  const char ChessValNone = '0';    ///< comman 'no value' value

  /*!
   * \brief Chess board file (column).
   */
  enum ChessFile
  {
    NoFile      = ChessValNone, ///< no file
    ChessFileA  = 'a',          ///< file a, column 0
    ChessFileB  = 'b',          ///< file b, column 1
    ChessFileC  = 'c',          ///< file c, column 3
    ChessFileD  = 'd',          ///< file d, column 3
    ChessFileE  = 'e',          ///< file e, column 4
    ChessFileF  = 'f',          ///< file f, column 5
    ChessFileG  = 'g',          ///< file g, column 6
    ChessFileH  = 'h'           ///< file h, column 7
  };

  /*!
   * \brief Chess board rank (row).
   *
   * Rank numbering is always relative to White starting from '1' to '8'.
   */
  enum ChessRank
  {
    NoRank      = ChessValNone, ///< no rank
    ChessRank1  = '1',          ///< rank 1, row 0
    ChessRank2  = '2',          ///< rank 2, row 1
    ChessRank3  = '3',          ///< rank 3, row 2
    ChessRank4  = '4',          ///< rank 4, row 3
    ChessRank5  = '5',          ///< rank 5, row 4
    ChessRank6  = '6',          ///< rank 6, row 5
    ChessRank7  = '7',          ///< rank 7, row 6
    ChessRank8  = '8'           ///< rank 8, row 7
  };

  /*!
   * \brief Chess square.
   */
  struct ChessSquare
  {
    char  m_file;   ///< file (column)
    char  m_rank;   ///< rank (row)
  };

  const ChessSquare NoMove = {NoFile, NoRank};  ///< no move or square

  /*!
   * \brief Chess player, piece, and move color.
   */
  enum ChessColor
  {
    NoColor = ChessValNone, ///< no color
    White   = 'w',          ///< white
    Black   = 'b'           ///< black
  };

  /*!
   * \brief Chess piece type.
   */
  enum ChessPiece
  {
    NoPiece = ChessValNone, ///< no piece
    King    = 'K',          ///< king
    Queen   = 'Q',          ///< queen
    Rook    = 'R',          ///< rook or castle
    Bishop  = 'B',          ///< bishop
    Knight  = 'N',          ///< knight
    Pawn    = 'P'           ///< pawn
  };

  /*!
   * \brief Chess castling.
   */
  enum ChessCastling
  {
    NoCastle  = ChessValNone, ///< no castle
    KingSide  = 'K',          ///< king side castle
    QueenSide = 'Q'           ///< queen side castle
  };

  /*!
   * \brief Chess move modifier.
   */
  enum ChessModifier
  {
    ModCheck      = '+',        ///< check
    ModCheckmate  = '#'         ///< checkmate
  };

  /*!
   * \brief Chess action result.
   */
  enum ChessResult
  {
    NoResult    = ChessValNone, ///< no result
    Ok          = 'y',          ///< good move
    BadMove     = 'n',          ///< invalid move attempt
    OutOfTurn   = '?',          ///< out-of-turn move attempt
    Checkmate   = '#',          ///< checkmate
    Draw        = 'd',          ///< game is a draw
    Resign      = 'r',          ///< player or engine resigned
    NoGame      = '$',          ///< no active game
    GameFatal   = '!'           ///< current game has unrecoverable errors
  };

#ifndef SWIG
} // namespace chess_engine
#endif // SWIG


#endif // _CHESS_H
