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

  const char None = 0;

  /*!
   * \brief Chess board file (column).
   */
  enum ChessFile
  {
    NoFile      = None,
    ChessFileA  = 'a',
    ChessFileB  = 'b',
    ChessFileC  = 'c',
    ChessFileD  = 'd',
    ChessFileE  = 'e',
    ChessFileF  = 'f',
    ChessFileG  = 'g',
    ChessFileH  = 'h'
  };

  /*!
   * \brief Chess board rank (row).
   *
   * Rank numbering is always relative to White starting from '1' to '8'.
   */
  enum ChessRank
  {
    NoRank      = None,
    ChessRank1  = '1',
    ChessRank2  = '2',
    ChessRank3  = '3',
    ChessRank4  = '4',
    ChessRank5  = '5',
    ChessRank6  = '6',
    ChessRank7  = '7',
    ChessRank8  = '8'
  };

  /*!
   * \brief 
   */
  struct ChessSquare
  {
    char  m_file;
    char  m_rank;
  };

  const ChessSquare NoMove = {NoFile, NoRank};

  /*!
   * \brief Chess Piece Color
   */
  enum ChessPieceColor
  {
    NoColor = None,
    White   = 'w',
    Black   = 'b'
  };

  /*!
   * \brief Chess Piece Type
   */
  enum ChessPieceType
  {
    NoPiece = None,
    King    = 'k',
    Queen   = 'q',
    Rook    = 'r',
    Bishop  = 'b',
    Knight  = 'n',
    Pawn    = 'p'
  };

  /*!
   * \brief Chess Castling
   */
  enum ChessCastling
  {
    NoCastling = None,
    Kingside   = 'k',
    Queenside  = 'q'
  };

  typedef ChessPieceType ChessPawnPromotion;

  /*!
   * \brief 
   */
  enum ChessCapture
  {
    NoCapture = None,
    Capture   = 'x'
  };

  /*!
   * \brief 
   */
  enum ChessMoveModifier
  {
    NoModifier  = None,
    Check       = '+',
    Checkmate   = '#',
    Draw        = 'd',
    Resign      = 'r'
  };

  class ChessMove
  {
  public:
    int                 m_nMoveNum;
    ChessPieceColor     m_colorPlayer;
    ChessSquare         m_squareFrom;
    ChessSquare         m_squareTo;
    ChessCapture        m_capture;
    ChessCastling       m_castling;
    ChessPawnPromotion  m_promotion;
    ChessMoveModifier   m_modifier;

    ChessMove() { };
    ~ChessMove() { };
  };

#ifndef SWIG
} // namespace chess_engine
#endif // SWIG


#endif // _CHESS_H
