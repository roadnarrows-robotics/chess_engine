////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_move.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Chess move class interface.
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

#ifndef _CHESS_MOVE_H
#define _CHESS_MOVE_H

#include <string>
#include <iostream>

#include "chess.h"


namespace chess_engine
{
  /*!
   * \brief ChessMove Class.
   *
   * Each move or action during play if fully captured by instances of this
   * class. The hope is that a robotic entity can play chess without having
   * any deep knowledge of chess nor having to keep extensive game state
   * information.
   */
  class ChessMove
  {
  public:
    int             m_nMove;      ///< move number (2 plies/move)
    ChessColor      m_color;      ///< move (and player) color
    std::string     m_strAN;      ///< algebraic notation of move
    ChessPiece      m_piece;      ///< moved piece
    ChessSquare     m_sqFrom;     ///< moved piece starting from chess square
    ChessSquare     m_sqTo;       ///< moved piece ending to chess square
    ChessPiece      m_captured;   ///< captured piece, if any
    bool            m_en_passant; ///< en passant move did [not] occur
    ChessCastling   m_castle;     ///< [no] castle move
    ChessSquare     m_sqAccAt;    ///< castle rook or en passant opponent square
    ChessSquare     m_sqAccTo;    ///< castle rook destination chess square
    ChessPiece      m_promotion;  ///< pawn promoted to this piece
    bool            m_check;      ///< [not] in check
    ChessColor      m_winner;     ///< winner of the game, if any
    ChessResult     m_result;     ///< result of move

    ChessMove()
    {
      clear();
    }

    ChessMove(const ChessMove &src)
    {
      copy(src);
    }

    virtual ~ChessMove() { };

    ChessMove operator=(const ChessMove &rhs)
    {
      copy(rhs);
      return *this;
    }
  
    std::string toSAN()
    {
      return toSAN(m_sqFrom, m_sqTo);
    }

    static std::string toSAN(const ChessSquare &sqFrom,
                             const ChessSquare &sqTo);

    void fromSAN(const std::string &strSAN);

    void fromAN(const std::string &strAN);

    void clear();

    friend std::ostream &operator<<(std::ostream &os, const ChessMove &move);
    
  protected:
    bool  m_bCapture;     ///< some type of piece capture occurred

    void copy(const ChessMove &src);
  };

  extern std::ostream &operator<<(std::ostream &os, const ChessMove &move);
} // namespace chess_engine


#endif // _CHESS_MOVE_H
