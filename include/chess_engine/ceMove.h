////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceMove.h
//
/*! \file
 *
 * \brief Chess move class interface.
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

#ifndef _CE_MOVE_H
#define _CE_MOVE_H

#include <string>
#include <iostream>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceUtils.h"


namespace chess_engine
{
  /*!
   * \brief Move Class.
   *
   * Each move or action during play if fully captured by instances of this
   * class. The hope is that a robotic entity can play chess without having
   * any deep knowledge of chess nor having to keep extensive game state
   * information.
   */
  class Move
  {
  public:
    int             m_nMove;      ///< move number (2 plies/move)
    ChessColor      m_player;     ///< player (and move) color
    std::string     m_strAN;      ///< algebraic notation of move
    ChessPiece      m_piece;      ///< moved piece
    ChessSquare     m_sqFrom;     ///< moved piece starting from chess square
    ChessSquare     m_sqTo;       ///< moved piece ending to chess square
    ChessPiece      m_captured;   ///< captured piece, if any
    bool            m_en_passant; ///< en passant move did [not] occur
    ChessCastling   m_castle;     ///< [no] castle move
    ChessSquare     m_sqAuxAt;    ///< castle rook or en passant opponent square
    ChessSquare     m_sqAuxTo;    ///< castle rook destination chess square
    ChessPiece      m_promotion;  ///< pawn promoted to this piece
    bool            m_check;      ///< opponent [not] placed in check
    ChessColor      m_winner;     ///< winner of the game, if any
    ChessResult     m_result;     ///< result of move

    Move()
    {
      clear();
    }

    Move(const Move &src)
    {
      copy(src);
    }

    virtual ~Move() { };

    Move operator=(const Move &rhs)
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

    friend std::ostream &operator<<(std::ostream &os, const Move &move);
    
  protected:
    bool  m_bCapture;     ///< some type of piece capture occurred

    void copy(const Move &src);
  };

  extern std::ostream &operator<<(std::ostream &os, const Move &move);
} // namespace chess_engine


#endif // _CE_MOVE_H
