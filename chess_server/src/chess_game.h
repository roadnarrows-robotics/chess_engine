////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_game.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The chess game state interface.
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

#ifndef _CHESS_GAME_H
#define _CHESS_GAME_H

#include <string>
#include <vector>
#include <map>

#include "chess.h"

#include "chess_move.h"
#include "chess_server.h"

namespace chess_engine
{
  struct ChessBoardElem
  {
    ChessColor  m_color;
    ChessPiece  m_piece;
  };

  const ChessBoardElem EmptyElem = {NoColor, NoPiece};

  const int NumOfFiles = 8;
  const int NumOfRanks = 8;

  class ChessGame
  {
  public:
    ChessGame();
  
    virtual ~ChessGame() { }
  
    void setupBoard();
  
    int sync(ChessMove &move);

    bool isPlaying()
    {
      return m_bIsPlaying;
    }

    int getNumOfMoves()
    {
    }

    int getGameHistory()
    {
    }

    int toRow(int rank)
    {
      return NumOfRanks - (rank - (int)ChessRank1) - 1;
    }

    int toCol(int file)
    {
      return file - (int)ChessFileA;
    }

    int toRowCol(const ChessSquare &sq, int &row, int &col)
    {
      row = toRow(sq.m_rank);
      col = toCol(sq.m_file);
      if( (row < 0) || (row >= NumOfRanks) || (col < 0) || (col >= NumOfFiles) )
      {
        return -CE_ECODE_CHESS_FATAL;
      }
      else
      {
        return CE_OK;
      }
    }

    ChessBoardElem *elem(const ChessSquare &sq)
    {
      int row, col;

      if( toRowCol(sq, row, col) == CE_OK )
      {
        return &m_board[row][col];
      }
      return NULL;
    }

  protected:
    ChessBoardElem  m_board[NumOfRanks][NumOfFiles];  ///< board matrix
    bool            m_bIsPlaying;

    void movePiece(const ChessSquare &sqFrom, const ChessSquare &sqTo)
    {
      movePiece(elem(sqFrom), elem(sqTo));
    }

    void movePiece(ChessBoardElem *pSrc, ChessBoardElem *pDst)
    {
      *pDst = *pSrc;
      *pSrc = EmptyElem;
    }

    void moveToBoneYard(ChessBoardElem *pDeadPiece);
  };

} // chess_engine

#endif // _CHESS_GAME_H
