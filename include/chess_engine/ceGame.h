////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceGame.ce
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

#ifndef _CE_GAME_H
#define _CE_GAME_H

#include <iostream>
#include <string>
#include <vector>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceUtils.h"

namespace chess_engine
{
  struct BoardElem
  {
    ChessColor  m_color;
    ChessPiece  m_piece;
  };

  const BoardElem EmptyElem = {NoColor, NoPiece};

  class Game
  {
  public:
    Game();
  
    virtual ~Game() { }
  
    void setupBoard();
  
    int sync(Move &move);

    bool isPlaying()
    {
      return m_bIsPlaying;
    }

    void stopPlaying(ChessResult reason, ChessColor winner=NoColor)
    {
      m_bIsPlaying  = false;
      m_endReason   = reason;
      m_winner      = winner;
    }

    int getNumOfMoves()
    {
      return ((int)m_history.size() + 1) / 2;
    }

    int getNumOfPlies()
    {
      return (int)m_history.size();
    }

    ChessColor getWinner()
    {
      return m_winner;
    }

    ChessResult getEndOfGameReason()
    {
      return m_endReason;
    }

    std::vector<Move> &getGameHistory()
    {
      return m_history;
    }

    Move &operator[](int i)
    {
      return m_history[i];
    }

    BoardElem *getBoardElem(ChessFile file, ChessRank rank)
    {
      ChessSquare sq;

      sq.m_file = file;
      sq.m_rank = rank;

      return elem(sq);
    }

    std::vector<ChessPiece> &getBoneYard(ChessColor color)
    {
      return color == White? m_boneYardWhite: m_boneYardBlack;
    }

    static int toRow(int rank)
    {
      return NumOfRanks - (rank - (int)ChessRank1) - 1;
    }

    static int toCol(int file)
    {
      return file - (int)ChessFileA;
    }

    static int toRowCol(const ChessSquare &sq, int &row, int &col)
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

    static ChessColor getSquareColor(int file, int rank)
    {
      return ((file - ChessFileA) + (rank - ChessRank1)) % 2 == 0? Black: White;
    }

    void setGuiState(bool on_off)
    {
      m_bGui = on_off;
    }

  protected:
    BoardElem               m_board[NumOfRanks][NumOfFiles]; ///< board matrix
    bool                    m_bIsPlaying;     ///< is [not] playing a game
    ChessResult             m_endReason;      ///< end of game reason
    ChessColor              m_winner;         ///< end of game winner, if any
    std::vector<Move>       m_history;        ///< game history
    std::vector<ChessPiece> m_boneYardWhite;  ///< captured white pieces
    std::vector<ChessPiece> m_boneYardBlack;  ///< captured black pieces
    bool                    m_bGui;           ///< [not] gui stream output

    BoardElem *elem(const ChessSquare &sq)
    {
      int row, col;

      if( toRowCol(sq, row, col) == CE_OK )
      {
        return &m_board[row][col];
      }
      return NULL;
    }

    void movePiece(const ChessSquare &sqFrom, const ChessSquare &sqTo)
    {
      movePiece(elem(sqFrom), elem(sqTo));
    }

    void movePiece(BoardElem *pSrc, BoardElem *pDst)
    {
      *pDst = *pSrc;
      *pSrc = EmptyElem;
    }

    void recordHistory(Move &move);

    void moveToBoneYard(BoardElem *pDeadPiece);

    friend std::ostream &operator<<(std::ostream &os, const Game &game);
  };

  extern std::ostream &operator<<(std::ostream &os, const Game &game);

} // chess_engine

#endif // _CE_GAME_H
