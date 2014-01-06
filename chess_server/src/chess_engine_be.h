
////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_engine_be.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The chess engine backend base class.
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

#ifndef _CHESS_ENGINE_BE_H
#define _CHESS_ENGINE_BE_H

#include <sys/types.h>

#include "rnr/rnrconfig.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"

namespace chess_engine
{
  class ChessEngineBe
  {
  public:
    ChessEngineBe() : m_strVersion("0.0.0")
    {
      m_bIsConn     = false;
      m_bIsPlaying  = false;
      m_fDifficulty = 1.0; 
      m_colorPlayer = NoColor;
      m_colorEngine = NoColor;
      m_colorTurn   = NoColor;
      m_nMove       = 0;
      m_eEoGReason  = NoGame;
    }
  
    virtual ~ChessEngineBe() { }
  

    //..........................................................................
    // High-Level Game Interface
    //..........................................................................
    
    virtual int openConnection()
    {
      return -CE_ECODE_NO_SUPP;
    }
  
    virtual int closeConnection()
    {
      return -CE_ECODE_NO_SUPP;
    }
  
    virtual std::string getVersion()
    {
      return m_strVersion;
    }

    virtual int setGameDifficulty(float fDifficulty)
    {
      if( fDifficulty < 1.0 )
      {
        fDifficulty = 1.0;
      }
      else if( fDifficulty > 10.0 )
      {
        fDifficulty = 10.0;
      }

      m_fDifficulty = fDifficulty;

      return CE_OK;
    }

    virtual int startNewGame(int player=White)
    {
      if( player == White )
      {
        m_colorPlayer = White;
        m_colorEngine = Black;
      }
      else
      {
        m_colorPlayer = Black;
        m_colorEngine = White;
      }

      m_bIsPlaying  = true;
      m_colorTurn   = White;
      m_nMove       = 1;

      return CE_OK;
    }

    virtual int endGame(ChessResult reason)
    {
      if( isPlayingAGame() )
      {
        m_bIsPlaying  = false;
        m_eEoGReason  = reason;
      }
    }

    virtual int makePlayersMove(Move &move)
    {
      return makeAMove(m_colorPlayer, move);
    }

    virtual int makeAMove(ChessColor colorMove, Move &move)
    {
      return -CE_ECODE_NO_EXEC;
    }

    virtual int getEnginesMove(Move &move, bool bAuto=false)
    {
      return -CE_ECODE_NO_EXEC;
    }

    virtual int resign()
    {
      return -CE_ECODE_NO_EXEC;
    }

    virtual int getCastlingOptions(std::string &strWhiteCastling,
                                   std::string &strBlackCastling)
    {
      return -CE_ECODE_NO_EXEC;
    }

    ChessColor getPlayersColor() const
    {
      return m_colorPlayer;
    }

    ChessColor getEnginesColor() const
    {
      return m_colorEngine;
    }

    virtual int whoseTurn() const
    {
      return m_colorTurn;
    }

    virtual bool isPlayingAGame() const
    {
      return m_bIsPlaying;
    }

    //..........................................................................
    // Low-Level I/O Interface
    //..........................................................................
    
    virtual int readline(std::string &strLine, uint_t msec=100)
    {
      return -CE_ECODE_NO_SUPP;
    }
  
    virtual int readline(char buf[], size_t sizeBuf, uint_t msec=100)
    {
      return -CE_ECODE_NO_SUPP;
    }
  
    virtual int writeline(const std::string &strLine)
    {
      return -CE_ECODE_NO_SUPP;
    }
  
    virtual int writeline(const char buf[], size_t sizeBuf)
    {
      return -CE_ECODE_NO_SUPP;
    }
  
    virtual void flushInput() { }
  
    virtual bool isConnected()
    {
      return m_bIsConn;
    }

  protected:
    std::string     m_strVersion;   ///< backend engine version string
    bool            m_bIsConn;      ///< [not] connected to backend engine
    bool            m_bIsPlaying;   ///< [no] active game in play
    float           m_fDifficulty;  ///< game engine difficulty [1,10]
    ChessColor      m_colorPlayer;  ///< color of player
    ChessColor      m_colorEngine;  ///< color of backend engine
    ChessColor      m_colorTurn;    ///< color to play
    int             m_nMove;        ///< number of full moves (2 plies/move)
    ChessResult     m_eEoGReason;   ///< reason for ending game

    virtual void alternateTurns(ChessColor colorLast)
    {
      if( isPlayingAGame() )
      {
        if( colorLast == Black )
        {
          ++m_nMove;
        }
        m_colorTurn = colorLast == White? Black: White;
      }
    }
  };

} // chess_engine

#endif // _CHESS_ENGINE_BE_H
