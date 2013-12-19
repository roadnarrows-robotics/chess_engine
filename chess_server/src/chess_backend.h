
////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_backend.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The backend engine base class.
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

#ifndef _CHESS_BACKEND_H
#define _CHESS_BACKEND_H

#include <sys/types.h>

#include "chess.h"
#include "chess_server.h"

namespace chess_engine
{
  class ChessBackend
  {
  public:
    ChessBackend() : m_strVersion("?")
    {
      m_fDifficulty = 2.0; 
      b_bInGame     = false;
      m_colorPlayer = NoColor;
      m_colorEngine = NoColor;
      m_colorTurn   = NoColor;
      m_nMoveNum    = 0;
    }
  
    virtual ~ChessBackend() { }
  
    virtual int openConnection()
    {
      return -CE_ECODE_SUPP;
    }
  
    virtual int closeConnection()
    {
      return -CE_ECODE_SUPP;
    }
  
    virtual int readline(std::string &strLine)
    {
      return -CE_ECODE_SUPP;
    }
  
    virtual int readline(char buf[], size_t sizeBuf)
    {
      return -CE_ECODE_SUPP;
    }
  
    virtual int writeline(const std::string &strLine)
    {
      return -CE_ECODE_SUPP;
    }
  
    virtual int writeline(const char buf[], size_t sizeBuf)
    {
      return -CE_ECODE_SUPP;
    }
  
    virtual void flushInput() { }
  
    virtual bool isOpen()
    {
      return false;
    }

    virtual std::string engineGetVersion()
    {
      return m_strVersion;
    }

    virtual void engineSetGameDifficulty(float fDifficulty)
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
    }

    virtual int engineStartNewGame(int player=White)
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
      m_colorTurn   = White;
      m_nMoveNum    = 0;
      b_bInGame     = true;

      return CE_OK;
    }

    virtual int engineMakeAMove(const ChessSquare &squareFrom,
                                const ChessSquare &quareTo)
    {
      return -CE_ECODE_NO_EXEC;
    }

    virtual int resign()
    {
      return -CE_ECODE_NO_EXEC;
    }

    virtual int engineGetEnginesMove()
    {
    }

    virtual int getWhoseTurn()
    {
      return m_colorTurn;
    }

    virtual int getWhoseTurn()
    {
      return m_colorTurn;
    }

    virtual int engineGetGameState()
    {
    }

  protected:
    std::string m_strVersion;   ///< backend engine version string
    float       m_fDifficulty;  ///< game engine difficulty [1,10]
    bool        b_bInGame;      ///< [not] currently playing a game
    int         m_colorPlayer;  ///< color of player
    int         m_colorEngine;  ///< color of backend engine
    int         m_colorTurn;    ///< color to play
    int         m_nMoveNum;     ///< number of full moves (2 plies/move)
  };

} // chess_engine

#endif // _CHESS_BACKEND_H
