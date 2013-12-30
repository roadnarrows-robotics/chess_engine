////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_engine_gnu.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The GNU chess gnuchess backend engine interface.
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

#ifndef _CHESS_ENGINE_GNU_H
#define _CHESS_ENGINE_GNU_H

#include <sys/types.h>

#include <string>
#include <vector>

#include <boost/regex.hpp>

#include "rnr/rnrconfig.h"

#include "chess.h"
#include "chess_move.h"
#include "chess_engine.h"

namespace chess_engine
{
  class ChessEngineGnu : public ChessEngine
  {
  public:
    static const int MaxLineSize = 80;

    ChessEngineGnu();
  
    virtual ~ChessEngineGnu();
  

    //..........................................................................
    // High-Level Interface
    //..........................................................................
  
    virtual int openConnection(const std::string &strChessApp="gnuchess");
  
    virtual int closeConnection();

    virtual int setGameDifficulty(float fDifficulty);

    virtual int startNewGame(int colorPlayer=White);

    virtual int makePlayersMove(ChessMove &move);

    virtual int getEnginesMove(ChessMove &move, bool bAuto=false);

    virtual int resign();


    //..........................................................................
    // Low-Level I/O Interface
    //..........................................................................
  
    virtual int readline(std::string &strLine, uint_t msec=100);

    virtual int readline(char buf[], size_t sizeBuf, uint_t msec=100);

    virtual int writeline(const std::string &strLine)
    {
      return writeline(strLine.c_str(), strLine.size());
    }
  
    virtual int writeline(const char buf[], size_t len);

    virtual void flushInput();
  
    virtual bool isOpen()
    {
      return m_pidChild > 0;
    }

  protected:
    // configuration and i/o operation
    std::string m_strChessApp;      ///< name of chess application
    int         m_pipeToChess[2];   ///< pipe to chess engine
    int         m_pipeFromChess[2]; ///< pipe from chess engine
    pid_t       m_pidChild;         ///< fork-exec'd engine process id
    int         m_nDepth;           ///< engine search ply depth (half-moves)

    // version specific configuration
    bool        m_bHasRandom;       ///< has [no] random start support
    bool        m_bHasTimeLimits;   ///< has [no] annoying TimeLimit strings

    // input lines and parsing
    std::string m_strLastLine;      ///< last unmatched line read from engine
    int         m_nNewMove;         ///< read new move number
    std::string m_strNewSAN;        ///< read/target new SAN
    std::string m_strNewAN;         ///< read new AN
    ChessResult m_eNewResult;       ///< read engine result (resign, etc)

    inline bool isVer5() { return m_strVersion[0] == '5'; }
    inline bool isVer6() { return m_strVersion[0] == '6'; }

    //
    // Initialization and tear down.
    //
    virtual void configure();
    virtual void killApp();

    //
    // Commands
    //
    virtual int cmdVersion();
    virtual int cmdXboard();
    virtual int cmdNew();
    virtual int cmdDepth(int nDepth);
    virtual int cmdRandom();
    virtual int cmdGo(bool bIsFirstGo=false);
    virtual int cmdMove(const std::string &strSAN);
    virtual int cmdShowGame(int nMove, ChessColor color);

    //
    // Responses
    //
    virtual int rspEnginesMove();

    virtual int rspFirstLine(boost::regex             &reRsp,
                             std::vector<std::string> &matches,
                             uint_t                   msec=100,
                             bool                     bLower=true);

    virtual int rspNextLine(boost::regex             &reRsp,
                            std::vector<std::string> &matches,
                            bool                      bLower=true);

    virtual int match(const std::string        &str,
                      boost::regex             &re,
                      std::vector<std::string> &matches);

    virtual void clearParseVars()
    {
      m_nNewMove = 0;
      m_strNewSAN.clear();
      m_strNewAN.clear();
      m_eNewResult = NoResult;
    }
  };

} // chess_engine

#endif // _CHESS_ENGINE_GNU_H
