////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_backend_gnu.h
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

#ifndef _CHESS_BACKEND_GNU_H
#define _CHESS_BACKEND_GNU_H

#include <sys/types.h>

#include <string>

#include "chess.h"
#include "chess_backend.h"

namespace chess_engine
{

  class ChessBackendGnu : public ChessBackend
  {
  public:
    ChessBackendGnu();
  
    virtual ~ChessBackendGnu();
  
    virtual int openConnection(const std::string &strChessApp="gnuchess");
  
    virtual int closeConnection();
  
    virtual int readline(std::string &strLine)
    {
      char  buf[80];
      int   n;

      if( (n = readline(buf, sizeof(buf))) >= 0 )
      {
        strLine = buf;
      }
      return n;
    }

    virtual int readline(char buf[], size_t sizeBuf);

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
    std::string m_strChessApp;
    int         m_pipeToChess[2];
    int         m_pipeFromChess[2];
    pid_t       m_pidChild;

    virtual void configure();
  
    virtual void killApp();
  };

} // chess_engine

#endif // _CHESS_BACKEND_GNU_H
