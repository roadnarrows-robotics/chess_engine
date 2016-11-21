////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceError.cpp
//
/*! \file
 *
 * \brief Common chess engine return values and error codes.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

#include "boost/assign.hpp"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"

using namespace std;
using namespace boost::assign;

namespace chess_engine
{
  /*! error strings */
  static map<int, string> Errors = map_list_of
    (CE_OK,                       "")
    (CE_ECODE_GEN,                "Error")
    (CE_ECODE_SYS,                "System error")
    (CE_ECODE_NO_SUPP,            "Operationnot supported")
    (CE_ECODE_TIMEDOUT,           "Operation timed out")
    (CE_ECODE_NO_EXEC,            "Cannot execute")
    (CE_ECODE_BUSY,               "Resource busy")
    (CE_ECODE_CHESS_NO_GAME,      "No active chess game")
    (CE_ECODE_CHESS_BAD_MOVE,     "Invalid chess move")
    (CE_ECODE_CHESS_OUT_OF_TURN,  "Chess move out-of-turn")
    (CE_ECODE_CHESS_RSP,          "Algebraic Notation parse failed")
    (CE_ECODE_CHESS_PARSE,        "Unexpected backend chess engine response")
    (CE_ECODE_CHESS_SYNC,         "Chess game state out-of-sync")
    (CE_ECODE_CHESS_FATAL,       "Chess game in unrecoverable error condition");

  string strecode(int ecode)
  {
    map<int, string>::iterator pos;

    if( ecode < 0 )
    {
      ecode = -ecode;
    }

    if( (pos = Errors.find(ecode)) != Errors.end() )
    {
      return pos->second;
    }
    else
    {
      return "Unknown error code";
    }
  }

} // namespace chess_engine
