////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceError.h
//
/*! \file
 *
 * \brief Library common return values and error codes.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2017  RoadNarrows LLC
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

#ifndef _CE_ERROR_H
#define _CE_ERROR_H

#include "chess_engine/ceTypes.h"

namespace chess_engine
{

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup ce_h
   * \defgroup ce_ecodes  RoadNarrows Chess Engine Error Codes
   *
   * Chess engine package-wide error codes.
   *
   * \{
   */
  const int CE_OK                       =  0; ///< not an error, success

  const int CE_ECODE_GEN                =  1; ///< general error
  const int CE_ECODE_SYS                =  2; ///< system (errno) error
  const int CE_ECODE_NO_SUPP            =  3; ///< not supported
  const int CE_ECODE_TIMEDOUT           =  4; ///< operation timed out
  const int CE_ECODE_NO_EXEC            =  5; ///< cannot execute
  const int CE_ECODE_BUSY               =  6; ///< resource busy
  const int CE_ECODE_INTR               =  7; ///< execution interrupted
  const int CE_ECODE_BAD_VAL            =  8; ///< bad value
  const int CE_ECODE_CHESS_NO_GAME      =  9; ///< no active chess game
  const int CE_ECODE_CHESS_BAD_MOVE     = 10; ///< invalid chess move
  const int CE_ECODE_CHESS_OUT_OF_TURN  = 11; ///< out-of-turn chess move
  const int CE_ECODE_CHESS_RSP          = 12; ///< unexpected response
  const int CE_ECODE_CHESS_PARSE        = 13; ///< parse error
  const int CE_ECODE_CHESS_SYNC         = 14; ///< game out-of-sync
  const int CE_ECODE_CHESS_FATAL        = 15; ///< game in fatal condition
  /* \} */

  /*!
   * \brief Get error string associated with chess error code.
   *
   * \param ecode Error code.
   *
   * \return String.
   */
  extern std::string strecode(int ecode);

} // namespace chess_engine


#endif // _CE_ERROR_H
