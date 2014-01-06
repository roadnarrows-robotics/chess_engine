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
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief Library common return values and error codes.
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

#ifndef _CE_ERROR_H
#define _CE_ERROR_H

#include "chess_engine/ceChess.h"

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
  static const int CE_OK                      =  0; ///< not an error, success

  static const int CE_ECODE_GEN               =  1; ///< general error
  static const int CE_ECODE_SYS               =  2; ///< system (errno) error
  static const int CE_ECODE_NO_SUPP           =  3; ///< not supported
  static const int CE_ECODE_TIMEDOUT          =  4; ///< operation timed out
  static const int CE_ECODE_NO_EXEC           =  5; ///< cannot execute
  static const int CE_ECODE_CHESS_NO_GAME     =  6; ///< no active chess game
  static const int CE_ECODE_CHESS_MOVE        =  7; ///< invalid chess move
  static const int CE_ECODE_CHESS_OUT_OF_TURN =  8; ///< out-of-turn chess move
  static const int CE_ECODE_CHESS_RSP         =  9; ///< unexpected response
  static const int CE_ECODE_CHESS_SYNC        = 10; ///< game in fatal condition
  static const int CE_ECODE_CHESS_FATAL       = 11; ///< game in fatal condition
  /* \} */

  extern std::string strerror(int ecode);

} // namespace chess_engine


#endif // _CE_ERROR_H
