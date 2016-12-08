////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceUtils.h
//
/*! \file
 *
 * \brief Common chess utilities.
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

#ifndef _CE_UTILS_H
#define _CE_UTILS_H

#include <string>

#include <ros/console.h>

#include "chess_engine/ceTypes.h"


namespace chess_engine
{
  /*!
   * \brief Get your opponent's color.
   *
   * \param color Your color.
   *
   * \return Opponent's color.
   */
  inline ChessColor opponent(ChessColor color)
  {
    return color == White? Black: White;
  }

  /*!
   * \brief Get the name of the chess color.
   *
   * \param color Chess color enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfColor(ChessColor color);

  /*!
   * \brief Get the name of the chess piece.
   *
   * \param color Chess piece enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfPiece(ChessPiece piece);

  /*!
   * \brief Get the chess figurine unicode string.
   *
   * \param color Chess color enumeration.
   * \param color Chess piece enumeration.
   *
   * \return String.
   */
  extern const std::string figurineOfPiece(ChessColor color, ChessPiece piece);

  /*!
   * \brief Get the name of the chess castling move.
   *
   * \param color Chess castling side enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfCastling(ChessCastling side);

  /*!
   * \brief Get the name of the chess play/game action result.
   *
   * \param color Chess result enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfResult(ChessResult result);

  /*!
   * \brief Get the name of check modifier.
   *
   * \param checkmod  Chess check modifier enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfCheckMod(ChessCheckMod checkmod);

  /*!
   * \brief Get the name of the chess algebra notation.
   *
   * \param algebra   Chess algebra notation enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfAlgebra(ChessAlgebra algebra);
  
  /*!
   * \brief Get boolean's value string equivalent.
   *
   * \param v   Boolean vale.
   *
   * \return String.
   */
  inline std::string nameOfBool(bool v)
  {
    return v? "true": "false";
  }

  /*!
   * \brief Convert function return code to move result enum.
   *
   * \param rc    Return code.
   *
   * \return Result enum.
   */
  ChessResult rcToMoveResult(int rc);

} // namespace chess_engine


#endif // _CE_UTILS_H
