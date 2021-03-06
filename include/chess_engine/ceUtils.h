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
 * (C) 2013-2017  RoadNarrows
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
   * \brief Check if chess file is a valid file on the standard 8x8 board.
   *
   * \param file  Chess file to check.
   *
   * \return Returns true or false.
   */
  inline bool isValidFile(const int file)
  {
    return (file >= ChessFileA) && (file <= ChessFileH);
  }

  /*!
   * \brief Check if chess rank is a valid rank on the standard 8x8 board.
   *
   * \param rank  Chess rank to check.
   *
   * \return Returns true or false.
   */
  inline bool isValidRank(const int rank)
  {
    return (rank >= ChessRank1) && (rank <= ChessRank8);
  }

  /*!
   * \brief Calculate difference in file column positions.
   *
   * \param file1 Chess file minuend.
   * \param file2 Chess file subtrahend.
   *
   * \return file1 - file2
   */
  inline int diffFiles(ChessFile file1, ChessFile file2)
  {
    return (int)file1 - (int)file2;
  }

  /*!
   * \brief Calculate difference in rank row positions.
   *
   * \param rank1 Chess rank minuend.
   * \param rank2 Chess rank subtrahend.
   *
   * \return rank1 - rank2
   */
  inline int diffRanks(ChessRank rank1, ChessRank rank2)
  {
    return (int)rank1 - (int)rank2;
  }

  /*!
   * \brief Get the name of the chess color.
   *
   * \param color Chess color enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfColor(const ChessColor color);

  /*!
   * \brief Get the name of the chess piece.
   *
   * \param color Chess piece enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfPiece(const ChessPiece piece);

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
  extern const std::string nameOfCastling(const ChessCastling side);

  /*!
   * \brief Get the name of the chess play/game action result.
   *
   * \param color Chess result enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfResult(const ChessResult result);

  /*!
   * \brief Get the name of check modifier.
   *
   * \param checkmod  Chess check modifier enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfCheckMod(const ChessCheckMod checkmod);

  /*!
   * \brief Get the name of the chess algebra notation.
   *
   * \param algebra   Chess algebra notation enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfAlgebra(const ChessAlgebra algebra);
  
  /*!
   * \brief Get the name of the type of player.
   *
   * \param type  Player type enumeration.
   *
   * \return String.
   */
  extern const std::string nameOfPlayerType(const ChessPlayerType type);

  /*!
   * \brief Get boolean's value string equivalent.
   *
   * \param v   Boolean vale.
   *
   * \return String.
   */
  inline std::string nameOfBool(const bool v)
  {
    return v? "true": "false";
  }

  /*!
   * \brief Names of move categories.
   */
  const char* const NameOfMoveEnPassant     = "En Passant";
  const char* const NameOfMovePawnPromotion = "Pawn Promotion";
  const char* const NameOfMoveCastling      = "Castling";
  const char* const NameOfMoveMajor         = "Major Piece";
  const char* const NameOfMoveCoord         = "Coordinate";
  const char* const NameOfMoveCapture       = "capture";

  /*!
   * \brief Convert function return code to move result enum.
   *
   * \param rc    Return code.
   *
   * \return Result enum.
   */
  ChessResult rcToMoveResult(int rc);

  /*!
   * \brief Integer absolute value.
   *
   * \param a   Integer value.
   *
   * \return Returns |a|.
   */
  inline int iabs(const int a)
  {
    return a >= 0? a: -a;
  }
} // namespace chess_engine


#endif // _CE_UTILS_H
