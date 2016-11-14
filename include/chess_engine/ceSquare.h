////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceSquare.h
//
/*! \file
 *
 * \brief The chess board square state interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows
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

#ifndef _CE_SQUARE_H
#define _CE_SQUARE_H

#include <iostream>
#include <string>
#include <vector>

#include "chess_engine/ceTypes.h"

/*!
 * \brief Package top-level namespace.
 */
namespace chess_engine
{
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Class ChessSquare
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Chess board square class.
   */
  class ChessSquare
  {
  public:
    /*!
     * \brief Default constructor.
     */
    ChessSquare();

    /*!
     * \brief Initialization constructor.
     *
     * \param [in] pos      Square position on board.
     * \param [in] fqPiece  Fully-qualified piece.
     */
    ChessSquare(const ChessPos &pos, const ChessFqPiece &fqPiece);

    /*!
     * \brief Initialization constructor.
     *
     * \param [in] pos          Square position on board.
     * \param [in] ePieceColor  Piece color [NoColor, White, Black].
     * \param [in] ePieceType   Piece type [NoPiece, King, Queen, ..., Pawn].
     * \param [in] strPieceId   Fixed unique piece id.
     */
    ChessSquare(const ChessPos    &pos,
                const ChessColor  ePieceColor,
                const ChessPiece  ePieceType,
                const std::string &strPieceId);

    /*!
     * \brief Destructor.
     */
    virtual ~ChessSquare() { };

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right-hand side source object.
     *
     * \return Dereferenced this
     */
    ChessSquare operator==(const ChessSquare &rhs);

    /*!
     * \brief Set the square position within the chess board.
     *
     * \param pos   Square's position.
     */
    void setPos(const ChessPos &pos);

    /*!
     * \brief Get the square's position on the chess board.
     *
     * \return Square's position.
     */
    ChessPos getPos() const;

    /*!
     * \brief Get this chess square's color.
     *
     * \return Square color [NoColor, White, Black]
     */
    ChessColor getColor() const;

    /*!
     * \brief Set the chess piece on the chess square.
     *
     * \param [in] fqPiece  Fully qualified piece.
     */
    void setPiece(const ChessFqPiece &fqPiece);

    /*!
     * \brief Set the chess piece on the chess square.
     *
     * \param [in] ePieceColor  Piece color [NoColor, White, Black].
     * \param [in] ePieceType   Piece type [NoPiece, King, Queen, ..., Pawn].
     * \param [in] strPieceId   Fixed unique piece id.
     */
    void setPiece(const ChessColor   ePieceColor,
                  const ChessPiece   ePieceType,
                  const std::string &strPieceId);

    /*!
     * \brief Get the chess piece on the chess square.
     *
     * \return Reference to fully-qualified piece.
     */
    ChessFqPiece &getPiece();

    /*!
     * \brief Get chess piece info, if any, on the chess square.
     *
     * \param [out] ePieceColor Piece color [NoColor, White, Black].
     * \param [out] ePieceType  Piece type [NoPiece, King, Queen, ..., Pawn].
     * \param [out] strPieceId  Fixed unique piece id.
     */
    void getPiece(ChessColor  &ePieceColor,
                  ChessPiece  &ePieceType,
                  std::string &strPieceId) const;

    /*!
     * \brief Get this chess square's chess piece color.
     *
     * \return Piece color [NoColor, White, Black].
     */
    ChessColor getPieceColor() const;

    /*!
     * \brief Get this chess square's chess piece type.
     *
     * \return Piece type [NoPiece, King, Queen, ..., Pawn].
     */
    ChessPiece getPieceType() const;

    /*!
     * \brief Get this chess square's chess piece unique string id.
     *
     * \return Fixed unique piece id.
     */
    std::string getPieceId() const;

    /*!
     * \brief Copy this square's piece to destination square.
     *
     * \param [out] dst   Destination square.
     */
    void copyPiece(ChessSquare &dst);

    /*!
     * \brief Move this square's piece to destination square.
     *
     * \param [out] dst   Destination square.
     */
    void movePiece(ChessSquare &dst);

    /*!
     * \brief Remove this square's piece from square.
     *
     * The square is without a chess piece. (isEmpty() is true).
     */
    void removePiece();

    /*!
     * \brief Test if square is empty (i.e. no piece).
     *
     * \return Returns true or false.
     */
    bool isEmpty() const;

    /*!
     * \brief Test if square is not associated with a board position.
     *
     * \return Returns true or false.
     */
    bool isOnChessBoard() const;


    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Static Member Functions
    //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Determine color of a chess square.
     *
     * \param pos   Chess square position.
     *
     * \return Returns White, Black, NoColor
     */
    static ChessColor colorOfSquare(const ChessPos &pos);

    /*!
     * \brief Determine color of a chess square.
     *
     * \param file    Chess square file (column).
     * \param rank    Chess square rank (row).
     *
     * \return Returns White, Black, NoColor
     */
    static ChessColor colorOfSquare(int file, int rank);

  protected:
    ChessPos      m_pos;          ///< square location on board (file, rank)
    ChessColor    m_eColor;       ///< square color [NoColor, White, Black]
    ChessFqPiece  m_fqPiece;      ///< fully qualified chess piece, if any
  };

} // chess_engine

#endif // _CE_SQUARE_H
