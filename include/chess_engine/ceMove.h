////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceMove.h
//
/*! \file
 *
 * \brief Chess move class interface.
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

#ifndef _CE_MOVE_H
#define _CE_MOVE_H

#include <string>
#include <iostream>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceUtils.h"


namespace chess_engine
{
  /*!
   * \brief ChessMove Class.
   *
   * Each move or action during play if fully captured by instances of this
   * class. The hope is that a robotic entity can play chess without having
   * any deep knowledge of chess nor having to keep extensive game state
   * information.
   */
  class ChessMove
  {
  public:
    int           m_nMoveNum;       ///< move number (2 plies/move)
    ChessColor    m_ePlayer;        ///< player (and piece) color
    std::string   m_strSAN;         ///< standard algebraic notation of move

    ChessPiece    m_ePieceMoved;    ///< moved piece
    ChessPos      m_posSrc;         ///< moved piece source chess square
    ChessPos      m_posDst;         ///< moved piece destination chess square

    ChessPiece    m_ePieceCaptured; ///< captured piece, if any
    ChessPiece    m_ePiecePromoted; ///< promoted piece, if any
    bool          m_bIsEnPassant;   ///< is [not] an en passant move
    ChessCastling m_eCastling;      ///< castling side, if any

    ChessCheckMod m_eCheck;         ///< opponent [not] placed in check

    ChessResult   m_eResult;        ///< result of move

    /*!
     * \brief Default constructor.
     */
    ChessMove();

    /*!
     * \brief Copy constructor.
     *
     * \param src Source object.
     */
    ChessMove(const ChessMove &src);

    /*!
     * \brief Destructor.
     */
    virtual ~ChessMove();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object
     *
     * \return This.
     */
    ChessMove operator=(const ChessMove &rhs);
  
    /*!
     * \brief Convert move state to Coordinate Algebraic Notation string.
     *
     * \return String.
     */
    std::string CAN();

    /*!
     * \brief Convert move state to Standard Algebraic Notation string.
     *
     * \return String.
     */
    std::string SAN();

    /*!
     * \brief Clear move.
     */
    void clear();

    /*!
     * \brief Copy move object.
     *
     * \param src Source object.
     */
    void copy(const ChessMove &src);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Static Member Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get en passant move's captured opponent pawn position.
     *
     * \param ePlayer           En passant player.
     * \param posDst            Destination chess position of moved pawn.
     * \param [out] posCapture  Position of captured pawn.
     */
    static void getEnPassantCapturedPawnPos(const ChessColor ePlayer,
                                            const ChessPos   &posDst,
                                            ChessPos         &posCapture);

    /*!
     * \brief Get the castling king's source and destination positions.
     *
     * \param ePlayer           Castling player.
     * \param eCastling         Castling side.
     * \param [out] posDst      Source chess position of king.
     * \param [out] posDst      Destination chess position of king.
     */
    static void getCastlingKingMove(const ChessColor     ePlayer,
                                    const ChessCastling  eCastling,
                                    ChessPos             &posSrc,
                                    ChessPos             &posDst);

    /*!
     * \brief Get the castling king's rook source and destination positions.
     *
     * \param ePlayer           Castling player.
     * \param eCastling         Castling side.
     * \param [out] posDst      Source chess position of rook.
     * \param [out] posDst      Destination chess position of rook.
     */
    static void getCastlingRookMove(const ChessColor     ePlayer,
                                    const ChessCastling  eCastling,
                                    ChessPos             &posSrc,
                                    ChessPos             &posDst);


    /*!
     * \brief Convert move state to Coordinate Algebraic Notation string.
     *
     * \param move  Chess move.
     *
     * \return String.
     */
    static std::string CAN(ChessMove &move);

    /*!
     * \brief Convert to Coordinate Algebraic Notation string.
     *
     * \param posSrc          Source square position.
     * \param posDst          Deistinate square position.
     * \param ePiecePromoted  Promoted piece, if any.
     *
     * \return String.
     */
    static std::string CAN(const ChessPos   &posSrc,
                           const ChessPos   &posDst,
                           const ChessPiece ePiecePromoted = NoPiece);

    /*!
     * \brief Convert move state to Standard Algebraic Notation string.
     *
     * \param move  Chess move.
     *
     * \return String.
     */
    static std::string SAN(ChessMove &move);
    
    /*!
     * \brief Parse Coordinate Algebraic Notation string to populate move
     * values.
     *
     * This parse variation uses regular expressions to determine the move.
     *
     * \note Conversion will only partially set the move state since
     * neither the board nor game context is included.
     *
     * \param strCAN      Coordinate Algebraic Notation string.
     * \param [out] move  Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    static int parseCAN(const std::string &strCAN, ChessMove &move);
    
    /*!
     * \brief Parse Standard Algebraic Notation string to populate move values.
     *
     * This parse variation uses regular expressions to determine the move.
     *
     * \note Conversion will only partially modify the state since
     * neither the board nor game context is included.
     *
     * \param strSAN      Standard Algebraic Notation string.
     * \param [out] move  Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    static int parseSAN(const std::string &strSAN, ChessMove &move);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Friends.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    friend std::ostream &operator<<(std::ostream &os, const ChessMove &move);
    
  protected:
  };

  /*!
   * \brief Stream move state to output stream.
   *
   * \param os    Output stream.
   * \param move  Chess move object
   *
   * \return Reference to output stream.
   */
  extern std::ostream &operator<<(std::ostream &os, const ChessMove &move);
} // namespace chess_engine


#endif // _CE_MOVE_H
