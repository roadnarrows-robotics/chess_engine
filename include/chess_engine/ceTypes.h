////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceTypes.h
//
/*! \file
 *
 * \brief Chess engine basic core types.
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

#ifndef _CE_TYPES_H
#define _CE_TYPES_H

#include <iostream>
#include <vector>

namespace chess_engine
{

  const char ChessValNone  = '0';   ///< common 'no value' value
  const char ChessValUndef = '?';   ///< common undefined/unknown value

  /*!
   * \brief Chess board file (column).
   */
  enum ChessFile
  {
    NoFile      = ChessValNone, ///< no file
    ChessFileA  = 'a',          ///< file a, column 0
    ChessFileB  = 'b',          ///< file b, column 1
    ChessFileC  = 'c',          ///< file c, column 3
    ChessFileD  = 'd',          ///< file d, column 3
    ChessFileE  = 'e',          ///< file e, column 4
    ChessFileF  = 'f',          ///< file f, column 5
    ChessFileG  = 'g',          ///< file g, column 6
    ChessFileH  = 'h'           ///< file h, column 7
  };

  /*!
   * \brief Chess board rank (row).
   *
   * Rank numbering is always relative to White starting from '1' to '8'.
   */
  enum ChessRank
  {
    NoRank      = ChessValNone, ///< no rank
    ChessRank1  = '1',          ///< rank 1, row 0
    ChessRank2  = '2',          ///< rank 2, row 1
    ChessRank3  = '3',          ///< rank 3, row 2
    ChessRank4  = '4',          ///< rank 4, row 3
    ChessRank5  = '5',          ///< rank 5, row 4
    ChessRank6  = '6',          ///< rank 6, row 5
    ChessRank7  = '7',          ///< rank 7, row 6
    ChessRank8  = '8'           ///< rank 8, row 7
  };

  const int NumOfFiles = 8; ///< number of standard chess board files
  const int NumOfRanks = 8; ///< number of standard chess board ranks

  /*!
   * \brief Chess board position. (Definition found in ceCore.cpp.)
   */
  struct ChessPos
  {
    ChessFile m_file;   ///< file (column)
    ChessRank m_rank;   ///< rank (row)

    /*!
     * \brief Default constructor.
     */
    ChessPos();

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    ChessPos(const ChessPos &src);

    /*!
     * \brief Initialization constructor.
     *
     * \param file    Chess file ['a' - 'h']
     * \param rank    Chess rank ['1' - '8']
     */
    ChessPos(const int &file, const int &rank);

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    ChessPos operator=(const ChessPos &rhs);

    /*!
     * \brief Clear values.
     */
    void clear();

    /*!
     * \brief Test if position specifies a location on a standard chess board.
     *
     * \return Returns true or false.
     */
    bool isOnChessBoard() const;

    // Friends
    friend std::ostream &operator<<(std::ostream &os, const ChessPos &pos);
  };

  const ChessPos NoPos;  ///< no position or move

  /*! list of positions type */
  typedef std::vector<ChessPos> list_of_pos;

  /*!
   * \brief Chess player, piece, and move color.
   */
  enum ChessColor
  {
    NoColor = ChessValNone, ///< no color
    White   = 'w',          ///< white
    Black   = 'b'           ///< black
  };

  /*!
   * \brief Chess piece type.
   */
  enum ChessPiece
  {
    NoPiece     = ChessValNone, ///< no piece
    King        = 'K',          ///< king
    Queen       = 'Q',          ///< queen
    Rook        = 'R',          ///< rook or castle
    Bishop      = 'B',          ///< bishop
    Knight      = 'N',          ///< knight
    Pawn        = 'P',          ///< pawn
    UndefPiece  = ChessValUndef ///< undefined/unknown piece
  };

  /*!
   * \brief Chess fully qualified piece. (Definition found in ceCore.cpp.)
   *
   * A fully-qualified piece specifies its color, type, and its game unique
   * identifier.
   */
  class ChessFqPiece
  {
  public:
    ChessColor  m_ePieceColor;  ///< piece color [NoColor, White, Black]
    ChessPiece  m_ePieceType;   ///< piece type [NoPiece, King, ..., Pawn]
    std::string m_strPieceId;   ///< piece unique id (eg. "white-b2-pawn")

    /*!
     * \brief Default constructor.
     */
    ChessFqPiece();

    /*!
     * \brief Destructor.
     */
    virtual ~ChessFqPiece();

    /*!
     * \brief Copy constructor.
     */
    ChessFqPiece(const ChessFqPiece &src);

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    ChessFqPiece operator=(const ChessFqPiece &FqPiece);

    /*!
     * \brief Set data.
     *
     * \param m_ePieceColor   Chess piece color [NoColor, White, Black]
     * \param m_ePieceType    Chess piece type [NoPiece, King, ..., Pawn]
     * \param m_strPieceId    Chess piece unique id (eg. "black-d1-queen")
     */
    void set(const ChessColor  ePieceColor,
             const ChessPiece  ePieceType, 
             const std::string &strPieceId);

    /*!
     * \brief Clear values.
     */
    void clear();

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Static Member Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Make game unique piece identification string.
     *
     * The id may be used by external operations to follow, draw, render, etc.
     *
     * Id format: color-origin-piece
     *
     * \par Examples:
     * Identifier  | Description
     * ----------  | -----------
     *  Black-c7-Pawn   | black pawn with origin at c7.
     *  White-h2-Pawn   | white pawn with origin at h2.
     *  White-a1-Rook   | white queen's rook
     *  White-e1-King   | white king
     *  Black-d8-Queen  | black queen
     *  Black-c8-Bishop | black queens's bishop
     *
     * \param pos     Chess piece origin position.
     * \param eColor  Piece color.
     * \param ePiece  Piece type.
     *
     * \return String id.
     */
    static std::string makePieceId(const ChessPos &pos,
                                  ChessColor      eColor,
                                  ChessPiece      ePiece);

    /*!
     * \brief Make game unique piece identification string.
     *
     * \ref See makePieceId
     *
     * \param file    Chess file ['a', 'h'].
     * \param rank    Chess rank ['1', '8'].
     * \param eColor  Piece color.
     * \param ePiece  Piece type.
     *
     * \return String id.
     */
    static std::string makePieceId(int file, int rank,
                                   ChessColor eColor,
                                   ChessPiece ePiece);

    /*!
     * \brief Make game unique piece identification string with instance number.
     *
     * This version of unique id is needed for pawn promotions.
     * \ref See makePieceId
     *
     * Id format: color-origin-piece-instance
     *
     * \par Examples:
     * Identifier  | Description
     * ----------  | -----------
     *  White-c8-Queen-1 | first promotion by white of a pawn to a queen.
     *  White-h8-Queen-2 | second promotion by white of a pawn to a queen.
     *
     * \param pos         Chess piece origin position.
     * \param eColor      Piece color.
     * \param ePiece      Piece type.
     * \param nInstance   Instance number.
     *
     * \return String id.
     */
    static std::string makePieceId(const ChessPos &pos,
                                  ChessColor      eColor,
                                  ChessPiece      ePiece,
                                  int             nInstance);

    // Friends
    friend std::ostream &operator<<(std::ostream &os,
                                    const ChessFqPiece &fqPiece);
  };

  /*!
   * \brief Chess castling.
   */
  enum ChessCastling
  {
    NoCastling  = ChessValNone, ///< no castling
    KingSide    = 'K',          ///< king side castling
    QueenSide   = 'Q'           ///< queen side castling
  };

  /*! list of castling options available */
  typedef std::vector<ChessCastling> list_of_castling;

  /*!
   * \brief Chess move modifier.
   */
  enum ChessCheckMod
  {
    NoCheckMod      = ChessValNone, ///< no check modifier
    ModCheck        = '+',          ///< check
    ModDoubleCheck  = '2',          ///< double check
    ModCheckmate    = '#'           ///< checkmate
  };

  /*!
   * \brief Chess action result.
   */
  enum ChessResult
  {
    NoResult      = ChessValNone, ///< no result
    Ok            = 'y',          ///< good move
    BadMove       = 'n',          ///< invalid move attempt
    OutOfTurn     = '?',          ///< out-of-turn move attempt
    Checkmate     = '#',          ///< checkmate
    Draw          = 'd',          ///< game is a draw
    Resign        = 'r',          ///< player or engine resigned
    Disqualified  = 'k',          ///< player is disqualified
    NoGame        = '$',          ///< no active game
    GameFatal     = '!'           ///< current game has unrecoverable errors
  };

  /*!
   * \brief Chess Algebra Notations.
   */
  enum ChessAlgebra
  {
    UnknownAN = 0,      ///< unknown algebra notation
    CAN       = 1,      ///< coordinate algebra notation
    SAN       = 2       ///< standard algebra notation
  };


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Friends
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief ChessPos output stream operator.
   *
   * \param os      Output stream.
   * \param fqPiece Chess position.
   *
   * \return Output stream.
   */
  extern std::ostream &operator<<(std::ostream &os, const ChessPos &pos);

  /*!
   * \brief ChessFqPiece output stream operator.
   *
   * \param os      Output stream.
   * \param fqPiece Fully qualified chess piece.
   *
   * \return Output stream.
   */
  extern std::ostream &operator<<(std::ostream       &os,
                                  const ChessFqPiece &fqPiece);

} // namespace chess_engine

#endif // _CE_TYPES_H
