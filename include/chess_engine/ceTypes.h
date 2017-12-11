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

#ifndef _CE_TYPES_H
#define _CE_TYPES_H

#include <stddef.h>

#include <iostream>
#include <vector>

namespace chess_engine
{
  // --------------------------------------------------------------------------
  // Enumerated Types and Values
  // --------------------------------------------------------------------------

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
   * \brief Chess player, piece, and move color.
   */
  enum ChessColor
  {
    NoColor     = ChessValNone, ///< no color
    White       = 'w',          ///< white
    Black       = 'b',          ///< black
    UndefColor  = ChessValUndef ///< undefined/unknown color
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
    Busy          = 'b',          ///< busy making a move
    InPlay        = '>',          ///< game currently in play
    Checkmate     = '#',          ///< checkmate
    Draw          = 'd',          ///< game is a draw
    Resign        = 'r',          ///< player or engine resigned
    Disqualified  = 'k',          ///< player is disqualified
    Aborted       = '~',          ///< game aborted
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

  /*!
   * \brief Chess board direction.
   */
  enum ChessBoardDir
  {
    DirNone       = 0,    ///< no direction; delta file,rank = (0, 0)
    DirUp         = 1,    ///< up;           delta file,rank = (0, 1)
    DirUpperLeft  = 2,    ///< upper left;   delta file,rank = (-1, 1)
    DirLeft       = 3,    ///< left;         delta file,rank = (-1, 0)
    DirLowerLeft  = 4,    ///< lower left;   delta file,rank = (-1, -1)
    DirDown       = 5,    ///< down;         delta file,rank = (0, -1)
    DirLowerRight = 6,    ///< lower right;  delta file,rank = (1, -1)
    DirRight      = 7,    ///< right;        delta file,rank = (1, 0)
    DirUpperRight = 8,    ///< upper right;  delta file,rank = (1, 1)

    DirNumOf              ///< number of board directions   
  };

  const size_t EndOfBoard = 8; ///< til the end of the board

  /*!
   * \brief Chess player id type and reserved values.
   */
  typedef size_t      ChessPlayerId;  ///< player id

  const ChessPlayerId NoPlayerId        = 0; ///< "no player id" id
  const ChessPlayerId AnonPlayerId      = 1; ///< anonymous player id
  const ChessPlayerId AnonWhitePlayerId = 2; ///< anonymous white player id
  const ChessPlayerId AnonBlackPlayerId = 3; ///< anonomous black player id

  /*!
   * \brief Chess player types.
   */
  enum ChessPlayerType
  {
    PlayerTypeAnon    = ChessValNone, ///< anonymous, unknown player type
    PlayerTypeHuman   = 'H',          ///< human player
    PlayerTypeRobot   = 'R',          ///< autonomous robot player
    PlayerTypeSwAgent = 'A'           ///< software agent type
  };


  // --------------------------------------------------------------------------
  // ChessPos Structure
  // --------------------------------------------------------------------------

  /*!
   * \brief Chess board position. (Definition found in ceCore.cpp.)
   */
  struct ChessPos
  {
    ChessFile m_file;   ///< file (column)
    ChessRank m_rank;   ///< rank (row)

    /*!
     * \brief Default constructor.
     *
     * Position is initialized to 'no position'.
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
    ChessPos(const int file, const int rank);

    /*!
     * \brief Parse constructor.
     *
     * \param filerank  Chess square file-rank string ['a' - 'h']['1' - '8'].
     */
    ChessPos(const std::string &filerank);

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
     * \brief Test if the position is fully specified.
     *
     * \return Returns true or false.
     */
    bool isSpecified() const
    {
      return (m_file != NoFile) && (m_rank != NoRank);
    }

    /*!
     * \brief Test if file is specified.
     *
     * \return Returns true or false.
     */
    bool hasFile() const
    {
      return (m_file != NoFile);
    }

    /*!
     * \brief Test if rank is specified.
     *
     * \return Returns true or false.
     */
    bool hasRank() const
    {
      return (m_rank != NoRank);
    }

    /*!
     * \brief Test if position specifies a location on a standard chess board.
     *
     * \return Returns true or false.
     */
    bool isOnChessBoard() const;

    /*!
     * \brief Equality operator.
     *
     * Test if both file and rank equal.
     *
     * \param b   Position to compare with.
     *
     * \return Returns true if equal, false otherwise.
     */
    bool operator==(const ChessPos &b) const
    {
      return (m_file == b.m_file) && (m_rank == b.m_rank);
    }

    /*!
     * \brief Equality operator.
     *
     * Test if files equal.
     *
     * \param b   Chess file.
     *
     * \return Returns true if equal, false otherwise.
     */
    bool operator==(const ChessFile &b) const
    {
      return m_file == b;
    }

    /*!
     * \brief Equality operator.
     *
     * Test if ranks equal.
     *
     * \param b   Chess rank.
     *
     * \return Returns true if equal, false otherwise.
     */
    bool operator==(const ChessRank &b) const
    {
      return (m_rank == b);
    }

    /*!
     * \brief Inequality operator.
     *
     * Test if either the file or the rank do not equal.
     *
     * \param b   Position to compare with.
     *
     * \return Returns true if inequal, false otherwise.
     */
    bool operator!=(const ChessPos &b) const
    {
      return (m_file != b.m_file) || (m_rank != b.m_rank);
    }

    /*!
     * \brief Inequality operator.
     *
     * Test if files do not equal.
     *
     * \param b   Chess file.
     *
     * \return Returns true if inequal, false otherwise.
     */
    bool operator!=(const ChessFile &b) const
    {
      return m_file != b;
    }

    /*!
     * \brief Inequality operator.
     *
     * Test if ranks do not equal.
     *
     * \param b   Chess rank.
     *
     * \return Returns true if inequal, false otherwise.
     */
    bool operator!=(const ChessRank &b) const
    {
      return (m_rank != b);
    }

    /*!
     * \brief Output stream insertion operator.
     *
     * \param os    Output stream.
     * \param pos   Chess position to insert.
     *
     * \return Returns os.
     */
    friend std::ostream &operator<<(std::ostream &os, const ChessPos &pos);

  }; // struct ChessPos

  /*! list of positions type */
  typedef std::vector<ChessPos> list_of_pos;

  const ChessPos  NoPos;  ///< no position or move

  /*!
   * \brief Start of Game Positions
   * \{
   */
  // white
  const ChessPos  SoGPosWhiteQR(ChessFileA, ChessRank1);  ///< queen's rook
  const ChessPos  SoGPosWhiteQN(ChessFileB, ChessRank1);  ///< queen's knight
  const ChessPos  SoGPosWhiteQB(ChessFileC, ChessRank1);  ///< queen's bishop
  const ChessPos  SoGPosWhiteQ( ChessFileD, ChessRank1);  ///< queen
  const ChessPos  SoGPosWhiteK( ChessFileE, ChessRank1);  ///< king
  const ChessPos  SoGPosWhiteKB(ChessFileF, ChessRank1);  ///< king's bishop
  const ChessPos  SoGPosWhiteKN(ChessFileG, ChessRank1);  ///< king's knight
  const ChessPos  SoGPosWhiteKR(ChessFileH, ChessRank1);  ///< king's rook
  const ChessRank SoGRankWhiteP = ChessRank2;             ///< pawn's rank

  // black
  const ChessPos  SoGPosBlackQR(ChessFileA, ChessRank8);  ///< queen's rook
  const ChessPos  SoGPosBlackQN(ChessFileB, ChessRank8);  ///< queen's knight
  const ChessPos  SoGPosBlackQB(ChessFileC, ChessRank8);  ///< queen's bishop
  const ChessPos  SoGPosBlackQ( ChessFileD, ChessRank8);  ///< queen
  const ChessPos  SoGPosBlackK( ChessFileE, ChessRank8);  ///< king
  const ChessPos  SoGPosBlackKB(ChessFileF, ChessRank8);  ///< king's bishop
  const ChessPos  SoGPosBlackKN(ChessFileG, ChessRank8);  ///< king's knight
  const ChessPos  SoGPosBlackKR(ChessFileH, ChessRank8);  ///< king's rook
  const ChessRank SoGRankBlackP = ChessRank7;             ///< pawn's rank
  /*! \} */

  /*!
   * \brief Castling Positions
   * \{
   */
  // white castling source positions
  const ChessPos  CastlingPosWhiteKSrc(SoGPosWhiteK); ///< king source position
  const ChessPos  KSidePosWhiteRSrc(SoGPosWhiteKR);   ///< king's rook source
  const ChessPos  QSidePosWhiteRSrc(SoGPosWhiteQR);   ///< queen's rook source

  // white king side castling destination positions
  const ChessPos  KSidePosWhiteKDst(ChessFileG, ChessRank1);  // king dest
  const ChessPos  KSidePosWhiteRDst(ChessFileF, ChessRank1);  // rook dest

  // white queen side castling destination positions
  const ChessPos  QSidePosWhiteKDst(ChessFileC, ChessRank1);  // king dest
  const ChessPos  QSidePosWhiteRDst(ChessFileD, ChessRank1);  // rook dest

  // black
  const ChessPos  CastlingPosBlackKSrc(SoGPosBlackK); ///< king source position
  const ChessPos  KSidePosBlackRSrc(SoGPosBlackKR);   ///< king's rook source
  const ChessPos  QSidePosBlackRSrc(SoGPosBlackQR);   ///< queen's rook source

  // white king side castling destination positions
  const ChessPos  KSidePosBlackKDst(ChessFileG, ChessRank8);  // king dest
  const ChessPos  KSidePosBlackRDst(ChessFileF, ChessRank8);  // rook dest

  // white queen side castling destination positions
  const ChessPos  QSidePosBlackKDst(ChessFileC, ChessRank8);  // king dest
  const ChessPos  QSidePosBlackRDst(ChessFileD, ChessRank8);  // rook dest
  /*! \} */

  /*!
   * \brief Special Pawn Movement Positions
   * \{
   */
  // white pawn initial move
  const ChessRank PawnRankWhite2SqSrc = ChessRank2; ///< pawn 2 square src rank
  const ChessRank PawnRankWhite2SqDst = ChessRank4; ///< pawn 2 square dst rank

  // white pawn en passant move
  const ChessRank PawnRankWhiteEPSrc  = ChessRank5; ///< pawn's e.p src rank
  const ChessRank PawnRankWhiteEPDst  = ChessRank6; ///< pawn's e.p dst rank
  const ChessRank PawnRankWhiteEPCap  = ChessRank5; ///< pawn's e.p capture rank

  // white pawn promotion move
  const ChessRank PawnRankWhitePromo  = ChessRank8; ///< pawn's promotion rank

  // black pawn initial move
  const ChessRank PawnRankBlack2SqSrc = ChessRank7; ///< pawn 2 square src rank
  const ChessRank PawnRankBlack2SqDst = ChessRank5; ///< pawn 2 square dst rank

  // black pawn en passant move
  const ChessRank PawnRankBlackEPSrc  = ChessRank4; ///< pawn's e.p src rank
  const ChessRank PawnRankBlackEPDst  = ChessRank3; ///< pawn's e.p dst rank
  const ChessRank PawnRankBlackEPCap  = ChessRank4; ///< pawn's e.p capture rank

  // black pawn promotion move
  const ChessRank PawnRankBlackPromo  = ChessRank1; ///< pawn's promotion rank

  /*! \} */


  // --------------------------------------------------------------------------
  // ChessFqPeice Class
  // --------------------------------------------------------------------------

  /*!
   * \brief Chess fully qualified piece. (Implementation found in ceCore.cpp.)
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
     * Id format: color-bord-piece
     *
     * \par Examples:
     * Identifier  | Description
     * ----------  | -----------
     *  black-c7-pawn   | black pawn born at c7
     *  white-h2-pawn   | white pawn born at h2
     *  white-a1-rook   | white queen's rook
     *  white-e1-king   | white king
     *  black-d8-queen  | black queen
     *  black-c8-bishop | black queens's bishop
     *
     * \param pos     Chess piece birth origin position.
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
     * Id format: color-born-piece-instance
     *
     * \par Examples:
     * Identifier  | Description
     * ----------  | -----------
     *  white-c8-queen-1 | first promotion by white of a pawn to a queen
     *  white-h8-queen-2 | second promotion by white of a pawn to a queen
     *
     * \param pos         Chess piece birth origin position.
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

    /*!
     * \brief ChessFqPiece output stream operator.
     *
     * \param os      Output stream.
     * \param fqPiece Fully qualified chess piece to insert.
     *
     * \return Output stream.
     */
    friend std::ostream &operator<<(std::ostream       &os,
                                    const ChessFqPiece &fqPiece);

  }; // class ChessFqPiece


} // namespace chess_engine

#endif // _CE_TYPES_H
