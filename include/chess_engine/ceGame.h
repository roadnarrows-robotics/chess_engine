////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceGame.h
//
/*! \file
 *
 * \brief The chess game state interface.
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

#ifndef _CE_GAME_H
#define _CE_GAME_H

#include <iostream>
#include <string>
#include <vector>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceUtils.h"

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
     * \param [in] pos          Square position on board.
     * \param [in] ePieceColor  Piece color [NoColor, White, Black].
     * \param [in] ePieceType   Piece type [NoPiece, King, Queen, ..., Pawn].
     * \param [in] strPieceId   Fixed unique piece id.
     */
    void ChessSquare(const ChessPos    &pos,
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
    ChessPos getPos();

    /*!
     * \brief Get this chess square's color.
     *
     * \return Square color [NoColor, White, Black]
     */
    ChessColor getColor();

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
     * \brief Get chess piece info, if any, on the chess square.
     *
     * \param [out] ePieceColor Piece color [NoColor, White, Black].
     * \param [out] ePieceType  Piece type [NoPiece, King, Queen, ..., Pawn].
     * \param [out] strPieceId  Fixed unique piece id.
     */
    void getPiece(ChessColor  &ePieceColor,
                  ChessPiece  &ePieceType,
                  std::string &strPieceId);

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
    bool isEmpty();

    /*!
     * \brief Test if square is not associated with a board position.
     *
     * \return Returns true or false.
     */
    bool isNotOnBoard();

    /*!
     * \brief Get this chess square's chess piece color.
     *
     * \return Piece color [NoColor, White, Black].
     */
    ChessColor getPieceColor();

    /*!
     * \brief Get this chess square's chess piece type.
     *
     * \return Piece type [NoPiece, King, Queen, ..., Pawn].
     */
    ChessPiece getPieceType();

    /*!
     * \brief Get this chess square's chess piece unique string id.
     *
     * \return Fixed unique piece id.
     */
    std::string getPieceId();

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
    ChessPos    m_pos;          ///< square location on board (file, rank)
    ChessColor  m_eColor;       ///< square color [NoColor, White, Black]
    ChessColor  m_ePieceColor;  ///< piece color [NoColor, White, Black]
    ChessPiece  m_ePieceType;   ///< piece type [NoPiece, King, ..., Pawn]
    std::string m_strPieceId;   ///< piece unique id (eg. "white-b-pawn")
  };


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Class ChessGame
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Chess game class.
   */
  class Game
  {
  public:
    /*!
     * \brief Default constructor.
     */
    Game();
  
    /*!
     * \brief Destructor.
     */
    virtual ~Game() { }
  
    /*!
     * \brief Set up for the start a (new) game.
     */
    void setupGame();
  
    /*!
     * \brief Make the chess move.
     *
     * \param move  Fully specified chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int makeTheMove(Move &move);

    bool isPlaying()
    {
      return m_bIsPlaying;
    }

    void stopPlaying(ChessResult reason, ChessColor winner=NoColor);

    int getNumOfMoves();

    int getNumOfPlies();

    ChessColor getWinner()
    {
      return m_winner;
    }

    ChessResult getEndOfGameReason()
    {
      return m_endReason;
    }

    std::vector<Move> &getGameHistory()
    {
      return m_history;
    }

    Move &operator[](int i)
    {
      return m_history[i];
    }

    /*!
     * \brief Get a reference to the board square.
     *
     * \param file  Chess board file.
     * \param rank  Chess board rank.
     *
     * \return If in range, returns the reference to the board's square.\n
     * If out-of-range, returns "no square" square (see ChessSquare.isNoSquare).
     */
    ChessSquare &getBoardSquare(const int file, const int rank);

    /*!
     * \brief Get a reference to the board square.
     *
     * \param pos   Chess board position.
     *
     * \return If in range, returns the reference to the board's square.\n
     * If out-of-range, returns "no square" square
     * (see ChessSquare::isNoSquare()).
     */
    ChessSquare &getBoardSquare(const ChessPos &pos);

    std::vector<ChessPiece> &getBoneYard(ChessColor color);

    /*!
     * \brief Convert chess rank to board array row.
     *
     * \note Rank 8 is row 7, ..., rank 1 is row 0.
     *
     * \param rank    Chess rank ['1', '8'].
     *
     * \return Board row [0, 7]
     */
    static int toRow(int rank);

    /*!
     * \brief Convert chess file to board array column.
     *
     * \param file    Chess file ['a', 'h'].
     *
     * \return Board column [0, 7]
     */
    static int toCol(int file);

    /*!
     * \brief Convert chess square coordinates to board array row and column.
     *
     * \param [in] pos    Board position (file,rank).
     * \param [out] row   Board row [0, 7].
     * \param [out] col   Board column [0, 7].
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    static int toRowCol(const ChessPos &pos, int &row, int &col);

    /*!
     * \brief Convert board array column to chess file.
     *
     * \param col  Board column [0, 7].
     *
     * \reture Chess file ['a', 'h'].
     */
    static ChessFile toFile(int col);

    /*!
     * \brief Convert board array row to chess rank.
     *
     * \note Rank 8 is row 7, ..., rank 1 is row 0.
     *
     * \param row   Board row [0, 7].
     *
     * \return Chess rank ['1', '8'].
     */
    static ChessRank toRank(int row);

    /*!
     * \brief Get the next file after the given file.
     *
     * \param file  Current chess board file.
     *
     * \returns Returns the next file. If no more files, then returns NoFile.
     */
    static ChessFile nextFile(int file);

    /*!
     * \brief Get the next rank after the given rank.
     *
     * \param rank  Current chess board rank.
     *
     * \returns Returns the next rank. If no more ranks, then returns NoRank.
     */
    static ChessRank nextRank(int rank);

    /*!
     * \brief Get the color of the square at the given board location.
     *
     * \param file    Chess file ['a', 'h'].
     * \param rank    Chess rank ['1', '8'].
     *
     * \return Returns NoColor, White or Black.
     */
    static ChessColor getSquareColor(int file, int rank);

    /*!
     * \brief Set unicode GUI state.
     *
     * \param on_off    State.
     */
    void setGuiState(bool on_off)
    {
      m_bGui = on_off;
    }

  protected:
    ChessSquare             m_board[NumOfRanks][NumOfFiles]; ///< board matrix
    bool                    m_bIsPlaying;     ///< is [not] playing a game
    ChessResult             m_endReason;      ///< end of game reason
    ChessColor              m_winner;         ///< end of game winner, if any
    std::vector<Move>       m_history;        ///< game history
    std::vector<ChessPiece> m_boneYardWhite;  ///< captured white pieces
    std::vector<ChessPiece> m_boneYardBlack;  ///< captured black pieces
    bool                    m_bGui;           ///< [not] a gui stream output

    /*!
     * \brief One-time board initialization.
     */
    void initBoard();
  
    /*!
     * \brief Remove all pieces from the board.
     */
    void clearBoard();
  
    /*!
     * \brief Set up board for the start a (new) game.
     */
    void setupBoard();
  
    /*!
     * \brief Set up one side of the board for the start a game.
     *
     * \param eColor Side's color.
     */
    void setupBoard(ChessColor eColor);
  
    /*!
     * \brief Make unique piece identification string.
     *
     * The id is used by external operations to follow, draw, render, etc.
     *
     * Id format: color[-modifier]-piece\n
     *
     * Examples:
     * Identifier  | Description
     * ----------  | -----------
     *  black-c-Pawn | black pawn starting on the 'c' file.
     *  white-h-Paw | white pawn starting on the 'h' file.
     *  white-Queen-Rook | white queen's rook
     *  white-King | white king
     *  black-King-Bishop | black king's bishop
     *
     * \param file    Chess file ['a', 'h'].
     * \param rank    Chess rank ['1', '8'].
     * \param eColor  Piece color.
     * \param ePiece  Piece type.
     *
     * \return String id.
     */
    std::string makePieceId(int file, int rank,
                            ChessColor eColor, ChessPiece ePiece);




    BoardElem *elem(const ChessPos &pos);

    void movePiece(const ChessPos &posFrom, const ChessPos &posTo);

    void movePiece(BoardElem *pSrc, BoardElem *pDst);

    void recordHistory(Move &move);

    void moveToBoneYard(BoardElem *pDeadPiece);

    friend std::ostream &operator<<(std::ostream &os, const Game &game);
  };

  extern std::ostream &operator<<(std::ostream &os, const Game &game);

} // chess_engine

#endif // _CE_GAME_H
