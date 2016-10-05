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
     * \brief Set chess square data.
     *
     * \param [in] eColor       Piece color [NoColor, White, Black]
     * \param [in] eType        Piece type [NoPiece, King, Queen, ..., Pawn]
     * \param [in] strPieceId   Fixed unique piece id.
     */
    void set(const ChessColor eColor,
             const ChessPiece eType,
             const std::string &strPieceId);

    /*!
     * \brief Get chess square data.
     *
     * \param [out] eColor      Piece color [NoColor, White, Black]
     * \param [out] eType       Piece type [NoPiece, King, Queen, ..., Pawn]
     * \param [out] strPieceId  Fixed unique piece id.
     */
    void get(ChessColor &eColor,
             ChessPiece &eType,
             std::string &strPieceId);

    /*!
     * \brief Copy this square's data to destination square.
     *
     * \param [out] dst   Destination square.
     */
    void copy(ChessSquare &dst);

    /*!
     * \brief Move this square's data to destination square.
     *
     * \param [out] dst   Destination square.
     */
    void move(ChessSquare &dst);

    /*!
     * \brief Remove this square's data.
     *
     * The square is without a chess piece. (isEmpty() is true).
     */
    void remove();

    /*!
     * \brief Test if square is empty (i.e. no piece).
     *
     * \return Returns true or false.
     */
    bool isEmpty();

    /*!
     * \brief Get this chess square's chess piece color.
     *
     * \return Piece color [NoColor, White, Black]
     */
    ChessColor getPieceColor();

    /*!
     * \brief Get this chess square's chess piece type.
     *
     * \return Piece type [NoPiece, King, Queen, ..., Pawn]
     */
    ChessPiece getPieceType();

    /*!
     * \brief Get this chess square's chess piece unique string id.
     *
     * \return Fixed unique piece id.
     */
    std::string getPieceId();

  protected:
    ChessColor  m_eColor;     ///< piece color [NoColor, White, Black]
    ChessPiece  m_eType;      ///< piece type [NoPiece, King, Queen, ..., Pawn]
    std::string m_strPieceId; ///< fixed unique piece id (eg. "white-b-pawn")
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
     * \brief Set up board to start a game.
     */
    void setupBoard();
  
    int sync(Move &move);

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

    BoardElem *getBoardElem(ChessFile file, ChessRank rank);

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
     * \param [out] row   Board row [0, 7]
     * \param [out] col   Board column [0, 7]
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    static int toRowCol(const ChessPos &pos, int &row, int &col);

    /*!
     * \brief Convert board array column to chess file.
     *
     * \param col  Board column [0, 7]
     *
     * \reture Chess file ['a', 'h'].
     */
    static ChessFile toFile(int col);

    /*!
     * \brief Convert board array row to chess rank.
     *
     * \note Rank 8 is row 7, ..., rank 1 is row 0.
     *
     * \param row   Board row [0, 7]
     *
     * \return Chess rank ['1', '8'].
     */
    static ChessRank toRank(int row);

    static ChessColor getSquareColor(int file, int rank);

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
