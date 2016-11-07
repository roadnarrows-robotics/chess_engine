////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceBoard.h
//
/*! \file
 *
 * \brief The chess board state interface.
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

#ifndef _CE_BOARD_H
#define _CE_BOARD_H

#include <iostream>
#include <string>
#include <vector>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceSquare.h"
#include "chess_engine/ceMove.h"


/*!
 * \brief Package top-level namespace.
 */
namespace chess_engine
{
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Class ChessBoard
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Chess board class.
   */
  class ChessBoard
  {
  public:
    /*!
     * \brief Default constructor.
     */
    ChessBoard();
  
    /*!
     * \brief Destructor.
     */
    virtual ~ChessBoard() { }
  
    /*!
     * \brief Remove all pieces from the board.
     */
    void clearBoard();
  
    /*!
     * \brief Set up board for the start a (new) game.
     */
    void setupBoard();
  
    /*!
     * \brief Set a piece on the board.
     *
     * \param pos     Board position.
     * \param eColor  Piece color.
     * \param ePiece  Piece type.
     * \param strId   Piece unique id.
     */
    void setPiece(const ChessPos    &pos,
                  const ChessColor   eColor,
                  const ChessPiece   ePiece,
                  const std::string &strId);

    /*!
     * \brief Set a piece on the board.
     *
     * \param pos     Board position.
     * \param fqPiece Fully-qualified piece.
     */
    void setPiece(const ChessPos &pos, const ChessFqPiece &fqPiece);

    /*!
     * \brief Move a board piece.
     *
     * \param posSrc  Source board position.
     * \param posDst  Destination board position.
     */
    void movePiece(const ChessPos &posSrc, const ChessPos &posDst);

    /*!
     * \brief Remove a board piece.
     *
     * \param pos   Board position.
     */
    void removePiece(const ChessPos &pos);

    /*!
     * \brief Get a reference to the board square.
     *
     * \param file  Chess board file.
     * \param rank  Chess board rank.
     *
     * \return If in range, returns the reference to the board's square.\n
     * If out-of-range, returns "no square" square
     * (see ChessSquare::isOnChessBoard).
     */
    ChessSquare &at(const int file, const int rank);

    /*!
     * \brief Get a reference to the board square.
     *
     * \param pos   Chess board position.
     *
     * \return If in range, returns the reference to the board's square.\n
     * If out-of-range, returns "no square" square
     * (see ChessSquare::isOnChessBoard()).
     */
    ChessSquare &at(const ChessPos &pos);

    /*!
     * \brief Set unicode GUI state.
     *
     * \param on_off    State.
     */
    void setGuiState(bool on_off)
    {
      m_bGui = on_off;
    }

    /*!
     * \brief Determine moves source position from the the move and board state.
     *
     * \param [in,out] move   Chess move.
     *
     * \return Returns true of source position determined, false otherwise.
     */
    bool setMoveSrcPos(ChessMove &move)
    

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Static Member Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Convert chess rank to board array row.
     *
     * \note Rank 8 is row 7, ..., rank 1 is row 0.
     *
     * \param rank    Chess rank ['1', '8'].
     *
     * \return Board row (may be out-of-range).
     */
    static int toRow(int rank);

    /*!
     * \brief Convert chess file to board array column.
     *
     * \param file    Chess file ['a', 'h'].
     *
     * \return Board column (may be out-of-range).
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
     * \reture Chess file ['a', 'h'] or NoFile.
     */
    static ChessFile toFile(int col);

    /*!
     * \brief Convert board array row to chess rank.
     *
     * \note Rank 8 is row 7, ..., rank 1 is row 0.
     *
     * \param row   Board row [0, 7].
     *
     * \return Chess rank ['1', '8'] or NoRank.
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
     * \brief Shift file by the given offset.
     *
     * \param file    Given chess board file.
     * \param offset  Plus/minus offset from file.
     *
     * \returns Returns the shifted file. If off the board, then returns NoFile.
     */
    static ChessFile shiftFile(int file, int offset)
    
    /*!
     * \brief Shift rank by the given offset.
     *
     * \param rank    Given chess board rank.
     * \param offset  Plus/minus offset from rank.
     *
     * \returns Returns the shifted rank. If off the board, then returns NoRank.
     */
    static ChessRank shiftRank(int rank, int offset)

    /*!
     * \brief Test if position lies on a standard chess board.
     *
     * \param pos Chess board position.
     *
     * \return Returns true or false.
     */
    static bool isOnChessBoard(const ChessPos &pos);

    /*!
     * \brief Test if position lies on a standard chess board.
     *
     * \param file  Chess file.
     * \param rank  Chess rank.
     *
     * \return Returns true or false.
     */
    static bool isOnChessBoard(const ChessFile file, const ChessRank rank);

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
     * \brief Find all of the major pieces's candidate move positions.
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the major pieces.
     *
     * Since major pieces's movements are invertible, the list of positions
     * serve both as a previous source and future destination positions. The
     * list radiates outward from the current position.
     *
     * \param ePiece      Major chess piece type.
     * \param pos         The major pieces's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findMajorPieceMoves(const ChessPiece ePiece,
                                    const ChessPos   &pos,
                                    list_of_pos      &positions);

    /*!
     * \brief Find all of the king's candidate move positions.
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the king.
     *
     * Since the king's movements are invertible, the list of positions serve
     * both as a previous source and future destination positions. The list
     * radiates outward from the current position.
     *
     * \param pos         The king's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findKingMoves(const ChessPos &pos, list_of_pos &positions);

    /*!
     * \brief Find all of the queens's candidate move positions.
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the queens.
     *
     * Since the queens's movements are invertible, the list of positions serve
     * both as a previous source and future destination positions. The list
     * radiates outward from the current position.
     *
     * \param pos         The queens's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findQueenMoves(const ChessPos &pos, list_of_pos &positions);

    /*!
     * \brief Find all of the bishop's candidate move positions.
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the bishop.
     *
     * Since the bishop's movements are invertible, the list of positions serve
     * both as a previous source and future destination positions. The list
     * radiates outward from the current position.
     *
     * \param pos         The bishop's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findBishopMoves(const ChessPos &pos, list_of_pos &positions);

    /*!
     * \brief Find all of the knight's candidate move positions.
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the knight.
     *
     * Since the knight's movements are invertible, the list of positions serve
     * both as a previous source and future destination positions. The list
     * radiates outward from the current position.
     *
     * \param pos         The knight's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findKnightMoves(const ChessPos &pos, list_of_pos &positions);

    /*!
     * \brief Find all of the rook's candidate move positions.
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the rook.
     *
     * Since the rook's movements are invertible, the list of positions serve
     * both as a previous source and future destination positions. The list
     * radiates outward from the current position.
     *
     * \param pos         The rook's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findRookMoves(const ChessPos &pos, list_of_pos &positions);

    /*!
     * \brief Find all of the pawn's candidate source move positions.
     *
     * How did the pawn get here?
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the pawn.
     *
     * Pawns can only move forward and that direction is determine by its color.
     * A pawn's first move can be 1 or 2 squares forward. A capture moves
     * diagonally. The capture move, and when appropriate, the first 2 square
     * move are added to the list.
     *
     * \param eColor      The color of the pawn.
     * \param pos         The pawn's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findPawnSrcMoves(const ChessColor eColor,
                                 const ChessPos   &pos,
                                 list_of_pos      &positions);

    /*!
     * \brief Find all of the pawn's candidate destination move positions.
     *
     * Where can the pawn move?
     *
     * No board state is taken into account. That is, consider the board as 
     * empty accept for the pawn.
     *
     * Pawns can only move forward and that direction is determine by its color.
     * A pawn's first move can be 1 or 2 squares forward. A capture moves
     * diagonally. The capture move, and when appropriate, the first 2 square
     * move are added to the list.
     *
     * \param eColor      The color of the pawn.
     * \param pos         The pawn's current position.
     * \param positions   List (vector) of candidate positions. Any candidate
     *                    position is appended to the end of the list. The
     *                    list is not cleared.
     */
    static void findPawnDstMoves(const ChessColor eColor,
                                 const ChessPos   &pos,
                                 list_of_pos      &positions);

  protected:
    ChessSquare m_board[NumOfRanks][NumOfFiles];  ///< board matrix
    bool        m_bGui;                           ///< [not] a gui stream output

    /*!
     * \brief One-time board initialization.
     */
    void initBoard();
  
    /*!
     * \brief Set up one side of the board for the start a game.
     *
     * \param eColor Side's color.
     */
    void setupBoard(ChessColor eColor);
  
    void recordHistory(ChessMove &move);

    void dropInBoneYard();

    friend std::ostream &operator<<(std::ostream &os, const ChessBoard &game);
  };

  extern std::ostream &operator<<(std::ostream &os, const ChessBoard &game);

} // chess_engine

#endif // _CE_BOARD_H
