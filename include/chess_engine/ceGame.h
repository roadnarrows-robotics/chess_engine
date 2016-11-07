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
#include <map>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceSquare.h"
#include "chess_engine/ceBoard.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"

/*!
 * \brief Package top-level namespace.
 */
namespace chess_engine
{
  // ---------------------------------------------------------------------------
  // Class ChessGame
  // ---------------------------------------------------------------------------

  /*!
   * \brief Chess game class.
   */
  class ChessGame
  {
  public:
    //
    // Types
    //
    typedef std::vector<ChessFqPiece> ChessBoneYard;    ///< captured pieces
    typedef std::vector<ChessMove>    ChessHistory;     ///< game history
    typedef std::map<ChessColor, int> ChessPromotion;   ///< promotions/player
    typedef std::map<ChessColor, std::string> ChessPlayer; ///< player names

    /*!
     * \brief Default constructor.
     */
    ChessGame();
  
    /*!
     * \brief Destructor.
     */
    virtual ~ChessGame();
  
    /*!
     * \brief Start a new game.
     *
     * White moves first.
     *
     * \param strWhite    White player's nom de guerre.
     * \param strBlack    Black player's nom de guerre.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int startNewGame(const std::string &strWhite, const std::string &strBlack);
    
    /*!
     * \brief Eng current game.
     *
     * \param eReason   Reason game ended.
     * \param eWinner   Declared winner, if any.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int endCurrentGame(ChessResult eReason, ChessColor eWinner = NoColor);
    
    /*!
     * \brief Fully qualify the chess move.
     *
     * The move is reasonably verified and additional move fields are set
     * by examining the game and board state.
     *
     * \param [in,out] move   Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int qualifyMove(ChessMove &move);

    /*!
     * \brief Execute player's move.
     *
     * \param [in] move   Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int execMove(ChessMove &move);
    
    /*!
     * \brief Set up the game.
     */
    void setupGame();
  
    /*!
     * \brief Test if a game is being played.
     *
     * \return Returns true or false.
     */
    bool isPlaying()
    {
      return m_bIsPlaying;
    }

    /*!
     * \brief Get the game's winner, if any.
     *
     * \return Returns White, Black, or NoColor.
     */
    ChessColor getWinner()
    {
      return m_eWinner;
    }

    /*!
     * Get the reason for the gaming ending.
     *
     * \return Returns reason code.
     */
    ChessResult getEndOfGameReason()
    {
      return m_eEoGReason;
    }

    /*!
     * \brief Get the number of moves played.
     *
     * \return Number of moves.
     */
    int getNumOfMoves();

    /*!
     * \brief Get the number of plies (half-moves) played.
     *
     * \return Number of plies.
     */
    int getNumOfPlies();

    /*!
     * \brief Get a reference to the board square.
     *
     * \param file  Chess board file.
     * \param rank  Chess board rank.
     *
     * \return If in range, returns the reference to the board's square.\n
     * If out-of-range, returns "no square" square
     * (check with ChessSquare.isOnChessBoard).
     */
    ChessSquare &getBoardSquare(const int file, const int rank);

    /*!
     * \brief Get a reference to the board square.
     *
     * \param pos   Chess board position.
     *
     * \return If in range, returns the reference to the board's square.\n
     * If out-of-range, returns "no square" square
     * (check with ChessSquare.isOnChessBoard).
     */
    ChessSquare &getBoardSquare(const ChessPos &pos);

    /*!
     * \brief Get the chess board state.
     *
     * \return Reference to board.
     */
    const ChessBoard &getBoard()
    {
      return m_board;
    }

    /*!
     * \brief Get the player's bone yard (captured pieces).
     *
     * \param ePlayer   Player and color of captured piece.
     *
     * \return Reference to bone yard.
     */
    const ChessBoneYard &getBoneYard(const ChessColor ePlayer)
    {
      return ePlayer == White? m_boneyardWhite: m_boneyardBlack;
    }

    /*!
     * \brief Get the game history.
     *
     * \return Reference to history.
     */
    const ChessHistory &getGameHistory()
    {
      return m_history;
    }

    /*!
     * \brief Set unicode GUI state.
     *
     * \param on_off    State.
     */
    void setGuiState(bool on_off)
    {
      m_bGui = on_off;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Static Member Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


  protected:
    //
    // Layout
    //
    ChessBoard    m_board;          ///< the standard chess board
    ChessBoneYard m_boneyardWhite;  ///< captured white pieces
    ChessBoneYard m_boneyardBlack;  ///< captured black pieces
    ChessHistory  m_history;        ///< game history

    //
    // Game state
    //
    bool          m_bIsPlaying;     ///< is [not] playing a game
    ChessResult   m_eEoGReason;     ///< end of game reason, if any
    ChessColor    m_eWinner;        ///< end of game winner, if any

    //
    // Player state
    //
    ChessColor      m_eTurnToMove;    ///< current player's turn to move
    ChessPlayer     m_playerName;     ///< player's name 
    ChessPromotion  m_numPromotions;  ///< number of promotions/player

    //
    // Other
    //
    bool                    m_bGui;           ///< [not] a gui stream output

    /*!
     * \brief Record game history.
     *
     * \param move    Move to record.
     */
    void recordHistory(ChessMove &move);

    /*!
     * \brief Drop piece into the player's bone yard.
     *
     * The piece is removed from the board.
     *
     * \param ePlayer   Player and color of captured piece.
     * \param pos       Board position of piece.
     */
    void dropInBoneYard(const ChessColor ePlayer, const ChessPos &pos);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Friends
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    friend std::ostream &operator<<(std::ostream &os, const ChessGame &game);
  };

  extern std::ostream &operator<<(std::ostream &os, const ChessGame &game);

} // chess_engine

#endif // _CE_GAME_H
