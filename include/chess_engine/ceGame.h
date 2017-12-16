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

#ifndef _CE_GAME_H
#define _CE_GAME_H

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceSquare.h"
#include "chess_engine/ceBoard.h"
#include "chess_engine/cePlayer.h"
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
     * The caller provides the two antagonists. If NULL, an anonymous player is
     * provided.
     *
     * White moves first.
     *
     * \param pWhite    Pointer to player playing white.
     * \param pBlack    Pointer to player playing black.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int startNewGame(ChessPlayer *pWhite = NULL, ChessPlayer *pBlack = NULL);
    
    /*!
     * \brief End current game.
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
     * \brief Qualify the chess move source and destination positions.
     *
     * \param [in,out] move   Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int qualifyMovePositions(ChessMove &move);

    /*!
     * \brief Qualify the moved chess piece.
     *
     * \param [in,out] move   Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int qualifyMovePiece(ChessMove &move);

    /*!
     * \brief Qualify any capture.
     *
     * \param [in,out] move   Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int qualifyMoveCapture(ChessMove &move);

    /*!
     * \brief Execute move.
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
    bool isPlayingAGame() const
    {
      return m_bIsPlaying;
    }

    /*!
     * \brief Get current or most recent players' names.
     *
     * \param [out] strWhite  White player's name.
     * \param [out] strBlack  Black player's name.
     */
    void getPlayerNames(std::string &strWhite, std::string &strBlack) const;

    /*!
     * \brief Get the game's winner, if any.
     *
     * \return Returns White, Black, or NoColor.
     */
    ChessColor getWinner() const
    {
      return m_eWinner;
    }

    /*!
     * Get the current play state code.
     *
     * \return Returns reason code.
     */
    ChessResult getPlayState() const
    {
      return m_ePlayState;
    }

    /*!
     * \brief Get the number of plies (half-moves) played.
     *
     * \return Number of plies.
     */
    int getNumOfPliesPlayed() const;

    /*!
     * \brief Get the number of completed moves played.
     *
     * One completed move is White then Black, with White always starting.
     *
     * \return Number of moves.
     */
    int getNumOfMovesPlayed() const;

    /*!
     * \brief Get the current move number in play.
     *
     * \return If a game is in play, returns move number \> 0. Else returns 0.
     */
    int getMoveNumInPlay() const;

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
    ChessBoard &getBoard()
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
    const ChessBoneYard &getBoneYard(const ChessColor ePlayer) const
    {
      return ePlayer == White? m_boneyardWhite: m_boneyardBlack;
    }

    /*!
     * \brief Get the game history.
     *
     * \return Reference to history.
     */
    const ChessHistory &getGameHistory() const
    {
      return m_history;
    }

    /*!
     * \brief Get game history at the give ply.
     *
     * \param nPlyNum   Ply number (1/2 moves)
     *
     * \return Reference to history move.
     */
    const ChessMove &getHistoryAt(int nPlyNum) const;

    /*!
     * \brief Set unicode graphic state.
     *
     * \param on_off    State.
     */
    void setGuiState(bool on_off)
    {
      m_bGraphic = on_off;
      m_board.setGraphicState(on_off);
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
    ChessResult   m_ePlayState;     ///< current play state
    ChessColor    m_eWinner;        ///< end of game winner, if any

    //
    // Player state
    //
    ChessColor      m_eTurnToMove;    ///< current player's turn to move
    ChessPromotion  m_numPromotions;  ///< number of promotions/player
    ChessPlayer     m_anonWhite;      ///< anonymous white player
    ChessPlayer     m_anonBlack;      ///< anonymous black player
    ChessPlayer    *m_pPlayerWhite;   ///< pointer to current white player
    ChessPlayer    *m_pPlayerBlack;   ///< pointer to current black player

    //
    // Other
    //
    bool      m_bGraphic;             ///< [not] a graphic stream output

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

    //
    // Friends
    //
    friend std::ostream &operator<<(std::ostream &os, const ChessGame &game);
  };


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Friends
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief ChessGame output stream operator.
   *
   * \param os    Output stream.
   * \param board Chess game.
   *
   * \return Output stream.
   */
  extern std::ostream &operator<<(std::ostream &os, const ChessGame &game);

} // chess_engine

#endif // _CE_GAME_H
