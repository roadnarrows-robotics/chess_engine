////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceChess.h
//
/*! \file
 *
 * \brief Top-level chess class declarations.
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

#ifndef _CE_CHESS_H
#define _CE_CHESS_H

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceGame.h"
#include "chess_engine/ceParser.h"
#include "chess_engine/ceMove.h"

namespace chess_engine
{
  //----------------------------------------------------------------------------
  // Class Chess
  //----------------------------------------------------------------------------

  class Chess
  {
  public:
    /*!
     * \brief Default constructor.
     */
    Chess();

    /*!
     * \brief Destructor.
     */
    virtual ~Chess();

    /*!
     * \brief Initialize chess.
     *
     * Connection is made to the backend chess engine.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int init();

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
     * \brief End current game.
     *
     * \param eReason   Reason game ended.
     * \param eWinner   Declared winner, if any.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int endCurrentGame(ChessResult eReason, ChessColor eWinner);

    /*!
     * \brief Resign from the game.
     *
     * \param ePlayer Player resigning.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int resign(const ChessColor ePlayer);

    /*!
     * \brief Make a chess move.
     *
     * The move is made against the chess engine, and if successful, applied
     * to the game state.
     *
     * \param ePlayer         Player (color) making the move.
     * \param posSrc          Source chess position of move.
     * \param posDst          Destination chess position of move.
     * \param ePiecePromoted  Promoted piece, if any.
     * \param [out] move      Fully qualified chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int makeAMove(const ChessColor ePlayer,
                  const ChessPos   &posSrc,
                  const ChessPos   &posDst,
                  const ChessPiece ePiecePromoted,
                  ChessMove        &move);

    /*!
     * \brief Make a chess move.
     *
     * The move is made against the chess engine, and if successful, applied
     * to the game state.
     *
     * \param ePlayer         Player (color) making the move.
     * \param strAN           CAN or SAN move.
     * \param [out] move      Fully qualified chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int makeAMove(const ChessColor  ePlayer,
                  const std::string &strAN,
                  ChessMove         &move);

    /*!
     * \brief Get backend chess engine move.
     *
     * If successful, applied to the game state.
     *
     * \param [out] move      Fully qualified chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int getEnginesMove(ChessMove &move);

    /*!
     * \brief Set game difficulty level.
     *
     * \param fDifficulty   Difficulty level [1.0, 10.0] with 1 being the 
     *                      easiest.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int setGameDifficulty(float fDifficulty)
    {
      m_engine.setGameDifficulty(fDifficulty);
    }

    /*!
     * \brief Get backend chess engine.
     *
     * \return Reference to engine object.
     */
    ChessEngineGnu &getEngine()
    {
      return m_engine;
    }

    /*!
     * \brief Get game state.
     *
     * \return Reference to game object.
     */
    ChessGame &getGame()
    {
      return m_game;
    }

    /*!
     * \brief Get chess board state.
     *
     * \return Reference to board object.
     */
    ChessBoard &getBoard()
    {
      return m_game.getBoard();
    }

  protected:
    ChessEngineGnu    m_engine;   ///< backend chess engine
    ChessGame         m_game;     ///< game state
    ChessANParser     m_parser;   ///< AN parser

    /*!
     * \brief Finalize move result.
     *
     * Additional chess information may effect the current result.
     *
     * \param eCurrResult   Curret result value.
     *
     * \return Final result value.
     */
    ChessResult finalizeResult(const ChessResult eCurResult);
  };

} // namespace chess_engine


#endif // _CE_CHESS_H
