////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      cePlayer.h
//
/*! \file
 *
 * \brief The chess player class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2017  RoadNarrows
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

#ifndef _CE_PLAYER_H
#define _CE_PLAYER_H

#include <stddef.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/appkit/Time.h"

#include "chess_engine/ceTypes.h"

/*!
 * \brief Package top-level namespace.
 */
namespace chess_engine
{
  // ---------------------------------------------------------------------------
  // Class PlayerInfo
  // ---------------------------------------------------------------------------

  /*!
   * \brief Player information container class.
   */
  class PlayerInfo
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PlayerInfo();

    /*!
     * \brief Initialization constructor.
     *
     * \param id      Player numeric identifier.
     * \param strName Player's name.
     * \param eType   Type of player.
     */
    PlayerInfo(const ChessPlayerId   id,
               const std::string     &strName,
               const ChessPlayerType eType);

    /*!
     * \brief Destructor.
     */
    virtual ~PlayerInfo();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs Right-hand side object.
     *
     * \return Reference to this.
     */
    PlayerInfo &operator=(const PlayerInfo &rhs);

    /*!
     * \brief Clear data.
     */
    void clear();

    /*!
     * \brief Return player's identification number.
     *
     * \return Number.
     */
    const ChessPlayerId id() const
    {
      return m_id;
    }

    /*!
     * \brief Return player's name.
     *
     * \return String.
     */
    const std::string &name() const
    {
      return m_strName;
    }

    /*!
     * \brief Return player type.
     *
     * \return Enum.
     */
    const ChessPlayerId type() const
    {
      return m_eType;
    }
    
    /*!
     * \brief PlayerInfo output stream operator.
     *
     * \param os    Output stream.
     * \param obj   Object to insert.
     *
     * \return Output stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const PlayerInfo &obj);

    friend class ChessPlayer;

  protected:
    ChessPlayerId   m_id;       ///< player's id
    std::string     m_strName;  ///< player's name
    ChessPlayerType m_eType;    ///< type of player

  }; // class PlayerInfo


  // ---------------------------------------------------------------------------
  // Class PlayerSummary
  // ---------------------------------------------------------------------------

  /*!
   * \brief Player summary container class.
   */
  class PlayerSummary
  {
  public:
    /*!
     * \brief Default constructor.
     */
    PlayerSummary();

    /*!
     * \brief Destructor.
     */
    virtual ~PlayerSummary();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs Right-hand side object.
     *
     * \return Reference to this.
     */
    PlayerSummary &operator=(const PlayerSummary &rhs);

    /*!
     * \brief Clear data.
     */
    void clear();

    /*!
     * \brief Return number of games started.
     *
     * \return Number.
     */
    const size_t games() const
    {
      return m_games;
    }

    /*!
     * \brief Set the number of games started.
     *
     * \param n New number.
     */
    void games(const size_t n)
    {
      m_games = n;
    }

    /*!
     * \brief Return number of wins.
     *
     * \return Number.
     */
    const size_t wins() const
    {
      return m_wins;
    }

    /*!
     * \brief Set the number of wins.
     *
     * \param n New number.
     */
    void wins(const size_t n)
    {
      m_wins = n;
    }

    /*!
     * \brief Return number of losses.
     *
     * \return Number.
     */
    const size_t losses() const
    {
      return m_losses;
    }

    /*!
     * \brief Set the number of losses.
     *
     * \param n New number.
     */
    void losses(const size_t n)
    {
      m_losses = n;
    }

    /*!
     * \brief Return number of draws.
     *
     * \return Number.
     */
    const size_t draws() const
    {
      return m_draws;
    }

    /*!
     * \brief Set the number of draws.
     *
     * \param n New number.
     */
    void draws(const size_t n)
    {
      m_draws = n;
    }

    /*!
     * \brief PlayerSummary output stream operator.
     *
     * \param os    Output stream.
     * \param obj   Object to insert.
     *
     * \return Output stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const PlayerSummary &obj);

    friend class ChessPlayer;

  protected:
    size_t m_games;     ///< number of games started
    size_t m_wins;      ///< number of wins
    size_t m_losses;    ///< number of losses
    size_t m_draws;     ///< number of draws

  }; // class PlayerSummary


  // ---------------------------------------------------------------------------
  // Class GameRecord
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Historical record of game played container class.
   */
  class GameRecord
  {
  public:
    /*!
     * \brief Default constructor.
     */
    GameRecord();

    /*!
     * \brief Destructor.
     */
    virtual ~GameRecord();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs Right-hand side object.
     *
     * \return Reference to this.
     */
    GameRecord &operator=(const GameRecord &rhs);

    /*!
     * \brief Clear data.
     */
    void clear();

    /*!
     * \brief Return player's information.
     *
     * \return Reference to PlayerInfo.
     */
    const PlayerInfo &playerInfo() const
    {
      return m_infoPlayer;
    }

    /*!
     * \brief Return opponent's information.
     *
     * \return Reference to PlayerInfo.
     */
    const PlayerInfo &opponentInfo() const
    {
      return m_infoOpponent;
    }

    /*!
     * \brief Return color played by player.
     *
     * \return Color.
     */
    const ChessColor colorPlayed() const
    {
      return m_eColorPlayed;
    }

    /*!
     * \brief Return game's start and end times.
     *
     * \param [out] timeStart   Start time (UTC).
     * \param [out] timeEnd     End time (UTC).
     */
    const void gameTimes(rnr::chronos::Time &timeStart,
                         rnr::chronos::Time &timeEnd) const
    {
      timeStart = m_timeStart;
      timeEnd   = m_timeEnd;
    }

    /*!
     * \brief Return number of moves made in the game.
     *
     * \return Number.
     */
    const size_t numOfGameMoves() const
    {
      return m_uNumMoves;
    }

    /*!
     * \brief Return the winner of the game.
     *
     * \return Color.
     */
    const ChessColor gameWinner() const
    {
      return m_eWinner;
    }

    /*!
     * \brief Return the game result
     *
     * \return Result.
     */
    const ChessResult gameResult() const
    {
      return m_eResult;
    }

    /*!
     * \brief GameRecord output stream operator.
     *
     * \param os    Output stream.
     * \param obj   Object to insert.
     *
     * \return Output stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const GameRecord &record);

    friend class ChessPlayer;

  protected:
    PlayerInfo          m_infoPlayer;   ///< player information
    PlayerInfo          m_infoOpponent; ///< opponent information
    ChessColor          m_eColorPlayed; ///< color played
    rnr::chronos::Time  m_timeStart;    ///< time game started (UTC)
    rnr::chronos::Time  m_timeEnd;      ///< time game ended (UTC)
    size_t              m_uNumMoves;    ///< number of moves in game
    ChessColor          m_eWinner;      ///< color of winner, if any
    ChessResult         m_eResult;      ///< game result
  }; // class GameRecord

  //
  // Record History
  //
  typedef std::vector<GameRecord>       PlayerHistory;      ///< player history
  typedef PlayerHistory::iterator       PlayerHistoryIter;  ///< hist iterator
  typedef PlayerHistory::const_iterator PlayerHistoryCIter; ///< hist const iter


  // ---------------------------------------------------------------------------
  // Class ChessPlayer
  // ---------------------------------------------------------------------------

  /*!
   * \brief Chess player class.
   */
  class ChessPlayer
  {
  public:
    /*!
     * \brief Default constructor.
     */
    ChessPlayer();

    /*!
     * \brief Initialization constructor.
     *
     * \param id      Player numeric identifier.
     * \param strName Player's name.
     * \param eType   Type of player.
     */
    ChessPlayer(const ChessPlayerId   id,
                const std::string     strName,
                const ChessPlayerType eType);

    ChessPlayer(const PlayerInfo &info);

    /*!
     * \brief Destructor.
     */
    virtual ~ChessPlayer();

    /*!
     * \brief Assignment operator.
     *
     * Only the player information is copied. History is erased.
     *
     * \param rhs Right-hand side object.
     *
     * \return Reference to this.
     */
    ChessPlayer &operator=(const ChessPlayer &rhs);

    /*!
     * \brief Clear data.
     */
    void clear();

    /*!
     * \brief Return player's numeric identification.
     *
     * \return Number.
     */
    const ChessPlayerId id() const
    {
      return m_info.m_id;
    }

    /*!
     * \brief Test if player is the special "no player".
     */
    bool isNoPlayer()
    {
      return m_info.id() == NoPlayerId;
    }

    /*!
     * \brief Return player's name.
     *
     * \return String.
     */
    const std::string &name() const
    {
      return m_info.m_strName;
    }

    /*!
     * \brief Return type of player.
     *
     * \return Enum.
     */
    const ChessPlayerType type() const
    {
      return m_info.m_eType;
    }

    /*!
     * \brief Return player's information.
     *
     * \return Reference to PlayerInfo.
     */
    const PlayerInfo &info() const
    {
      return m_info;
    }

    /*!
     * \brief Return player's current color.
     *
     * \return Color. NoColor if not playing.
     */
    const ChessColor color() const
    {
      return m_eColor;
    }

    /*!
     * \brief Test if player is currently playing.
     *
     * \return Returns true or false.
     */
    const bool isPlaying() const
    {
      return m_eColor != NoColor;
    }

    /*!
     * \brief Return player's summary of played games.
     *
     * \return Reference to PlayerSummary.
     */
    const PlayerSummary &summary() const
    {
      return m_summary;
    }

    /*!
     * \brief Return full history of player.
     *
     * The history is in chronological order.
     *
     * \return Reference to PlayerHistory.
     */
    const PlayerHistory &history() const
    {
      return m_history;
    }

    /*!
     * \brief Return the number of historical records.
     *
     * \return Number.
     */
    const size_t sizeHistory() const
    {
      return m_history.size();
    }

    /*!
     * \brief Return the game record at the given index.
     *
     * \param i   Record index.
     *
     * \return Reference to GameRecord.
     */
    const GameRecord &recordAt(const size_t i) const
    {
      return m_history[i];
    }

    /*!
     * \brief Mark the start of a new game for the player.
     *
     * A new record is entered into the history.
     *
     * \param eColor        Color played.
     * \param infoOpponent  Opponent's information.
     * \param timeStart     Time game started (UTC).
     */
    void markStartOfGame(const ChessColor         &eColor,
                         const PlayerInfo         &infoOpponent,
                         const rnr::chronos::Time &timeStart);

    /*!
     * \brief Mark the end of the current game for the player.
     *
     * The historic record is updated.
     *
     * \param eWinner   The color of the winner.
     * \param eResult   The end of game result.
     * \param uNumMoves Number of moves made in game.
     * \param timeEnd   Time game ended (UTC).
     */
    void markEndOfGame(const ChessColor         eWinner,
                       const ChessResult        eResult, 
                       const size_t             uNumMoves,
                       const rnr::chronos::Time &timeEnd);

    //TODO int write(const std::string filename);

    //TODO int read(const std::string filename);

    /*!
     * \brief Create a new persona.
     *
     * The player information is updated, but any previous history is erased.
     * No identity theft here.
     *
     * \param id      Player numeric identifier.
     * \param strName Player's name.
     * \param eType   Type of player.
     */
    void newPersona(const ChessPlayerId   id,
                    const std::string     strName,
                    const ChessPlayerType eType);

    /*!
     * \brief Return "no player" player.
     *
     * \return ChessPlayer.
     */
    static ChessPlayer &noplayer();

    /*!
     * \brief ChessPlayer output stream operator.
     *
     * \param os    Output stream.
     * \param obj   Object to insert.
     *
     * \return Output stream.
     */
    friend std::ostream &operator<<(std::ostream &os, const ChessPlayer &obj);

  protected:
    PlayerInfo      m_info;     ///< player's information
    ChessColor      m_eColor;   ///< player's current color
    PlayerSummary   m_summary;  ///< summary of player's prowess
    PlayerHistory   m_history;  ///< player's history
 
  }; // class ChessPlayer

} // namespace chess_engine

#endif // _CE_PLAYER_H
