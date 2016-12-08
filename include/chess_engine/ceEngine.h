////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceEngine.h
//
/*! \file
 *
 * \brief The GNU chess gnuchess backend engine interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2016  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 *
 * \par Licence:
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

#ifndef _CE_ENGINE_H
#define _CE_ENGINE_H

#include <sys/types.h>
#include <signal.h>
#include <pthread.h>

#include <string>
#include <vector>

#include <boost/regex.hpp>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"


namespace chess_engine
{
  // ---------------------------------------------------------------------------
  // Class ChessEngine
  // ---------------------------------------------------------------------------

  /*!
   * \brief Chess backend engine virtual base class.
   */
  class ChessEngine
  {
  public:
    /*!
     * \brief Default constructor.
     *
     * \param strEngineName   Backend chess engine's name identifier.
     *
     * Keep light weight.
     */
    ChessEngine(std::string strEngineName="unknown");
  
    /*!
     * \brief Destructor.
     */
    virtual ~ChessEngine();
  
    /*!
     * \brief Open connection to backend engine.
     *
     * This is where the heavy hitting initialization occurs. Opening a
     * connection is an abstraction. Depending on the backend, no actual
     * file I/O is necessary.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int openConnection() = 0;
  
    /*!
     * \brief Close connection to backend engine.
     *
     * Relevant resources should be released.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int closeConnection() = 0;
  
    /*!
     * \brief Get backend engine's name.
     *
     * \return String.
     */
    virtual std::string getEngineName()
    {
      return m_strEngineName;
    }

    /*!
     * \brief Get backend engine's version.
     *
     * \return String.
     */
    virtual std::string getEngineVersion()
    {
      return m_strEngineVersion;
    }

    /*!
     * \brief Set game difficulty level.
     *
     * \param fDifficulty   Difficulty level [1.0, 10.0] with 1 being the 
     *                      easiest.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int setGameDifficulty(float fDifficulty);

    /*!
     * \brief Get game difficulty level.
     *
     * \return Difficulty level [1.0, 10.0] with 1 being the easiest.
     */
    virtual int getGameDifficulty() const
    {
      return m_fDifficulty;
    }

    /*!
     * \brief Start a new game.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int startNewGame();

    /*!
     * \brief End current game.
     *
     * \param eReason   Reason game ended.
     * \param eWinner   Declared winner, if any.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int endCurrentGame(ChessResult eReason, ChessColor eWinner=NoColor);

    /*!
     * \brief Make a player's move against the chess engine.
     *
     * The move is specified in Algebraic Notation.
     *
     * \param ePlayer         Player making the move.
     * \param [in,out] strAN  Move in AN. The string may be modified.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int makePlayersMove(const ChessColor ePlayer,
                                std::string      &strAN) = 0;

    /*!
     * \brief Compute backend chess engine's move.
     *
     * \param [out] eMoveColor  Color of move.
     * \param [out] strAN       Move in Algebraic Notation.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int computeEnginesMove(ChessColor  &eMoveColor,
                                   std::string &strAN) = 0;

    /*!
     * \brief Resign from the game.
     *
     * \param ePlayer Player resigning.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int resign(const ChessColor ePlayer);

    /*!
     * \brief Get available castling move options.
     *
     * \param [out] availWhite  List(vector) of available white castling moves.
     * \param [out] availBlack  List(vector) of available black castling moves.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int getCastlingOptions(list_of_castling &availWhite,
                                   list_of_castling &availBlack)
    {
      return -CE_ECODE_NO_SUPP; // not supported
    }

    /*!
     * \brief Get whose turn is it to move.
     *
     * \return Player's color
     */
    virtual ChessColor whoseTurn() const
    {
      return m_eColorTurn;
    }

    /*!
     * \brief Get the current move number in play.
     *
     * \return If a game is in play, returns move number \> 0. Else returns 0.
     */
    virtual int getMoveNumInPlay() const
    {
      return m_nMoveNum;
    }

    /*!
     * \brief Test if a game is currently being played.
     *
     * \return Returns true or false.
     */
    virtual bool isPlayingAGame() const
    {
      return m_bIsPlaying;
    }

    /*!
     * \brief Test if backend chess engine is busy.
     *
     * \return Returns true or false.
     */
    virtual bool isBusy() const
    {
      return m_bIsBusy;
    }

    /*!
     * \brief Test if connected/initialized with backend chess engine.
     *
     * \return Returns true or false.
     */
    virtual bool isConnected() const
    {
      return m_bIsConn;
    }

  protected:
    std::string   m_strEngineName;    ///< backend engine name string
    std::string   m_strEngineVersion; ///< backend engine version string
    bool          m_bIsConn;          ///< [not] connected to backend engine
    bool          m_bIsPlaying;       ///< [no] active game in play
    bool          m_bIsBusy;          ///< backend engine is [not] busy
    float         m_fDifficulty;      ///< game engine difficulty [1,10]
    ChessColor    m_eColorTurn;       ///< color to play
    int           m_nMoveNum;         ///< move number currently in play
    ChessResult   m_eEoGReason;       ///< reason for ending game
    ChessColor    m_eWinner;          ///< winner, if any

    /*!
     * \brief Alternate player turns.
     *
     * \param ePlayerThatMoved  Player that made the move.
     *
     * \return Next player to move.
     */
    ChessColor alternateTurns(const ChessColor ePlayerThatMoved);
  };


  // ---------------------------------------------------------------------------
  // Class ChessEngineGnu
  // ---------------------------------------------------------------------------

  /*!
   * \brief GNU chess (gnuchess) backend engine class.
   */
  class ChessEngineGnu : public ChessEngine
  {
  public:
    static const int    MaxLineSize = 80;       ///< maximum line length
    static const float  DepthDifficulty = 2.0;  ///< depth/difficulty ratio
    static const int    DepthDft = 2;           ///< default search depth

    /*!
     * \brief Structure to hold intermediate parsed backend engine values.
     */
    struct BeParsedVars
    {
      int         m_nMoveNum;     ///< read new move number
      std::string m_strAN;        ///< read parsed AN
      std::string m_strSAN;       ///< read parsed SAN
      ChessResult m_eEoGResult;   ///< read end of game result
      ChessColor  m_eEoGWinner;   ///< read end of game result winner
    };

    /*!
     * \brief Default constructor.
     */
    ChessEngineGnu();
  
    /*!
     * \brief Destructor.
     */
    virtual ~ChessEngineGnu();
  

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Base Interface
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
    /*!
     * \brief Open connection to backend gnuchess engine.
     *
     * The gnuchess application is fork'exec'ed and bi-direction pipe is set
     * up with the backend engine.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int openConnection();
  
    /*!
     * \brief Close connection to backend gnuchess engine.
     *
     * Relevant resources should be released.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int closeConnection();

    /*!
     * \brief Set game difficulty level.
     *
     * \param fDifficulty   Difficulty level [1.0, 10.0] with 1 being the 
     *                      easiest.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int setGameDifficulty(float fDifficulty);

    /*!
     * \brief Start a new game.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int startNewGame();

    /*!
     * \brief Make a player's move against the chess engine.
     *
     * The move is specified in Algebraic Notation.
     *
     * \param ePlayer         Player making the move.
     * \param [in,out] strAN  Make a move.
     *                        On input, the move can be specified in CAN or SAN.
     *                        On success, the output is SAN.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int makePlayersMove(const ChessColor ePlayer, std::string &strAN);

    /*!
     * \brief Compute backend chess engine's move.
     *
     * \param [out] eMoveColor  Color of move.
     * \param [out] strAN       Move in Standard Algebraic Notation.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int computeEnginesMove(ChessColor &eMoveColor, std::string &strAN);

    /*!
     * \brief Get available castling move options.
     *
     * \param [out] availWhite  List(vector) of available white castling moves.
     * \param [out] availBlack  List(vector) of available black castling moves.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int getCastlingOptions(list_of_castling &availWhite,
                                   list_of_castling &availBlack);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Extended Interface
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get any end of game declaration by engine.
     *
     * \param [out] eResult End of game result, if any.
     * \param [out] eWinner End of game winner, if any.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int getEnginesEoGDecl(ChessResult &eResult, ChessColor &eWinner);
    

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Low-Level I/O Interface
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
    /*!
     * \brief Read a line from the backend engine.
     *
     * \param [out] strLine   Input line read.
     * \param msec            Read timeout.
     *
     * \return On success returns number of bytes read. On failure, returns
     * negatvie error code.
     */
    virtual int readline(std::string &strLine, unsigned int msec=100);

    /*!
     * \brief Read a line from the backend engine.
     *
     * \param [out] buf   Input buffer read.
     * \param sizeBuf     Size of buffer (bytes).
     * \param msec        Read timeout.
     *
     * \return On success returns number of bytes read. On failure, returns
     * negatvie error code.
     */
    virtual int readline(char buf[], size_t sizeBuf, unsigned int msec=100);

    /*!
     * \brief Write a string to the backend engine.
     *
     * A newline '\n' character is written after the end of the line.
     *
     * \param [in] strLine  Output buffer write.
     * \param len           Number of bytes to write.
     *
     * \return On success returns number of bytes written. On failure, returns
     * negatvie error code.
     */
    virtual int writeline(const std::string &strLine)
    {
      return writeline(strLine.c_str(), strLine.size());
    }
  
    /*!
     * \brief Write a line to the backend engine.
     *
     * A newline '\n' character is written after the end of the line.
     *
     * \param [in] buf    Output buffer write.
     * \param len         Number of bytes to write.
     *
     * \return On success returns number of bytes written. On failure, returns
     * negatvie error code.
     */
    virtual int writeline(const char buf[], size_t len);

    /*!
     * \brief Flush all input from input pipe.
     */
    virtual void flushInput();
  
    /*!
     * \brief Test is communication is open to backend engine.
     *
     * \return Returns true or false.
     */
    virtual bool isOpen()
    {
      return m_pidChild > 0;
    }

    /*!
     * \brief Abort current read.
     */
    virtual void abortRead()
    {
      kill(m_pidParent, SIGUSR1);
    }

  protected:
    // configuration and i/o operation
    int             m_pipeToChess[2];   ///< pipe to chess engine
    int             m_pipeFromChess[2]; ///< pipe from chess engine
    pid_t           m_pidParent;        ///< parent (this) process id
    pid_t           m_pidChild;         ///< fork-exec'd engine process id
    pthread_mutex_t m_mutexIF;          ///< high-level interface mutex
    
    // version independent configuration
    int         m_nDepth;           ///< engine search ply depth (half-moves)
    bool        m_bNeedPrompting;   ///< engine does [not] need move prompting

    // version specific configuration
    bool        m_bHasRandom;       ///< has [no] random start support
    bool        m_bHasTimeLimits;   ///< has [no] annoying TimeLimit strings

    // working parsed and derived variables read from backend input

    std::string   m_strLastLine;    ///< last unmatched line read from engine
    BeParsedVars  m_parsed;         ///< last parsed values

    /*!
     * \brief Test what version of gnuchess is running.
     *
     * \return Returns true or false.
     */
    inline bool isVer5() { return m_strEngineVersion[0] == '5'; }
    inline bool isVer6() { return m_strEngineVersion[0] == '6'; }

    //
    // Mutual exclusion control.
    //
    void lock()
    {
      pthread_mutex_lock(&m_mutexIF);
      m_bIsBusy = true;
    }

    void unlock()
    {
      m_bIsBusy = false;
      pthread_mutex_unlock(&m_mutexIF);
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Initialization and Clean Up.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Configure backend engine for operation.
     */
    virtual void configure();

    /*!
     * \brief Kill backend engine process.
     */
    virtual void killApp();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Commands
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    
    /*!
     * \brief Get gnuchess's version string.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   version
     * Response:  string
     * Variables: m_strEngineVersion
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdVersion();

    /*!
     * \brief Enable xboard interface.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   xboard on
     * Response:  none
     * Variables: none
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdXboard();

    /*!
     * \brief Start a new game.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   new
     * Response:  none
     * Variables: none
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdNew();

    /*!
     * \brief Set engine's depth of search.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   depth DEPTH
     * Response:  none
     * Variables: none
     *
     * \param nDepth    Search depth.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdDepth(int nDepth);

    /*!
     * \brief Randomize engine's first moves.
     *
     * Backend gnuchess command.
     *
     * Not well supported by gnuchess.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   random
     * Response:  none
     * Variables: m_bHasRandom
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdRandom();

    /*!
     * \brief Force engine to make a move.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   go
     * Response:  none
     * Variables: none
     *
     * \param bIsFirstGo  This is [not] the first go since start of a game.
     *                    Some versions of gnuchess produce extraneous output
     *                    on the first go.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdGo(bool bIsFirstGo=false);

    /*!
     * \brief Make a player move.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   AN
     * Response:  N. AN
     * Variables: m_var.m_nMoveNum, m_var.m_strAN
     *
     * \param strAN   Move in either Coordinate or Standard Algebraic Notation.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdMove(const std::string &strAN);

    /*!
     * \brief Show the game history and retrieve move's SAN.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   show game
     * Response:  multi-line game history.
     * Variables: m_var.m_strSAN
     *
     * \param nMoveNum  Move number to search for.
     * \param eColor    Color of move to search for.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdShowGame(int nMoveNum, ChessColor color);

    /*!
     * \brief Show the game board state.
     *
     * Backend gnuchess command.
     *
     * Send command, receive response, and parse relevant values.
     *
     * Command:   show board
     * Response:  multi-line ascii board array with supplimentary info.
     * Variables: none
     *
     * \param nMove   Move number to search for.
     * \param eColor  Color of move to search for.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int cmdShowBoard(std::string &strCastling);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Responses
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    
    virtual int rspEnginesMove(ChessColor colorMove);

    virtual int rspFirstLine(boost::regex             &reRsp,
                             std::vector<std::string> &matches,
                             unsigned int             msec=100,
                             bool                     bLower=true);

    virtual int rspNextLine(boost::regex             &reRsp,
                            std::vector<std::string> &matches,
                            bool                      bLower=true);

    virtual int match(const std::string        &str,
                      boost::regex             &re,
                      std::vector<std::string> &matches);

    /*!
     * \brief Clear all parse variables.
     */
    virtual void clearParseVars();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Low-Level I/O
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Set up signal mask and handlers.
     *
     * \note Call only in main application context.
     */
    void setupSignals();

    /*!
     * \brief Wait for input from backend chess engine.
     *
     * The call is preemptable to support action server requests.
     *
     * \param msec  Maximum time to wait for input in milliseconds.
     *
     * \return
     * Returns \h_lt 0, 0, \ht_gt 0 if an I/O error, timeout, or input availble
     * event occurred, respectively.
     */
    int waitForPipe(unsigned int msec);
  };

} // chess_engine

#endif // _CE_ENGINE_H
