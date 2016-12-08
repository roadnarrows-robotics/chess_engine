////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_server.h
//
/*! \file
 *
 * \brief The ROS chess_server node class interface.
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

#ifndef _CHESS_SERVER_H
#define _CHESS_SERVER_H

#include <pthread.h>

#include <string>
#include <map>

#include <boost/bind.hpp>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

// published messages
#include "chess_server/ChessNewGameStatus.h"
#include "chess_server/ChessMoveStamped.h"
#include "chess_server/ChessEndGameStatus.h"

// services
#include "chess_server/StartNewGame.h"
#include "chess_server/MakeAMove.h"
#include "chess_server/MakeAMoveAN.h"
#include "chess_server/ComputeEnginesMove.h"
#include "chess_server/MoveCompleted.h"
#include "chess_server/Resign.h"
#include "chess_server/AutoPlay.h"
#include "chess_server/SetDifficulty.h"
#include "chess_server/GetGameState.h"
#include "chess_server/GetBoardState.h"
#include "chess_server/GetPlayHistory.h"

// chess engine library
#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

// chess server
#include "chess_thread.h"

namespace chess_server
{
  /*!
   * \brief The class embodiment of the chess_server ROS node.
   */
  class ChessServer
  {
  public:

    /*!
     * \brief Structure to organize published state variables.
     */
    struct PubVars
    {
      bool                m_bPubNewGame;        ///< do [not] publish new game
      int                 m_nPubLastPly;        ///< last published ply
      bool                m_bPubEndOfGame;      ///< do [not] publish end game
      ChessNewGameStatus  m_msgNewGameStatus;   ///< new game status message
      ChessMoveStamped    m_msgMoveStamped;     ///< stamped move message
      ChessEndGameStatus  m_msgEndOfGameStatus; ///< end of game status message
    };

    /*!
     * \brief Chess move sequence state.
     *
     * \verbatim
     * [idle] --> [start] --> [result] --> [marked] --+
     *   ^                                            |
     *   |                                            |
     *   +--------------------------------------------+
     * \endverbatim
     */
    enum MoveSeqState
    {
      MoveSeqStateIdle = 0, ///< no move in progress
      MoveSeqStateStart,    ///< move started
      MoveSeqStateResult,   ///< move result received
      MoveSeqStateMarked    ///< client marked move sequence as completed
    };

    /*!
     * \brief Structure to service sequencing state variables.
     */
    struct SvcSeqVars
    {
      MoveSeqState              m_eMoveSeqState;      ///< move sequence state
      chess_engine::ChessColor  m_ePlayerMakingMove;  ///< player making move
      int                       m_nPlyNum;            ///< ply number of move
    };

    /*!
     * \brief Structure to organize autoplay state variables.
     */
    struct AutoPlayVars
    {
      bool            m_bRun;         ///< do [not] run
      int             m_nNumPlies;    ///< run for max plies, 0 to end of game
      int             m_nLastPlyNum;  ///< last autoplay ply number
      double          m_fDelay;       ///< delay between plies, 0.0 is no delay
      ros::WallTimer  m_timer;        ///< autoplay timer
      int             m_rcAutoPlay;   ///< autoplay return code
    };

    /*! map of ROS services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;
   

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Constructors, destructor, initialization start-ups.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Default constructor.
     *
     * \brief nh    ROS node handle.
     */
    ChessServer(ros::NodeHandle &nh);

    /*!
     * \brief Destructor.
     */
    virtual ~ChessServer();

    /*!
     * \brief Initialize the game of chess.
     *
     * A connection to the backend chess engine is established and a
     * chess task thread is created, blocked on the idle state.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int initialize();

    /*!
     * \brief Advertise chess server ROS services.
     *
     * \return Number of services advertised.
     */
    virtual int advertiseServices();

    /*!
     * \breif Advertise chess server ROS published topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     *
     * \return Number of published topics advertiszed.
     */
    virtual int advertisePublishers(int nQueueDepth=10);

    /*!
     * \breif Subscribe to ROS published topics.
     *
     * \return Number of topics subscribed.
     */
    virtual int subscribeToTopics();

    /*!
     * \brief Publish topics.
     */
    virtual void publish();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Asynchronous Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Asynchronously make a chess move.
     *
     * \param ePlayer Player making the move.
     * \param strAN   Move speicfied in Algebraic Notation.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int asyncMakeAMove(const chess_engine::ChessColor ePlayer,
                       const std::string              &strAN);

    /*!
     * \brief Asynchronously compute engine's next move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int asyncComputeEnginesMove();

    /*!
     * \brief Get the last asynchronous move.
     *
     * \param [out] move  Fully qualified chess move.
     *
     * \return Returns last move results. CE_OK on success, negative error code
     * on failure.
     */
    int asyncGetLastMove(chess_engine::ChessMove &move);

    /*!
     * \brief Schedule user-defined task to execute by the chess thread.
     *
     * This function does not block, but rather schedules the thread to execute
     * the user-defined task.
     *
     * The provided function task() should not return until its tasks are
     * completed.
     *
     * The function task() can be a class member fucntion. For example:
     * \verbatim
     *  rc = th.execAction(boost::bind(&MyClass::doit, this));
     *
     *  where:
     *    th   is an instance of ChessThread class,
     *    doit is a member function of MyClass,
     *    this is an instance of class MyClass.   
     * \endverbatim
     *
     * \param task  The user-defined function to execute.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int schedUserTask(boost::function<void()> task)
    {
      return m_threadTask.schedUserTask(task);
    }

    /*!
     * \brief Begin autoplay move sequence.
     *
     * \param nNumMoves Max number of auto-moves to make (0 == end of game).
     * \param fDelay    Delay in seconds between move plies (0.0 = no delay).
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int beginAutoPlay(int nNumPlies, double fDelay);

    /*!
     * \brief Test if autoplay is running.
     *
     * \return Returns true or false.
     */
    bool isInAutoPlay();

    /*!
     * \brief End autoplay.
     *
     * \param strReason   Reason why autoplay terminated (for logging).
     * \param nReasonCode Termination reason code (\ref see ce_ecodes).
     */
    void endAutoPlay(const std::string &strReason  = "done",
                     const int         nReasonCode = chess_engine::CE_OK);

    /*!
     * \brief Get autoplay termination reason code.
     *
     * \return Termination reason code (\ref see ce_ecodes).
     */
    int getAutoPlayReasonCode();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Test if a game is currently being played.
     *
     * \return Returns true or false.
     */
    bool isPlayingAGame() const
    {
      return m_chess.isPlayingAGame();
    }

    /*!
     * \brief Get whose turn is it to move.
     *
     * \return Player's color
     */
    virtual chess_engine::ChessColor whoseTurn() const
    {
      return m_chess.whoseTurn();
    }

    /*!
     * \brief Test if a move is in progress.
     *
     * \return Returns true or false.
     */
    bool isMoveInProgress()
    {
      return isMoveSvcSeqBusy() || m_threadTask.isBusy();
    }

    /*!
     * \brief Get the number of plies (half-moves) played.
     *
     * \return Number of plies.
     */
    int getNumOfPliesPlayed() const
    {
      return m_chess.getNumOfPliesPlayed();
    }

    /*!
     * \brief Get the number of completed moves played.
     *
     * One completed move is White then Black, with White always starting.
     *
     * \return Number of moves.
     */
    int getNumOfMovesPlayed() const
    {
      return m_chess.getNumOfMovesPlayed();
    }

    /*!
     * \brief Get ROS node handle.
     *
     * \brief Handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

  protected:
    ros::NodeHandle  &m_nh;         ///< the node handler bound to this instance

    // chess
    chess_engine::Chess m_chess;    ///< the game of chess

    // ROS services, publishers, subscriptions, and local state
    MapServices   m_services;       ///< chess_server services
    MapPublishers m_publishers;     ///< chess_server publishers

    PubVars       m_pub;            ///< publish state 
    SvcSeqVars    m_svcseq;         ///< service sequencing
    AutoPlayVars  m_autoplay;       ///< autoplay state

    // working threads
    ChessThread   m_threadTask;     ///< chess asynchronous task thread 


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Service callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Start a new chess game service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool startNewGame(StartNewGame::Request  &req,
                      StartNewGame::Response &rsp);

    /*!
     * \brief Make a chess move service.
     *
     * Move is in Algebraic Notation.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool makeAMoveAN(MakeAMoveAN::Request  &req,
                     MakeAMoveAN::Response &rsp);

    /*!
     * \brief Make a chess move service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool makeAMove(MakeAMove::Request  &req,
                   MakeAMove::Response &rsp);

    /*!
     * \brief Compute chess engine's move service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool computeEnginesMove(ComputeEnginesMove::Request  &req,
                            ComputeEnginesMove::Response &rsp);

    /*!
     * \brief Mark client's move as completed service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool markMoveCompleted(MoveCompleted::Request  &req,
                           MoveCompleted::Response &rsp);

    /*!
     * \brief Resign from game service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool resign(Resign::Request  &req,
                Resign::Response &rsp);

    /*!
     * \brief Auto-play service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool autoplay(AutoPlay::Request  &req,
                  AutoPlay::Response &rsp);

    /*!
     * \brief Set engine's difficulty level service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool setDifficulty(SetDifficulty::Request  &req,
                       SetDifficulty::Response &rsp);

    /*!
     * \brief Get game state service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool getGameState(GetGameState::Request  &req,
                      GetGameState::Response &rsp);

    /*!
     * \brief Get board state service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool getBoardState(GetBoardState::Request  &req,
                       GetBoardState::Response &rsp);

    /*!
     * \brief Get play history service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool getPlayHistory(GetPlayHistory::Request  &req,
                        GetPlayHistory::Response &rsp);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Publisher State
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Initialize publish state variables.
     *
     * \param bNewGame  Is [not] start of a new game.
     */
    void initPubVars(const bool bNewGame = false);
    
    /*!
     * \brief Stamp message header.
     *
     * \param [out] header    Message header.
     * \param nSeqNum         Message header sequence number.
     * \param strFrameId      Message header frame id.
     */
    void stampHeader(std_msgs::Header  &header,
                     uint32_t          nSeqNum = 0,
                     const std::string &strFrameId= "0");


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Service Sequencing States
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Initialize all service sequencing state variables.
     */
    void initSvcSeq();

    /*!
     * \brief Begin move service sequence.
     *
     * A move service sequence is:
     *  - MakeAMove(AN) or ComputeEnginesMove
     *  - MoveCompleted 
     *
     * \param ePlayer Player making the move.
     */
    void beginMoveSvcSeq(const chess_engine::ChessColor ePlayer);

    /*!
     * \brief Mark move service sequence state.
     *
     * \param eState  New move servce sequence state.
     */
    void markMoveSvcSeq(const MoveSeqState eState);

    /*!
     * \brief End move service sequence.
     *
     * \param bAbort  Do [not] force abort of current move sequence.
     */
    void endMoveSvcSeq(bool bAbort = false);

    /*!
     * \brief Test if move sequence is in progress.
     *
     * \return Returns true or false.
     */
    bool isMoveSvcSeqBusy()
    {
      return m_svcseq.m_eMoveSeqState != MoveSeqStateIdle;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Autoplay Support
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Initialize autoplay state variables.
     */
    void initAutoPlay();

    /*!
     * \brief Execute autoplay move.
     */
    void execAutoPlay();

    /*!
     * \brief Autoplay timer callback
     *
     * \param event Timer event.
     */
    void cbAutoPlay(const ros::WallTimerEvent& event);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Move Execution Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Execute a chess move.
     *
     * \param strEvent        The move event name or description (for logging).
     * \param [in,out] move   On input, the move contains sufficient information
     *                        to make a chess move. If synchronous, move is
     *                        fully qualified.
     * \param bSynchronous    The move is [not] synchronous.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int makeAMove(const std::string       &strEvent,
                  chess_engine::ChessMove &move,
                  bool                    bSynchronous = true);

    /*!
     * \brief Compute engines move.
     *
     * \param strEvent        The move event name or description (for logging).
     * \param [in,out] move   On input, the move contains sufficient information
     *                        to compute a chess move. If synchronous, move is
     *                        fully qualified.
     * \param bSynchronous    The move is [not] synchronous.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int computeEnginesMove(const std::string       &strEvent,
                           chess_engine::ChessMove &move,
                           bool                    bSynchronous = true);
  };

} // namespace chess_server


#endif // _CHESS_SERVER_H
