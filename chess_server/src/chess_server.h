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

#include "chess_server/ChessNewGameStatus.h"
#include "chess_server/ChessMoveStamped.h"
#include "chess_server/ChessEndGameStatus.h"

#include "chess_server/StartNewGame.h"
#include "chess_server/MakeAMove.h"
#include "chess_server/MakeAMoveAN.h"
#include "chess_server/ComputeEnginesMove.h"
#include "chess_server/Resign.h"
#include "chess_server/AutoPlay.h"
#include "chess_server/SetDifficulty.h"
#include "chess_server/GetPlayHistory.h"
#include "chess_server/GetGameState.h"

#include "chess_server/GetEnginesMoveAction.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"


#define INC_ACTION_THREAD   ///< include an action thread with this class

namespace chess_server
{
  /*!
   * \brief The class embodiment of the chess_server ROS node.
   */
  class ChessServer
  {
  public:
    /*!
     * \brief Asynchronous task state.
     */
    enum ActionState
    {
      ActionStateExit     = 0,    ///< exit(ing)
      ActionStateIdle     = 1,    ///< idle, no actions running
      ActionStateWorking  = 2     ///< action(s) running
    };

    /*! map type of ROS services */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map type of ROS publishers */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

#ifdef INC_ACTION_THREAD
    typedef actionlib::SimpleActionServer<chess_server::GetEnginesMoveAction>
                    GetEnginesMoveAS;
#endif // INC_ACTION_THREAD

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
     * A connection to the backend chess engine is established.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    virtual int initializeChess();

    /*!
     * \brief Advertise chess server ROS services.
     */
    virtual void advertiseServices();

    /*!
     * \breif Advertise chess server ROS published topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \breif Subscribe to ROS published topics.
     */
    virtual void subscribeToTopics();

    /*!
     * \brief Publish topics.
     */
    virtual void publish();

    /*!
     * \brief Get ROS node handle.
     *
     * \brief Handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

#ifdef INC_ACTION_THREAD
    /*!
     * \brief Start execution a (component) an action server's action in the
     * action thread.
     *
     * This function does not block.
     *
     * \param execAction  Function to execute. Should not return until its
     *                    action is finished. The function can be a class
     *                    member fucntion. For example:\n
     *                    rc = x.execAction(boost::bind(&AS::thExec, this));\n
     *                    where:\n
     *                    x is an instance of this class and this is an instance
     *                    of class AS.   
     *
     * \return
     * Returns CE_OK of success, \<0 on failure.
     */
     int execAction( boost::function<void()> execAction);
#endif // INC_ACTION_THREAD

  protected:
    ros::NodeHandle  &m_nh;         ///< the node handler bound to this instance

    // chess
    chess_engine::Chess m_chess;    ///< the game of chess

    // ROS services, publishers, subscriptions, and local state
    MapServices   m_services;       ///< chess_server services
    MapPublishers m_publishers;     ///< chess_server publishers
    bool          m_bPubNewGame;    ///< do [not] publish new game
    int           m_nPubLastPly;    ///< last published ply (1/2 move)
    bool          m_bPubEndOfGame;  ///< do [not] publish end of game

    // Messages for published data.
    ChessNewGameStatus  m_msgNewGameStatus;
    ChessMoveStamped    m_msgMoveStamped;
    ChessEndGameStatus  m_msgEndOfGameStatus;
    
    // action thread control
#ifdef INC_ACTION_THREAD
    ActionState             m_eActionState; ///< action task state
    boost::function<void()> m_execAction;   ///< function to execute
    pthread_mutex_t         m_mutexAction;  ///< mutex
    pthread_cond_t          m_condAction;   ///< condition
    pthread_t               m_threadAction; ///< action pthread identifier 
#endif // INC_ACTION_THREAD

    //
    // Service callbacks
    //

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
     * \brief Get play history service.
     *
     * \param req Reqquest message.
     * \param rsp Response message.
     *
     * \return Returns true (success) or false (failure).
     */
    bool getPlayHistory(GetPlayHistory::Request  &req,
                        GetPlayHistory::Response &rsp);

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
     * \brief Stamp message header.
     *
     * \param [out] header    Message header.
     * \param nSeqNum         Message header sequence number.
     * \param strFrameId      Message header frame id.
     */
    void stampHeader(std_msgs::Header  &header,
                     uint32_t          nSeqNum = 0,
                     const std::string &strFrameId= "0");

    //
    // Action (and service) thread
    //
#ifdef INC_ACTION_THREAD
    int createActionThread();

    void destroyActionThread();

    /*!
     * \brief Lock the action thread.
     *
     * The calling thread will block while waiting for the mutex to become 
     * available. Once locked, the action thread will block.
     *
     * The lock()/unlock() primitives provide a safe mechanism to modify state. 
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutexAction);
    }

    /*!
     * \brief Unlock the action thread.
     *
     * The action thread will be available to run.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutexAction);
    }
    
    /*!
     * \brief Signal action thread of change of state.
     *
     * \par Context:
     * Calling thread or background thread.
     */
    void signalActionThread();

    /*!
     * \brief Wait indefinitely in idle state.
     *
     * \par Context:
     * Calling thread or background thread.
     */
    void idleWait();
    
    static void *actionThread(void *pArg);
#endif // INC_ACTION_THREAD
  };

} // namespace chess_server


#endif // _CHESS_SERVER_H
