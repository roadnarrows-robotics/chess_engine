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
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The ROS chess_server node class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#include "chess_server/StartNewGameSvc.h"
#include "chess_server/MakeAMoveSvc.h"
#include "chess_server/MakeAMoveSANSvc.h"
#include "chess_server/GetEnginesMoveSvc.h"
#include "chess_server/ResignSvc.h"
#include "chess_server/AutoPlaySvc.h"
#include "chess_server/SetDifficultySvc.h"
#include "chess_server/GetPlayHistorySvc.h"
#include "chess_server/GetBoardStateSvc.h"

#include "chess_server/GetEnginesMoveAction.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

#include "chess_engine_gnu.h"

#define INC_ACTION_THREAD   ///< include an action thread with this class

namespace chess_engine
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

    typedef actionlib::SimpleActionServer<chess_server::GetEnginesMoveAction>
      GetEnginesMoveAS;

    ChessServer(ros::NodeHandle &nh);

    virtual ~ChessServer();

    virtual int connect(const std::string &strChessApp="gnuchess")
    {
      return m_engine.openConnection(strChessApp);
    }

    virtual int disconnect()
    {
      return m_engine.closeConnection();
    }

    virtual void advertiseServices();

    virtual void advertisePublishers(int nQueueDepth=10);

    virtual void subscribeToTopics();

    virtual uint32_t publish(uint32_t seqnum);

    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    ChessEngineGnu &getEngine()
    {
      return m_engine;
    }

    Game &getGame()
    {
      return m_game;
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

    //
    // Support
    //
    void toMsgMove(const Move &move, chess_server::ChessMoveMsg &msgMove);

  protected:
    ros::NodeHandle  &m_nh;         ///< the node handler bound to this instance

    // chess
    ChessEngineGnu    m_engine;     ///< chess backend engine
    Game              m_game;       ///< chess game state

    // ROS services, publishers, subscriptions, and local state
    MapServices       m_services;     ///< chess_server services
    MapPublishers     m_publishers;   ///< chess_server publishers
    bool              m_bPubNewGame;  ///< do [not] publish new game
    int               m_nPubLastPly;  ///< last published ply (1/2 move)
    bool              m_bPubEndGame;  ///< do [not] publish end of game

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
    bool startNewGame(chess_server::StartNewGameSvc::Request  &req,
                      chess_server::StartNewGameSvc::Response &rsp);

    bool makeAMoveSAN(chess_server::MakeAMoveSANSvc::Request  &req,
                      chess_server::MakeAMoveSANSvc::Response &rsp);

    bool makeAMove(chess_server::MakeAMoveSvc::Request  &req,
                   chess_server::MakeAMoveSvc::Response &rsp);

    bool getEnginesMove(chess_server::GetEnginesMoveSvc::Request  &req,
                        chess_server::GetEnginesMoveSvc::Response &rsp);

    bool resign(chess_server::ResignSvc::Request  &req,
                chess_server::ResignSvc::Response &rsp);

    bool autoplay(chess_server::AutoPlaySvc::Request  &req,
                  chess_server::AutoPlaySvc::Response &rsp);

    bool setDifficulty(chess_server::SetDifficultySvc::Request  &req,
                       chess_server::SetDifficultySvc::Response &rsp);

    bool getPlayHistory(chess_server::GetPlayHistorySvc::Request  &req,
                        chess_server::GetPlayHistorySvc::Response &rsp);

    bool getBoardState(chess_server::GetBoardStateSvc::Request  &req,
                       chess_server::GetBoardStateSvc::Response &rsp);

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

} // namespace chess_engine


#endif // _CHESS_SERVER_H
