////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_thread.h
//
/*! \file
 *
 * \brief The ROS chess_server chess thread class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows
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

#ifndef _CHESS_THREAD_H
#define _CHESS_THREAD_H

#include <pthread.h>

#include <string>
#include <map>

#include <boost/bind.hpp>

#include "ros/ros.h"

// chess engine library
#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

namespace chess_server
{
  /*!
   * \brief The class embodiment of the chess_server ROS node.
   */
  class ChessThread
  {
  public:

    /*!
     * \brief Thread state.
     */
    enum ThreadState
    {
      ThreadStateUninit   = 0,    ///< unitialized
      ThreadStateIdle     = 1,    ///< idle, no actions running
      ThreadStateWorking  = 2,    ///< working on a task
      ThreadStateExiting  = 3     ///< exiting
    };

    /*!
     * \brief Scheduled move task arguments structure.
     */
    struct MoveTaskArgs
    {
      chess_engine::ChessColor  m_ePlayer;          ///< player 
      std::string               m_strAN;            ///< algebraic notation 
      chess_engine::ChessPos    m_posSrc;           ///< source chess square
      chess_engine::ChessPos    m_posDst;           ///< destination square
      chess_engine::ChessPiece  m_ePiecePromoted;   ///< piece promoted
    };

    /*!
     * \brief Default constructor.
     *
     * \param chess Bound reference to the game of chess object.
     */
    ChessThread(chess_engine::Chess &chess);

    /*!
     * \brief Destructor.
     */
    virtual ~ChessThread();

    /*!
     * \brief Create the chess thread.
     *
     * The thread will immediately block waiting for a task to execute.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int createThread();

    /*!
     * \brief Destroy the chess thread.
     *
     * Thread resources are freed. Any executing task in aborted.
     */
    void destroyThread();

    /*! 
     * \brief Schedule the built-in chess move task.
     *
     * The move will be made against the chess engine, and if successful,
     * applied to the game state.
     *
     * \param ePlayer   Player (color) making the move.
     * \param strAN     Move in Coordinage/Standard Algebraic Notation.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int schedMoveToMake(const chess_engine::ChessColor ePlayer,
                        const std::string              &strAN);

    /*!
     * \brief Schedule the built-in chess move task.
     *
     * The move will be made against the chess engine, and if successful,
     * applied to the game state.
     *
     * \param ePlayer         Player (color) making the move.
     * \param posSrc          Source chess position of move.
     * \param posDst          Destination chess position of move.
     * \param ePiecePromoted  Promoted piece, if any.
     * \param [out] move      Fully qualified chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int schedMoveToMake(const chess_engine::ChessColor ePlayer,
                        const chess_engine::ChessPos   &posSrc,
                        const chess_engine::ChessPos   &posDst,
                        const chess_engine::ChessPiece ePiecePromoted);

    /*!
     * \brief Schedule the backend chess engine to compute the next move.
     *
     * If successful, applied to the game state.
     *
     * \param ePlayer Engine (color) making the move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int schedMoveToCompute(const chess_engine::ChessColor ePlayer);

    /*!
     * \brief Schedule the start execution a user-defined task.
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
    int schedUserTask(boost::function<void()> task);

    /*!
     * \brief Synchronously wait for a move to complete.
     *
     * This function can be called after any of the built-in move tasks for
     * synchronous calls (e.g. services).
     *
     * \param [out] move   Fully qualified move. 
     *
     * \return Returns last move's return code.
     * CE_OK on success, negative error code on failure.
     */
    int waitForMove(chess_engine::ChessMove &move);

    /*!
     * \brief Get last move made by this thread.
     *
     * \param [out] move   Fully qualified move. 
     *
     * \return Returns last move's return code.
     * CE_OK on success, negative error code on failure.
     */
    int getLastMove(chess_engine::ChessMove &move);

    /*!
     * \brief Test if chess thread is busy working on a task.
     *
     * \return Returns true or false.
     */
     bool isBusy()
     {
       return m_eState == ThreadStateWorking;
     }

  protected:
    chess_engine::Chess     &m_chess;       ///< the game of chess

    ThreadState             m_eState;       ///< thread state
    boost::function<void()> m_execTask;     ///< task function to execute
    pthread_mutex_t         m_mutex;        ///< mutex
    pthread_cond_t          m_cond;         ///< condition
    pthread_t               m_threadChess;  ///< pthread identifier 

    MoveTaskArgs            m_argsMove;     ///< move task input arguments
    chess_engine::ChessMove m_move;         ///< move task output 
    int                     m_rcMove;       ///< move task return code

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
      pthread_mutex_lock(&m_mutex);
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
      pthread_mutex_unlock(&m_mutex);
    }
    
    /*!
     * \brief Signal change of state.
     *
     * \param eNewState New thread state to set prior to signal.
     *
     * \par Context:
     * Calling thread or this thread.
     */
    void signalStateChange(ThreadState eNewState);

    /*!
     * \brief Wait indefinitely in idle state.
     *
     * \par Context:
     * Calling thread or background thread.
     */
    void idleWait();
    
    /*!
     * \brief Chess thread main loop.
     *
     * \param pArg  Thread argument (this).
     *
     * \return Returns NULL on exit.
     */
    static void *threadMain(void *pArg);

    /*!
     * \brief Actually schedule the task.
     *
     * No context locking is performed by this lower-level call.
     *
     * \param task  The task function to execute.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int schedTask(boost::function<void()> task);

    /*!
     * \brief Execute task to make a chess move specified in AN.
     */
    void execMakeAMoveAN();

    /*!
     * \brief Execute task to make a chess move specified by source and
     * destination move postions.
     */
    void execMakeAMove();

    /*!
     * \brief Execute task to compute engine's move.
     */
    void execComputeEnginesMove();
  };

} // namespace chess_server


#endif // _CHESS_THREAD_H
