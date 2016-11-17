////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceEngine.cpp
//
/*! \file
 *
 * \brief The GNU chess gnuchess backend engine implimentation.
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

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <signal.h>
#include <libgen.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <pthread.h>

#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include <ros/console.h>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceEngine.h"

using namespace std;
using namespace chess_engine;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// GNU Chess Tracing Macros
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#undef GC_TRACE_ENABLE   ///< tracing on/off switch

#ifdef GC_TRACE_ENABLE

#define GC_TRACE_FILE "gctrace.log"   ///< tracing output file name

static FILE *FpGcTrace = NULL;        ///< tracing output file pointer

/*!
 * \brief
 */
#define GC_TRACE(fmt, ...) \
  do \
  { \
    if( FpGcTrace != NULL )\
    { \
      fprintf(FpGcTrace, fmt, ##__VA_ARGS__); \
      fflush(FpGcTrace); \
    } \
  } while(0)

/*!
 * \brief
 */
#define GC_TRACE_OPEN() \
  do \
  { \
    if( (FpGcTrace = fopen(GC_TRACE_FILE, "w")) != NULL ) \
    { \
      GC_TRACE("### Start GNU Chess (GC) Tracing.\n\n"); \
    } \
  } while(0)

/*!
 * \brief
 */
#define GC_TRACE_CLOSE() \
  do \
  { \
    if( FpGcTrace != NULL )\
    { \
      fclose(FpGcTrace); \
      GC_TRACE("### End GNU Chess Backend (GC) Tracing.\n\n"); \
    } \
  } while(0)

#else

#define GC_TRACE(fmt, ...)
#define GC_TRACE_OPEN()
#define GC_TRACE_CLOSE()

#endif // GC_TRACE_ENABLE


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Error Handling Macros
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Macro to try an expression.
 *
 * On failure, log error, unlock mutex, and return from function.
 *
 * Must be used within the ChessEngine derived classes with unlock() defined.
 *
 * \param expr  Express to evaluate.
 * \param ecode Function return error code on failure.
 * \param fmt   Logging format string.
 * \param ...   Option specific variable arguments for fmt.
 */
#define ENGINE_TRY(expr, ecode, fmt, ...) \
  do \
  { \
    if( !(expr) ) \
    { \
      ROS_ERROR(fmt, ##__VA_ARGS__); \
      unlock(); \
      return (ecode) < 0? (ecode): -(ecode); \
    } \
  } while(0)


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Response regualar expressions.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! null response */
static boost::regex reNull("(^$)");

/*! any response */
static boost::regex reAny("(.*)");

/*! pre-response: 'timelime...' (for version >= 6) */
static boost::regex reTimeLimit("(^timelimit.*)");

/*! 'version' response: 'gnu chess VERSION' */
static boost::regex reVersion("^gnu chess (.*)");

/*! 'depth DEPTH' response: 'search to a depth of DEPTH' */
static boost::regex reDepth("^search to a depth of ([0-9]+)");

/*! engine's numbered move response: 'N. ... AN' */
static boost::regex reNumMoveEngine("^([0-9]+)\\.\\s+\\.\\.\\.\\s+(\\S+)");

/*! players's numbered move response: 'N. AN' */
static boost::regex reNumMovePlayer("^([0-9]+)\\.\\s+(\\S+)");

/*! engines move response: 'my move is : SAN' */
static boost::regex reEngineMove("^my move is\\s*:\\s*"
                                 "([a-h][1-8][a-h][1-8]\\S*)");

/*! engine resigns: 'resign ...' */
static boost::regex reResign("(resign).*");

/*! engine determines a draw: '1/2-1/2 ...' */
static boost::regex reDraw("(1/2-1/2).*");

/*! white checkmates: '1-0 ...' */
static boost::regex reWhiteWins("(1-0).*");

/*! black checkmates: '0-1 ...' */
static boost::regex reBlackWins("(0-1).*");

/*! 'show game' response header: 'white black ...' */
static boost::regex reWhiteBlackHdr("\\s*(white)\\s+(black)\\s*");

/*! 'show game' response 1 move: 'N. AN' */
static boost::regex reWhiteBlackMove1("\\s*([0-9]+)\\.\\s+(\\S+)\\s*");

/*! 'show game' response 2 moves: 'N. AN AN' */
static boost::regex reWhiteBlackMove2("\\s*([0-9]+)\\.\\s+"
                                      "(\\S+)\\s+(\\S+)\\s*");

/*! 'show board' response status line: 'COLOR [CASTLING] ...' */
static boost::regex reCastling("\\s*(\\S+)\\s+([KQkq]*)\\s*");

/*! bad move response: 'invalid move: SAN' or 'illegal move: SAN' */
static boost::regex reInvalidMove("^(invalid move:|illegal move:)\\s+(.*)");

/*! unsupported command response: 'command 'CMD' is currently not supported.' */
static boost::regex reNotSupported("^command '(.*)' "
                                   "is currently not supported.");


//
// Turn off annoying warning with some compilers
//
//PRAGMA_IGNORED(sign-conversion)
/*!
 * \brief FD_SET() wrapper with no annoying warnings.
 * \param fd    File descriptor to add to set.
 * \param pset  Pointer to fd set.
 */
static inline void fdset_nowarn(int fd, fd_set *pset)
{
  FD_SET(fd, pset);
}
//PRAGMA_WARNING(sign-conversion)


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Timing Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system seconds and useconds.
 */
static inline void timer_mark(struct timeval *pTvMark)
{
  if( gettimeofday(pTvMark, NULL) != 0 )
  {
    ROS_ERROR("gettimeofday() %s(errno=%d)", strerror(errno), errno);
    timerclear(pTvMark);
  }
}

/*! 
 * \brief Calculate the elapsed time between the given time mark and this call.
 *
 * \param pTvMark   Pointer to timeval holding time mark.
 *
 * \return 
 * Number of microseconds elasped. If the marked time is invalid or the current
 * time cannot be ascertained, UINT_MAX is returned.
 */
unsigned int timer_elapsed(struct timeval *pTvMark)
{
  struct timeval  tvEnd, tvDelta;

  timer_mark(&tvEnd);

  if( !timerisset(pTvMark) || !timerisset(&tvEnd) )
  {
    return UINT_MAX;
  }

  tvDelta.tv_sec = tvEnd.tv_sec - pTvMark->tv_sec;
  if( tvEnd.tv_usec < pTvMark->tv_usec )
  {
    tvDelta.tv_sec--;
    tvEnd.tv_usec += 1000000;
  }
  tvDelta.tv_usec = tvEnd.tv_usec - pTvMark->tv_usec;

  return (unsigned int)(tvDelta.tv_sec * 1000000 + tvDelta.tv_usec);
}

static void setTs(int64_t nsec, timespec &ts)
{
  ts.tv_sec  = (time_t)(nsec / 1000000000);
  ts.tv_nsec = (time_t)(nsec % 1000000000);
}

static bool AbortFlagged = false;

static void abort_handler(int sig)
{
  //fprintf(stderr, "abort_handler(%d)\n", sig);
  AbortFlagged = true;
}


// -----------------------------------------------------------------------------
// Class ChessEngine
// -----------------------------------------------------------------------------

ChessEngine::ChessEngine(string strEngineName) :
    m_strEngineName(strEngineName),
    m_strEngineVersion("0.0.0")
{
  m_bIsConn     = false;
  m_bIsPlaying  = false;
  m_bIsBusy     = false;
  m_fDifficulty = 1.0; 
  m_eColorTurn  = NoColor;
  m_nNumMoves   = 0;
  m_eEoGReason  = NoGame;
  m_eWinner     = NoColor;
}

ChessEngine::~ChessEngine()
{
}

int ChessEngine::setGameDifficulty(float fDifficulty)
{
  // normalize
  if( fDifficulty < 1.0 )
  {
    fDifficulty = 1.0;
  }
  else if( fDifficulty > 10.0 )
  {
    fDifficulty = 10.0;
  }

  m_fDifficulty = fDifficulty;

  return CE_OK;
}

int ChessEngine::startNewGame()
{
  if( isConnected() )
  {
    m_bIsPlaying  = true;
    m_eColorTurn  = White;
    m_nNumMoves   = 0;
    m_eEoGReason  = Ok;
    m_eWinner     = NoColor;

    return CE_OK;
  }

  return -CE_ECODE_NO_EXEC;
}

int ChessEngine::endCurrentGame(ChessResult eReason, ChessColor eWinner)
{
  if( isPlayingAGame() )
  {
    m_bIsPlaying  = false;
    m_eColorTurn  = NoColor;
    m_eEoGReason  = eReason;
    m_eWinner     = eWinner;
  }

  return CE_OK;
}

int ChessEngine::resign(const ChessColor ePlayer)
{
  return endCurrentGame(Resign, opponent(ePlayer));
}

ChessColor ChessEngine::alternateTurns(const ChessColor ePlayerLast)
{
  if( isPlayingAGame() )
  {
    if( ePlayerLast == White )
    {
      m_eColorTurn = Black;
    }
    else
    {
      m_eColorTurn = White;
      ++m_nNumMoves;
    }
  }

  return m_eColorTurn;
}


// -----------------------------------------------------------------------------
// Class ChessEngineGnu
// -----------------------------------------------------------------------------

ChessEngineGnu::ChessEngineGnu() : ChessEngine("gnuchess")
{
  m_pipeToChess[0]    = -1;
  m_pipeToChess[1]    = -1;
  m_pipeFromChess[0]  = -1;
  m_pipeFromChess[1]  = -1;
  m_pidParent         = getpid();
  m_pidChild          = -1;

  m_nDepth            = DepthDft;
  m_bNeedPrompting    = false;

  m_bHasRandom        = true;
  m_bHasTimeLimits    = true;

  pthread_mutex_init(&m_mutexIF, NULL);
}

ChessEngineGnu::~ChessEngineGnu()
{
  closeConnection();

  pthread_mutex_destroy(&m_mutexIF);
}
  

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Base Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessEngineGnu::openConnection()
{
  char   *argv[8];

  // make chess server node to chess engine backend pipe
  if( pipe(m_pipeToChess) == -1 )
  {
    ROS_ERROR("Failed to create pipe to %s. %s(errno=%d)",
        m_strEngineName.c_str(), strerror(errno), errno);
    return -CE_ECODE_SYS;
   }

  // make chess engine backend to chess server node pipe
  if ( pipe(m_pipeFromChess) == -1 )
  {
    ROS_ERROR("Failed to create pipe from %s. %s(errno=%d)",
        m_strEngineName.c_str(), strerror(errno), errno);
    return -CE_ECODE_SYS;
  }

  // fork calling application
  m_pidChild = fork();

  // fork failed
  if( m_pidChild == -1 )
  {
    ROS_ERROR("Could not fork. %s(errno=%d)", strerror(errno), errno);
    return -CE_ECODE_SYS;
  }

  //
  // Child Process
  //
  if( m_pidChild == 0 )
  {
    // close unused pipe ends 
    close(m_pipeToChess[1]);
    close(m_pipeFromChess[0]);

    // duplciate pipe ends to standard input and output
    dup2(m_pipeToChess[0], 0);
    dup2(m_pipeFromChess[1], 1);

    // build exec arguments
    argv[0] = (char *)m_strEngineName.c_str();
    //argv[1] = (char *)"--xboard";
    argv[1] = NULL;
    argv[2] = NULL;

    // exec chess application
    execvp(argv[0], argv);

    // if here then exec failed
    exit(1);
  }

  //
  // Parent Process
  //
  else
  {
    ROS_INFO("Forked-exec'ed %s, child pid=%u.",
        m_strEngineName.c_str(), m_pidChild);

    // close unused pipe ends 
    close(m_pipeToChess[0]);
    close(m_pipeFromChess[1]);

    if( fcntl(m_pipeFromChess[0], F_SETFL, O_NONBLOCK) < 0 )
    {
      ROS_ERROR("fcntl(%d,F_SETFL,O_NONBLOCK) %s(errno=%d)",
          m_pipeFromChess[0], strerror(errno), errno);
      return -CE_ECODE_SYS;
    }

    GC_TRACE_OPEN();
    GC_TRACE("Application: %s\n", m_strEngineName.c_str());

    sleep(1);

    configure();

    m_bIsConn     = true;
    m_bIsPlaying  = false;

    return CE_OK;
  }
}

int ChessEngineGnu::closeConnection()
{
  if( m_pidChild > 0 )
  {
    killApp();
  }

  // close pipe ends 
  if( m_pipeToChess[1] >= 0 )
  {
    close(m_pipeToChess[1]);
  }

  if( m_pipeFromChess[0] >=  0 )
  {
    close(m_pipeFromChess[0]);
  }

  m_strEngineName.clear();

  m_pipeToChess[0]    = -1;
  m_pipeToChess[1]    = -1;
  m_pipeFromChess[0]  = -1;
  m_pipeFromChess[1]  = -1;
  m_pidChild          = -1;

  GC_TRACE("closeConnection()\n");

  m_bIsConn     = false;
  m_bIsPlaying  = false;
  m_bIsBusy     = false;

  return CE_OK;
}

int ChessEngineGnu::setGameDifficulty(float fDifficulty)
{
  int   rc;

  lock();

  ChessEngine::setGameDifficulty(fDifficulty);

  m_nDepth = (int)(DepthDifficulty * m_fDifficulty);

  // ok to delay setting engine's search depth 
  if( isConnected() )
  {
    rc = cmdDepth(m_nDepth);
  }
  else
  {
    rc = CE_OK;
  }

  unlock();

  return rc;
}

int ChessEngineGnu::startNewGame()
{
  int   rc;

  lock();

  ENGINE_TRY(isConnected(), CE_ECODE_NO_EXEC, "Not connected.");

  flushInput();

  if( (rc = cmdNew()) == CE_OK )
  {
    cmdDepth(m_nDepth);
    //cmdRandom(); // N.B. not supported in v6 and poorly in v5
    ChessEngine::startNewGame();
  }

  m_bNeedPrompting = true;

  unlock();

  return rc;
}

int ChessEngineGnu::makePlayersMove(const ChessColor ePlayer, string &strAN)
{
  int     nNextMoveNum;
  int     rc;

  lock();

  //
  // Pre-checks
  //
  ENGINE_TRY(isConnected(), CE_ECODE_NO_EXEC, "Not connected.");
  ENGINE_TRY(isPlayingAGame(), CE_ECODE_CHESS_NO_GAME, "No game.");
  ENGINE_TRY(whoseTurn() == ePlayer, CE_ECODE_CHESS_OUT_OF_TURN,
                                              "Move out-of-turn.");

  // next move number
  nNextMoveNum = m_nNumMoves + 1;

  // clear old parsed variables
  clearParseVars();

  // send move command
  rc = cmdMove(strAN);

  ENGINE_TRY(rc == CE_OK, rc, "Player's move command failed.");

  // out-of-sync with chess engine
  ENGINE_TRY(m_var.m_nMoveNum == nNextMoveNum, CE_ECODE_CHESS_FATAL,
      "Response: Move %d != expected move %d.", m_var.m_nMoveNum, nNextMoveNum);

  // the move in, out should match, but try to continue
  if( m_var.m_strAN != strAN )
  {
    ROS_WARN("Response to move: '%s' != '%s'.",
        m_var.m_strAN.c_str(), strAN.c_str());
  }

  // get SAN
  rc = cmdShowGame(nNextMoveNum, ePlayer);

  ENGINE_TRY(rc == CE_OK, CE_ECODE_CHESS_FATAL, "Show game command failed.");

  // copy SAN
  strAN = m_var.m_strSAN;

  // alternate turns
  alternateTurns(ePlayer);

  // chess engine's move response is automatic after player makes a move
  m_bNeedPrompting = false;

  unlock();

  return CE_OK;
}

int ChessEngineGnu::computeEnginesMove(ChessColor &eMoveColor, string &strSAN)
{
  int     nNextMoveNum;
  int     rc;

  lock();

  //
  // Pre-checks
  //
  ENGINE_TRY(isConnected(), CE_ECODE_NO_EXEC, "Not connected.");
  ENGINE_TRY(isPlayingAGame(), CE_ECODE_CHESS_NO_GAME, "No game.");

  // engine's color this round
  eMoveColor = whoseTurn();

  // next move number
  nNextMoveNum = m_nNumMoves + 1;

  // tell engine to generate next move, its automatic after a player's move
  if( m_bNeedPrompting )
  {
    cmdGo( ((nNextMoveNum == 1) && (eMoveColor == White)) );
  }

  // clear old parsed variables
  clearParseVars();

  // wait for move response from engine
  rc = rspEnginesMove(eMoveColor);

  ENGINE_TRY(rc == CE_OK, rc, "Engine's move failed.");

  // out-of-sync with chess engine
  ENGINE_TRY(m_var.m_nMoveNum == nNextMoveNum, CE_ECODE_CHESS_FATAL,
      "Response: Move %d != expected move %d.", m_var.m_nMoveNum, nNextMoveNum);

  // get SAN
  rc = cmdShowGame(nNextMoveNum, eMoveColor);

  ENGINE_TRY(rc == CE_OK, CE_ECODE_CHESS_FATAL, "Show game command failed.");

  // copy SAN
  strSAN = m_var.m_strSAN;

  // alternate turns
  alternateTurns(eMoveColor);

  // chess engine's move is not automatic after it makes a move
  m_bNeedPrompting = true;

  unlock();

  return rc;
}

int ChessEngineGnu::getCastlingOptions(list_of_castling &availWhite,
                                       list_of_castling &availBlack)
{
  string  strOptions;
  int     rc;

  lock();

  availWhite.clear();
  availBlack.clear();

  //
  // Pre-checks
  //
  ENGINE_TRY(isConnected(), CE_ECODE_NO_EXEC, "Not connected.");
  ENGINE_TRY(isPlayingAGame(), CE_ECODE_CHESS_NO_GAME, "No game.");

  if( (rc = cmdShowBoard(strOptions)) == CE_OK )
  {
    for(size_t i=0; i<strOptions.size(); ++i)
    {
      switch( strOptions[i] )
      {
        // white
        case 'K':
          availWhite.push_back(KingSide);
        case 'Q':
          availWhite.push_back(QueenSide);
          break;
        // black
        case 'k':
          availBlack.push_back(KingSide);
        case 'q':
          availBlack.push_back(QueenSide);
          break;
        // ignore
        default:
          break;
      }
    }
  }

  unlock();

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Extended Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ChessResult ChessEngineGnu::rcToMoveResult(int rc)
{
  if( rc < 0 )
  {
    rc = -rc;
  }

  switch( rc )
  {
    case CE_OK:                       // good move
      return Ok;
    case CE_ECODE_CHESS_MOVE:         // invalid move 
    case CE_ECODE_CHESS_PARSE:
      return BadMove;
    case CE_ECODE_CHESS_OUT_OF_TURN:  // player out of turn
      return OutOfTurn;
    case CE_ECODE_CHESS_NO_GAME:      // no game in progress
      return NoGame;
    default:                          // fatal game errors
      return GameFatal;
  }
}

int ChessEngineGnu::getEnginesEoGDecl(ChessResult &eResult, ChessColor &eWinner)
{
  eResult = m_var.m_eEoGResult;
  eWinner = m_var.m_eEoGWinner;

  return CE_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Low-Level Public I/O Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessEngineGnu::readline(string &strLine, unsigned int msec)
{
  char  buf[MaxLineSize];
  int   n;

  strLine.clear();

  if( (n = readline(buf, sizeof(buf), msec)) > 0 )
  {
    strLine = buf;
  }

  return n;
}

int ChessEngineGnu::readline(char buf[], size_t sizeBuf, unsigned int msec)
{
  int             fd = m_pipeFromChess[0];
  int             nFd;
  ssize_t         n;
  ssize_t         nBytes = 0;
  bool            bDone = false;

  buf[0] = 0;

  //
  // Read the data until either 1) count bytes are read, 2) a time out occurs,
  // or 3) an error occurs.
  //
  while( (nBytes < sizeBuf-1) && !bDone )
  {
    nFd = waitForPipe(msec);
    
    // system error occurred on select - interpret
    if( nFd < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout, retry
          break;
        case EINTR:         // read was interrupted
          buf[0] = 0;
          return -CE_ECODE_SYS;
        default:            // non-recoverable error
          buf[nBytes] = 0;
          ROS_ERROR("select(%d,...) %s(errno=%d)", fd, strerror(errno), errno);
          GC_TRACE("readline() %s\n", buf);
          return -CE_ECODE_SYS;
      }
    }

    // timeout occurred
    else if( nFd == 0 )
    {
      ROS_DEBUG("select() on read timed out.");
      bDone = true;
    }

    // read the available data from the pipe
    else if( (n = read(fd, buf+nBytes, (size_t)1)) == 1 )
    {
      switch( buf[nBytes] )
      {
        case '\n':    // received a response line
          buf[nBytes] = 0;
          //ROS_DEBUG("readline() %zd bytes response line.", nBytes);
          GC_TRACE("readline() %s\n", buf);
          return (int)nBytes;
        case '\r':    // received a carriage return, but ignore
          break;
        default:      // received a character
          // got some data
          nBytes += 1;
          break;
      }
    }

    // system error occurred on read - interpret
    else if( n < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout, try again
        case EINTR:         // read was interrupted, try again
          break;
        default:            // non-recoverable error
          buf[nBytes] = 0;
          ROS_ERROR("select(%d,...) %s(errno=%d)", fd, strerror(errno), errno);
          GC_TRACE("readline() %s\n", buf);
          return -CE_ECODE_SYS;
      }
    }

    // got a select but no bytes 
    else if( n == 0 )
    {
      ROS_DEBUG("select() ok, but read()=0.");
      bDone = true;
    }
  }

  ROS_DEBUG("readline(): %zd bytes partial response line.", nBytes);
  
  buf[nBytes] = 0;
  GC_TRACE("readline() %s\n", buf);

  return -CE_ECODE_TIMEDOUT;
}

int ChessEngineGnu::writeline(const char buf[], size_t len)
{
  ssize_t m, n;

  m = len > 0?  write(m_pipeToChess[1], buf, len): 0;
  n = write(m_pipeToChess[1], "\n", 1);

  GC_TRACE("writeline() %*s\n", (int)len, buf);

  return (m >= 0) && (n > 0)? m+n: -CE_ECODE_SYS;
}

void ChessEngineGnu::flushInput()
{
  char    buf[256];

  GC_TRACE(">> flushInput() Begin\n", buf);
  m_strLastLine.clear();
  while( readline(buf, sizeof(buf)) > 0 )
  {
    //GC_TRACE("%s\n", buf);
  }
  GC_TRACE(">> flushInput() End\n");
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Initialization and Clean Up Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessEngineGnu::configure()
{
  setupSignals();

  flushInput();

  // determine version
  cmdVersion();

  GC_TRACE("GNU Chess Engine %s\n", m_strEngineVersion.c_str());

  // version 5.y.z
  if( isVer5() )
  {
    m_bHasTimeLimits  = false;
    m_bHasRandom      = true;
  }
  // version 6.y.z
  else if( isVer6() )
  {
    m_bHasTimeLimits  = true;
    m_bHasRandom      = false;
  }
  // version ?
  else
  {
    ROS_WARN("GNU Chess %s unsupported. Assuming version 6.x protocol.",
        m_strEngineVersion.c_str());
    m_bHasTimeLimits  = true;
    m_bHasRandom      = false;
  }

  // place in xboard
  cmdXboard();

  return;
}

void ChessEngineGnu::killApp()
{
  int   status;

  kill(m_pidChild, SIGTERM);
  waitpid(m_pidChild, &status, WEXITED);

  if( WIFEXITED(status) )
  {
    //fprintf(stderr, "DBG: good exit\n");
  }

  m_pidChild = -1;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Commands
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessEngineGnu::cmdVersion()
{
  static const char *cmd = "version";

  vector<string>  matches;
  int             n;

  writeline(cmd);

  if( rspFirstLine(reVersion, matches) == 1 )
  {
    m_strEngineVersion = matches[0];
    return CE_OK;
  }
  else
  {
    return -CE_ECODE_CHESS_RSP;
  }
}

int ChessEngineGnu::cmdXboard()
{
  static const char *cmd = "xboard on";

  vector<string>  matches;
  int             n;

  writeline(cmd);

  flushInput();

  return CE_OK;
}

int ChessEngineGnu::cmdNew()
{
  static const char *cmd = "new";

  vector<string>  matches;
  int             n;

  writeline(cmd);

  rspFirstLine(reAny, matches);

  return CE_OK;
}

int ChessEngineGnu::cmdDepth(int nDepth)
{
  static const char *cmd = "depth";

  char            buf[MaxLineSize];
  vector<string>  matches;

  sprintf(buf, "%s %d", cmd, m_nDepth);

  writeline(buf);

  if( rspFirstLine(reDepth, matches) == 1 )
  {
    return CE_OK;
  }
  else if( isVer6() )
  {
    return -CE_ECODE_CHESS_RSP;
  }
  else
  {
    return CE_OK;
  }
}

int ChessEngineGnu::cmdRandom()
{
  static const char *cmd = "random";

  vector<string>  matches;

  if( !m_bHasRandom )
  {
    return CE_OK;
  }

  writeline(cmd);

  if( rspFirstLine(reNotSupported, matches) == 1 )
  {
    m_bHasRandom = false;
  }

  return CE_OK;
}

int ChessEngineGnu::cmdGo(bool bIsFirstGo)
{
  static const char *cmd = "go";

  vector<string>  matches;

  writeline(cmd);

  // delayed 'depth...' response.
  if( bIsFirstGo && isVer5() )
  {
    rspFirstLine(reDepth, matches);
  }

  return CE_OK;
}

int ChessEngineGnu::cmdMove(const string &strAN)
{
  vector<string>  matches;

  writeline(strAN.c_str());

  if( rspFirstLine(reNumMovePlayer, matches) == 2 )
  {
    m_var.m_nMoveNum = atoi(matches[0].c_str());
    m_var.m_strAN    = matches[1];
    return CE_OK;
  }
  else if( match(m_strLastLine, reInvalidMove, matches) > 0 )
  {
    m_strLastLine.clear();
    return -CE_ECODE_CHESS_MOVE;
  }
  else
  {
    return -CE_ECODE_CHESS_RSP;
  }
}

int ChessEngineGnu::cmdShowGame(int nMove, ChessColor eColor)
{
  static const char *cmd = "show game";

  vector<string>  matches;
  int             n;

  writeline(cmd);

  // white black
  if( rspFirstLine(reWhiteBlackHdr, matches) != 2 )
  {
    flushInput();
    return -CE_ECODE_CHESS_RSP;
  }

  // n. <white>
  // n. <white> <black>
  for(n = 0; n < nMove; ++n)
  {
    if( rspNextLine(reWhiteBlackMove2, matches, false) != 3 )
    {
      if( rspNextLine(reWhiteBlackMove1, matches, false) != 2 )
      {
        flushInput();
        return -CE_ECODE_CHESS_RSP;
      }
    }
  }

  flushInput();

  n = atoi(matches[0].c_str());

  if( n != nMove )
  {
    ROS_ERROR("Response to 'show game': last move %d != expected move %d.",
        n, m_nNumMoves);
    return -CE_ECODE_CHESS_RSP;
  }

  if( eColor == White )
  {
    m_var.m_strSAN = matches[1];
  }
  else if( matches.size() == 3 )
  {
    m_var.m_strSAN = matches[2];
  }
  else
  {
    ROS_ERROR("Response to 'show game': Bad move line."); 
    return -CE_ECODE_CHESS_RSP;
  }

  return CE_OK;
}

int ChessEngineGnu::cmdShowBoard(string &strCastling)
{
  static const char *cmd = "show board";

  vector<string>  matches;
  int             rc = CE_OK;

  strCastling.clear();

  writeline(cmd);

  // null line
  if( rspFirstLine(reNull, matches) != 1 )
  {
    rc = -CE_ECODE_CHESS_RSP;
  }

  // COLOR [CASTLING] ...
  else if( rspNextLine(reCastling, matches, false) != 2 )
  {
    rc = -CE_ECODE_CHESS_RSP;
  }

  else
  {
    strCastling = matches[1];
    rc = CE_OK;
  }

  // flush board grid
  flushInput();

  return rc;
}

int ChessEngineGnu::rspEnginesMove(ChessColor colorMove)
{
  unsigned int    msec = (unsigned int)(m_nDepth * 4 * 1000);
  vector<string>  matches;
  string          strMyMoveAN;

  // n. ... AN
  if( rspFirstLine(reNumMoveEngine, matches, msec) == 2 )
  {
    m_var.m_nMoveNum = atoi(matches[0].c_str());
    m_var.m_strAN    = matches[1];
  }
  else
  {
    return -CE_ECODE_CHESS_RSP;
  }

  // my move is : AN
  if( rspNextLine(reEngineMove, matches) == 1 )
  {
    strMyMoveAN = matches[0];
  }

  else
  {
    return -CE_ECODE_CHESS_RSP;
  }

  //
  // Optional extra line of information produced by gnuchess.
  //
  if( rspNextLine(reNull, matches) == 1 )
  {
    // no extra info
  }
  else if( rspNextLine(reResign, matches) == 1 )
  {
    m_var.m_eEoGResult = Resign;
    m_var.m_eEoGWinner = opponent(colorMove);;
  }
  else if( rspNextLine(reDraw, matches) == 1 )
  {
    m_var.m_eEoGResult = Draw;
    m_var.m_eEoGWinner = NoColor;
  }
  else if( rspNextLine(reWhiteWins, matches) == 1 )
  {
    m_var.m_eEoGResult = Checkmate;
    m_var.m_eEoGWinner = White;
  }
  else if( rspNextLine(reBlackWins, matches) == 1 )
  {
    m_var.m_eEoGResult = Checkmate;
    m_var.m_eEoGWinner = Black;
  }

  return CE_OK;
}

int ChessEngineGnu::rspFirstLine(boost::regex   &reRsp,
                                 vector<string> &matches,
                                 unsigned int   msec,
                                 bool           bLower)
{
  int   n;

  //
  // Pre-response nonsense.
  //
  while( !m_strLastLine.empty() || (readline(m_strLastLine, msec) > 0) )
  {
    if( bLower )
    {
      boost::algorithm::to_lower(m_strLastLine);
    }

    if( !m_bHasTimeLimits || (match(m_strLastLine, reTimeLimit, matches) == 0) )
    {
      break;
    }
    else
    {
      m_strLastLine.clear();
    }
  }

  if( (n = match(m_strLastLine, reRsp, matches)) > 0 )
  {
    m_strLastLine.clear();
  }

  return n;
}

int ChessEngineGnu::rspNextLine(boost::regex   &reRsp,
                                vector<string> &matches,
                                bool            bLower)
{
  int   n;

  if( m_strLastLine.empty() )
  {
    readline(m_strLastLine);
  }

  if( bLower )
  {
    boost::algorithm::to_lower(m_strLastLine);
  }

  if( (n = match(m_strLastLine, reRsp, matches)) > 0 )
  {
    m_strLastLine.clear();
  }

  return n;
}

int ChessEngineGnu::match(const string   &str,
                          boost::regex   &re,
                          vector<string> &matches)
{
  boost::cmatch what;

  matches.clear();

  GC_TRACE("match(): %s\n", re.str().c_str());

  if( boost::regex_match(str.c_str(), what, re) )
  {
    // what[0] is the whole string
    for(size_t i=1; i<what.size(); ++i)
    {
      GC_TRACE("  \"%s\"\n", what[i].str().c_str());
      matches.push_back(what[i].str());
    }
  }

  GC_TRACE("  %zu matches\n", matches.size());

  return (int)matches.size();
}

void ChessEngineGnu::clearParseVars()
{
  m_var.m_nMoveNum = 0;
  m_var.m_strAN.clear();
  m_var.m_strSAN.clear();
  m_var.m_eEoGResult = NoResult;
  m_var.m_eEoGWinner = NoColor;
}

void ChessEngineGnu::setupSignals()
{
  sigset_t          blockset;
  struct sigaction  sa;

  // block signals
  sigemptyset(&blockset);
  sigaddset(&blockset, SIGUSR1);
  sigprocmask(SIG_BLOCK, &blockset, NULL);
  //pthread_sigmask(SIG_BLOCK, &blockset, NULL);

  // install signal handlers
  sa.sa_handler = abort_handler;
  sa.sa_flags = 0;
	sigemptyset(&sa.sa_mask);
  sigaction(SIGUSR1, &sa, NULL);
}

int ChessEngineGnu::waitForPipe(unsigned int msec)
{
  static int64_t  nsecStep = 10000000;  // tenth of a second in nanoseconds

  int64_t         nsecTotal;    // total nsecs to wait on pipe read
  int64_t         nsecWait;     // wait increment
  int             fd;           // pipe fd
  fd_set          rset;         // read set
  struct timespec timeout;      // select timeout
  sigset_t        emptyset;     // blocked signals
  int             nFd;          // number of fds changed on read

  // pipe from chess backend engine
  fd = m_pipeFromChess[0];

  // milliseconds to nanoseconds
  nsecTotal = (int64_t)msec * 1000000;

  // make set of unblock signals
  sigemptyset(&emptyset);

  //
  // Pseudo block for msec milliseconds.
  //
  // Note:  The pselect() call does not return after signal, even though handler
  //        works. So, break up block time into acceptable step delay sizes.
  //
  while( nsecTotal > 0 )
  {
    nsecWait = nsecTotal >= nsecStep?  nsecStep: nsecTotal;

    nsecTotal -= nsecStep;

    setTs(nsecWait, timeout);

    FD_ZERO(&rset);
    fdset_nowarn(fd, &rset);

    //
    // pselect atomically:
    //
    // sigset_t save;
    // sigprocmask(SIG_BLOCK, &emptyset, &save);
    // select(...)
    // sigprocmask(SIG_BLOCK, &save, NULL);
    //
    nFd = pselect(fd+1, &rset, NULL, NULL, &timeout, &emptyset);

    //
    // AbortFlagged can only be set in the signal handler which is
    // unblocked only in pselect(). N.B. signals are queued however.
    //
    if( AbortFlagged )
    {
      AbortFlagged  = false;
      errno         = EINTR;
      return -1;
    }

    // read ready or error
    else if( nFd != 0 )
    {
      return nFd;
    }
  }
  
  // timeout return
  return 0;
}
