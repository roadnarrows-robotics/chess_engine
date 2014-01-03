////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_engine_gnu.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The GNU chess gnuchess backend engine implimentation.
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

#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>

#include "ros/ros.h"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "chess.h"
#include "chess_server.h"
#include "chess_engine_gnu.h"

using namespace std;
using namespace chess_engine;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// GNU Chess Tracing
//

#define GC_TRACE_ENABLE   ///< tracing on/off switch

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


//
// Response regualar expressions.
//

/*! null response */
boost::regex reNull("(^$)");

/*! any response */
boost::regex reAny("(.*)");

/*! pre-response: 'timelime...' (for version >= 6) */
boost::regex reTimeLimit("(^timelimit.*)");

/*! 'version' response: 'gnu chess VERSION' */
boost::regex reVersion("^gnu chess (.*)");

/*! 'depth DEPTH' response: 'search to a depth of DEPTH' */
boost::regex reDepth("^search to a depth of ([0-9]+)");

/*! engine's numbered move response: 'N. ... AN' */
boost::regex reNumMoveEngine("^([0-9]+)\\.\\s+\\.\\.\\.\\s+(\\S+)");

/*! players's numbered move response: 'N. AN' */
boost::regex reNumMovePlayer("^([0-9]+)\\.\\s+(\\S+)");

/*! engines move response: 'my move is : SAN' */
boost::regex reEngineMove("^my move is\\s*:\\s*([a-h][1-8][a-h][1-8]\\S*)");

/*! engine resigns: 'resign ...' */
boost::regex reResign("(resign).*");

/*! engine determines a draw: '1/2-1/2 ...' */
boost::regex reDraw("(1/2-1/2).*");

/*! white checkmates: '1-0 ...' */
boost::regex reWhiteMates("(1-0).*");

/*! black checkmates: '0-1 ...' */
boost::regex reBlackMates("(0-1).*");

/*! 'show game' response header: 'white black ...' */
boost::regex reWhiteBlackHdr("\\s*(white)\\s+(black)\\s*");

/*! 'show game' response 1 move: 'N. AN' */
boost::regex reWhiteBlackMove1("\\s*([0-9]+)\\.\\s+(\\S+)\\s*");

/*! 'show game' response 2 moves: 'N. AN AN' */
boost::regex reWhiteBlackMove2("\\s*([0-9]+)\\.\\s+(\\S+)\\s+(\\S+)\\s*");

/*! 'show board' response status line: 'COLOR [CASTLING] ...' */
boost::regex reCastling("\\s*(\\S+)\\s+([KQkq]*)\\s*");

/*! bad move response: 'invalid move: SAN' or 'illegal move: SAN' */
boost::regex reInvalidMove("^(invalid move:|illegal move:)\\s+(.*)");

/*! unsupported command response: 'command 'CMD' is currently not supported.' */
boost::regex reNotSupported("^command '(.*)' is currently not supported.");


// (partial) response strings
const char* const UciRspCheckMate   = "mates";
const char* const UciRspDraw        = "1/2";
const char* const UciRspResign      = "resign";
const char* const UciRspInvalidMove = "Invalid move";


PRAGMA_IGNORED(sign-conversion)
/*!
 * \brief FD_SET() wrapper with no annoying warnings.
 * \param fd    File descriptor to add to set.
 * \param pset  Pointer to fd set.
 */
static inline void fdset_nowarn(int fd, fd_set *pset)
{
  FD_SET(fd, pset);
}
PRAGMA_WARNING(sign-conversion)

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system seconds and useconds.
 */
static inline void timer_mark(struct timeval *pTvMark)
{
  if( gettimeofday(pTvMark, NULL) != OK )
  {
    LOGSYSERROR("gettimeofday()");
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
uint_t timer_elapsed(struct timeval *pTvMark)
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

  return (uint_t)(tvDelta.tv_sec * 1000000 + tvDelta.tv_usec);
}


// -----------------------------------------------------------------------------
// Class ChessEngineGnu
// -----------------------------------------------------------------------------

ChessEngineGnu::ChessEngineGnu()
{
  m_pipeToChess[0]    = -1;
  m_pipeToChess[1]    = -1;
  m_pipeFromChess[0]  = -1;
  m_pipeFromChess[1]  = -1;
  m_pidChild          = -1;
  m_nDepth            = 2;

  m_bHasRandom        = true;
  m_bHasTimeLimits    = true;

  //bool   UciWaitingForEngineMoveRsp;
  //char   UciEngineMoveRspBuf[GC_MAX_BUF_SIZE];
  //int    UciMovePairCnt;
}

ChessEngineGnu::~ChessEngineGnu()
{
  closeConnection();
}
  

//..............................................................................
// High-Level Public Interface
//..............................................................................

int ChessEngineGnu::openConnection(const string &strChessApp)
{
  char   *argv[8];

  m_strChessApp = strChessApp;

  // make chess server node to chess engine backend pipe
  if( pipe(m_pipeToChess) == -1 )
  {
    LOGSYSERROR("Failed to create pipe to %s.", m_strChessApp.c_str());
    return -CE_ECODE_SYS;
   }

  // make chess engine backend to chess server node pipe
  if ( pipe(m_pipeFromChess) == -1 )
  {
    LOGSYSERROR("Failed to create pipe from chess.", m_strChessApp.c_str());
    return -CE_ECODE_SYS;
  }

  // fork server node
  m_pidChild = fork();

  // fork failed
  if( m_pidChild == -1 )
  {
    LOGSYSERROR("Could not fork %s.", "chess_server");
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
    argv[0] = (char *)m_strChessApp.c_str();
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
    LOGDIAG1("Forked-exec'ed %s, child pid=%u.", sChessApp, m_pidChild);

    // close unused pipe ends 
    close(m_pipeToChess[0]);
    close(m_pipeFromChess[1]);

    if( fcntl(m_pipeFromChess[0], F_SETFL, O_NONBLOCK) < 0 )
    {
      LOGSYSERROR("fcntl(%d,F_SETFL,O_NONBLOCK)", m_pipeFromChess[0]);
      return -CE_ECODE_SYS;
    }

    GC_TRACE_OPEN();
    GC_TRACE("Application: %s\n", m_strChessApp.c_str());

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

  m_strChessApp.clear();

  m_pipeToChess[0]    = -1;
  m_pipeToChess[1]    = -1;
  m_pipeFromChess[0]  = -1;
  m_pipeFromChess[1]  = -1;
  m_pidChild          = -1;

  GC_TRACE("closeConnection()\n");

  m_bIsConn     = false;
  m_bIsPlaying  = false;

  return CE_OK;
}

int ChessEngineGnu::setGameDifficulty(float fDifficulty)
{
  ChessEngine::setGameDifficulty(fDifficulty);

  m_nDepth = (int)(2.0 * m_fDifficulty);

  // ok to delay setting engine's search depth 
  if( isConnected() )
  {
    return cmdDepth(m_nDepth);
  }

  return CE_OK;
}

int ChessEngineGnu::startNewGame(int colorPlayer)
{
  int   rc;

  if( !isConnected() )
  {
    return -CE_ECODE_NO_EXEC;
  }

  m_bIsPlaying  = false;
  m_nMove       = 0;

  flushInput();

  if( (rc = cmdNew()) == CE_OK )
  {
    cmdDepth(m_nDepth);
    //cmdRandom(); // N.B. not supported in v6 and poorly in v5
    ChessEngine::startNewGame(colorPlayer);
  }

  return rc;
}

int ChessEngineGnu::makeAMove(ChessColor colorMove, ChessMove &move)
{
  ChessSquare sqFrom;
  ChessSquare sqTo;
  int         rc;

  // save target move
  sqFrom  = move.m_sqFrom;
  sqTo    = move.m_sqTo;

  // clear move
  move.clear();

  // initialize known move data
  move.m_nMove  = m_nMove;
  move.m_color  = colorMove;
  move.m_sqFrom = sqFrom;
  move.m_sqTo   = sqTo;

  //
  // Pre-checks
  //
  if( !isConnected() ) 
  { 
    printf("ROSLOG: Error: not connected.\n");
    move.m_result = GameFatal;
    return -CE_ECODE_NO_EXEC;
  }

  if( !isPlayingAGame() ) 
  { 
    printf("ROSLOG: Error: no game.\n");
    move.m_result = NoGame;
    return -CE_ECODE_CHESS_NO_GAME;
  }

  if( whoseTurn() != colorMove )
  {
    printf("ROSLOG: Error: out-of-turn.\n");
    move.m_result = OutOfTurn;
    return -CE_ECODE_CHESS_OUT_OF_TURN;
  }

  // clear old parsed variables
  clearParseVars();

  // send move command
  rc = cmdMove(move.toSAN());

  // results
  switch( rc )
  {
    // command succedded
    case CE_OK:
      // out-of-sync with chess engine
      if( m_nNewMove != m_nMove )
      {
        printf("ROSLOG: Error: newmove=%d != move=%d.\n", m_nNewMove, m_nMove);
        move.m_result = GameFatal;
        return -CE_ECODE_CHESS_FATAL;
      }

      if( m_strNewAN != m_strNewSAN )
      {
        printf("ROSLOG: Warning: %s != %s.\n",
            m_strNewAN.c_str(), m_strNewSAN.c_str());
      }

      rc = cmdShowGame(m_nMove, colorMove);

      if( rc != CE_OK )
      {
        printf("ROSLOG: Error: failed to parse game state.\n");
        move.m_result = GameFatal;
        return -CE_ECODE_CHESS_FATAL;
      }

      // parse AN for more info
      move.fromAN(m_strNewAN);

      // alternate turns
      alternateTurns(colorMove);

      return CE_OK;

    // invalid move, but ok since player may be testing a move
    case -CE_ECODE_CHESS_MOVE:
      move.m_result = BadMove;
      return CE_OK;

    // fatal game errors
    case -CE_ECODE_CHESS_RSP:
    case -CE_ECODE_CHESS_FATAL:
    default:
      move.m_result = GameFatal;
      return -CE_ECODE_CHESS_FATAL;
  }
}

int ChessEngineGnu::getEnginesMove(ChessMove &move, bool bAuto)
{
  ChessColor  colorMove;
  int         rc;

  // auto self-play
  if( bAuto )
  {
    colorMove = m_colorTurn;
  }
  // opponent play
  else
  {
    colorMove = m_colorEngine;
  }

  // clear move
  move.clear();

  // initialize known move data
  move.m_nMove  = m_nMove;
  move.m_color  = colorMove;

  //
  // Pre-checks
  //
  if( !isConnected() ) 
  { 
    printf("ROSLOG: Error: not connected.\n");
    move.m_result = GameFatal;
    return -CE_ECODE_NO_EXEC;
  }

  if( !isPlayingAGame() ) 
  { 
    ROS_ERROR("ROSLOG: Error: no game.\n");
    move.m_result = NoGame;
    return -CE_ECODE_CHESS_NO_GAME;
  }

  if( whoseTurn() != colorMove )
  {
    printf("ROSLOG: Error: out-of-turn.\n");
    move.m_result = OutOfTurn;
    return -CE_ECODE_CHESS_OUT_OF_TURN;
  }

  // tell engine to go first
  if( (m_nMove == 1) && (colorMove == White) )
  {
    cmdGo(true);
  }
  else if( bAuto )
  {
    cmdGo();
  }

  // clear old parsed variables
  clearParseVars();

  // wait for move response from engine
  rc = rspEnginesMove(colorMove);

  // results
  switch( rc )
  {
    // response succedded
    case CE_OK:
      // out-of-sync with chess engine
      if( m_nNewMove != m_nMove )
      {
        printf("ROSLOG: Error: newmove=%d != move=%d.\n", m_nNewMove, m_nMove);
        move.m_result = GameFatal;
        return -CE_ECODE_CHESS_FATAL;
      }

      // extra line indication engine's call of the game
      move.m_result = m_eNewResult;
      move.m_winner = m_eNewWinner;

      // from-to move
      move.fromSAN(m_strNewSAN);

      if( m_strNewAN != m_strNewSAN )
      {
        printf("ROSLOG: Warning: %s != %s.\n",
            m_strNewAN.c_str(), m_strNewSAN.c_str());
      }

      rc = cmdShowGame(m_nMove, colorMove);

      if( rc != CE_OK )
      {
        printf("ROSLOG: Error: failed to parse game state.\n");
        move.m_result = GameFatal;
        return -CE_ECODE_CHESS_FATAL;
      }

      // parse AN for more info
      move.fromAN(m_strNewAN);

      // alternate turns
      alternateTurns(colorMove);

      break;

    // fatal game errors
    case -CE_ECODE_CHESS_RSP:
    case -CE_ECODE_CHESS_FATAL:
    default:
      move.m_result = GameFatal;
      break;
  }

  return rc;
}

int ChessEngineGnu::resign()
{
  //
  // Pre-checks
  //
  if( !isConnected() ) 
  { 
    printf("ROSLOG: Error: not connected.\n");
    return -CE_ECODE_NO_EXEC;
  }

  if( !isPlayingAGame() ) 
  { 
    printf("ROSLOG: Error: no game.\n");
    return -CE_ECODE_CHESS_NO_GAME;
  }

  m_bIsPlaying = false;

  return CE_OK;
}

int ChessEngineGnu::getCastlingOptions(string &strWhiteCastling,
                                       string &strBlackCastling)
{
  string  strOptions;
  int     rc;

  strWhiteCastling.clear();
  strBlackCastling.clear();

  //
  // Pre-checks
  //
  if( !isConnected() ) 
  { 
    printf("ROSLOG: Error: not connected.\n");
    return -CE_ECODE_NO_EXEC;
  }

  if( !isPlayingAGame() ) 
  { 
    printf("ROSLOG: Error: no game.\n");
    return -CE_ECODE_CHESS_NO_GAME;
  }

  if( (rc = cmdShowBoard(strOptions)) == CE_OK )
  {
    for(size_t i=0; i<strOptions.size(); ++i)
    {
      switch( strOptions[i] )
      {
        // white
        case 'K':
        case 'Q':
          strWhiteCastling.push_back(strOptions[i]);
          break;
        // black
        case 'k':
        case 'q':
          strBlackCastling.push_back(toupper(strOptions[i]));
          break;
        // ignore
        default:
          break;
      }
    }
  }

  return rc;
}


//..............................................................................
// Low-Level Public I/O Interface
//..............................................................................

int ChessEngineGnu::readline(string &strLine, uint_t msec)
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

int ChessEngineGnu::readline(char buf[], size_t sizeBuf, uint_t msec)
{
  uint_t          usec = msec * 1000;
  struct timeval  timeout;
  fd_set          rset;
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
    FD_ZERO(&rset);
    fdset_nowarn(fd, &rset);

    // (re)load timeout (gets munged after each select())
    timeout.tv_sec  = (time_t)(usec / 1000000);
    timeout.tv_usec = (time_t)(usec % 1000000);

    // wait to bytes to be available to be read
    nFd = select(fd+1, &rset, NULL, NULL, &timeout);

    // system error occurred on select - interpret
    if( nFd < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout, retry
        case EINTR:         // read was interrupted, retry
          break;
        default:            // non-recoverable error
          buf[nBytes] = 0;
          LOGSYSERROR("select(%d,...)", fd);
          GC_TRACE("readline() %s\n", buf);
          return -CE_ECODE_SYS;
      }
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on read timed out.");
      bDone = true;
    }

    // read the available data from the pipe
    else if( (n = read(fd, buf+nBytes, (size_t)1)) == 1 )
    {
      switch( buf[nBytes] )
      {
        case '\n':    // received a response line
          buf[nBytes] = 0;
          //LOGDIAG4("%s(): %zd bytes response line.", LOGFUNCNAME, nBytes);
          GC_TRACE("readline() %s\n", buf);
          return (int)nBytes;
        case '\r':    // received a character, but ignore
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
          LOGSYSERROR("select(%d,...)", fd);
          GC_TRACE("readline() %s\n", buf);
          return -CE_ECODE_SYS;
      }
    }

    // got a select but no bytes 
    else if( n == 0 )
    {
      LOGDIAG4("select() ok, but read()=0.");
      bDone = true;
    }
  }

  LOGDIAG4("%s(): %zd bytes partial response line.", LOGFUNCNAME, nBytes);
  
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


//..............................................................................
// Initialization and Clean Up Functions
//..............................................................................

void ChessEngineGnu::configure()
{
  flushInput();

  // determine version
  cmdVersion();

  GC_TRACE("Chess Engine %s\n", m_strVersion.c_str());

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
    LOGWARN("GNU Chess %s unsupported. Assume version 6.x protocol.",
        m_strVersion.c_str());
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


//..............................................................................
// Low-Level Comand and Response Functions
//..............................................................................

int ChessEngineGnu::cmdVersion()
{
  static const char *cmd = "version";

  vector<string>  matches;
  int             n;

  writeline(cmd);

  if( rspFirstLine(reVersion, matches) == 1 )
  {
    m_strVersion = matches[0];
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

int ChessEngineGnu::cmdMove(const string &strSAN)
{
  vector<string>  matches;

  m_strNewSAN = strSAN;

  writeline(strSAN.c_str());

  if( rspFirstLine(reNumMovePlayer, matches) == 2 )
  {
    m_nNewMove = atoi(matches[0].c_str());
    m_strNewAN = matches[1];
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

int ChessEngineGnu::cmdShowGame(int nMove, ChessColor color)
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
    printf("ROSLOG: Error: show game: game=%d != move=%d.\n", n, m_nMove);
    return -CE_ECODE_CHESS_RSP;
  }

  if( color == White )
  {
    m_strNewAN = matches[1];
  }
  else if( matches.size() == 3 )
  {
    m_strNewAN = matches[2];
  }
  else
  {
    printf("ROSLOG: Error: show game: bad move line."); 
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
  uint_t          msec = (uint_t)(m_nDepth * 4 * 1000);
  vector<string>  matches;

  // n. ... SAN
  if( rspFirstLine(reNumMoveEngine, matches, msec) == 2 )
  {
    m_nNewMove  = atoi(matches[0].c_str());
    m_strNewAN  = matches[1];
  }
  else
  {
    return -CE_ECODE_CHESS_RSP;
  }

  // my move ...
  if( rspNextLine(reEngineMove, matches) == 1 )
  {
    m_strNewSAN = matches[0];
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
    m_eNewResult = Resign;
    m_eNewWinner = opponent(colorMove);;
  }
  else if( rspNextLine(reDraw, matches) == 1 )
  {
    m_eNewResult = Draw;
  }
  else if( rspNextLine(reWhiteMates, matches) == 1 )
  {
    m_eNewResult = Checkmate;
    m_eNewWinner = White;
  }
  else if( rspNextLine(reBlackMates, matches) == 1 )
  {
    m_eNewResult = Checkmate;
    m_eNewWinner = Black;
  }

  return CE_OK;
}

int ChessEngineGnu::rspFirstLine(boost::regex   &reRsp,
                                 vector<string> &matches,
                                 uint_t         msec,
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


#if 0 // RDK
static int UciParseMoveRsp(const char  bufRsp[],
                           const char *sExpectedMove,
                           char        bufAlgSqFrom[],
                           char        bufAlgSqTo[])
{
  int   n;
  char  bufEcho[GC_MAX_BUF_SIZE];
  
  bufAlgSqFrom[0] = 0;
  bufAlgSqTo[0] = 0;

  n = 0;
  bufEcho[0] = 0;

  if( strcasestr(bufRsp, UciRspCheckMate) != NULL )
  {
    bufAlgSqFrom[0] = UciCodeCheckMate;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeCheckMate;
    bufAlgSqTo[1]   = 0;

    return CE_OK;
  }

  else if( !strncasecmp(bufRsp, UciRspResign, strlen(UciRspResign)) )
  {
    bufAlgSqFrom[0] = UciCodeResign;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeResign;
    bufAlgSqTo[1]   = 0;

    return CE_OK;
  }

  else if( !strncasecmp(bufRsp, UciRspDraw, strlen(UciRspDraw)) )
  {
    bufAlgSqFrom[0] = UciCodeDraw;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeDraw;
    bufAlgSqTo[1]   = 0;

    return CE_OK;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspInvalidMove,
                                              strlen(UciRspInvalidMove)) )
  {
    bufAlgSqFrom[0] = UciCodeInvalid;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeInvalid;
    bufAlgSqTo[1]   = 0;
  }

  sscanf(bufRsp, "%d. %s", &n, bufEcho);
  
  if( (n == 0) || (bufEcho[0] == 0) )
  {
    LOGERROR("CBG: Bad response '%s'", bufRsp);
    return -CE_ECODE_GEN;
  }

  else if( n != UciMovePairCnt ) 
  {
    LOGERROR("CBG: Response move out-of-sequence: Expected %d, got %d.",
        UciMovePairCnt, n);
    return -CE_ECODE_GEN;
  }

  else if( (sExpectedMove != NULL) && strcasecmp(sExpectedMove, bufEcho) )
  {
    LOGERROR("CBG: Response move unexpected: Expected '%s', got '%s'.",
        sExpectedMove, bufEcho);
    return -CE_ECODE_GEN;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspResign,
                                              strlen(UciRspResign)) )
  {
      bufAlgSqFrom[0] = UciCodeResign;
      bufAlgSqFrom[1] = 0;
      bufAlgSqTo[0]   = UciCodeResign;
      bufAlgSqTo[1]   = 0;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspDraw, strlen(UciRspDraw)) )
  {
    bufAlgSqFrom[0] = UciCodeDraw;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeDraw;
    bufAlgSqTo[1]   = 0;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspInvalidMove,
                                              strlen(UciRspInvalidMove)) )
  {
    bufAlgSqFrom[0] = UciCodeInvalid;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeInvalid;
    bufAlgSqTo[1]   = 0;
  }

  else
  {
    bufAlgSqFrom[0] = bufEcho[0];
    bufAlgSqFrom[1] = bufEcho[1];
    bufAlgSqFrom[2] = 0;

    bufAlgSqTo[0]   = bufEcho[2];
    bufAlgSqTo[1]   = bufEcho[3];
    bufAlgSqTo[2]   = 0;
  }

  return CE_OK;
}

int StaleMateUciTrans(StaleMateSession &session,
                      const char       *sReq,
                      char              bufRsp[],
                      size_t            sizeRspBuf)
{
  ssize_t   n;
  size_t    i;
  int       rc;

  if( !session.m_game.bUseChessEngine )
  {
    return CE_OK;
  }

  //UciFlushIn();

  if( sizeRspBuf > 0 )
  {
    GC_TRACE("CBG: Req: \"%s\"\n", sReq);
  }
  else
  {
    GC_TRACE("CBG: Cmd: \"%s\"\n", sReq);
  }

  n = write(UciPipeToChess[1], sReq, strlen(sReq));
  n = write(UciPipeToChess[1], "\n", 1);

  if( n < 0 )
  {
    LOGSYSERROR("write(%d, \"%s\", %uz)",
                        UciPipeToChess[1], sReq, strlen(sReq));
    return -CE_ECODE_SYS;
  }

  if( sizeRspBuf > 0 )
  {
    rc = UciReadLine(bufRsp, sizeRspBuf);
    GC_TRACE("CBG: Rsp: \"%s\"\n", bufRsp);
    return rc;
  }
  else
  {
    return CE_OK;
  }
}

void StaleMateUciNewGame(StaleMateSession &session)
{
  char    bufRsp[GC_MAX_BUF_SIZE];
  int     rc;

  if( !session.m_game.bUseChessEngine )
  {
    return;
  }

  StaleMateUciTrans(session, "new", NULL, 0);

  UciMovePairCnt = 1;

  // hekateros is white, make the first move.
  if( session.m_game.bHekHasWhite )
  {
    UciWaitingForEngineMoveRsp = true;
    UciEngineMoveRspBuf[0] = 0;
    StaleMateUciTrans(session, "go", NULL, 0);
  }
  else
  {
    UciWaitingForEngineMoveRsp = false;
  }

  UciFlushIn();
}

// get move response from engine
bool StaleMateUciGetEnginesMove(StaleMateSession &session,
                                char             bufAlgSqFrom[],
                                char             bufAlgSqTo[])
{
  char  bufRsp[GC_MAX_BUF_SIZE];
  int   rc;

  rc = UciReadLine(bufRsp, sizeof(bufRsp));

  GC_TRACE("CBG: Eng: \"%s\"\n", bufRsp);

  switch( rc )
  {
    case CE_OK:
      strcat(UciEngineMoveRspBuf, bufRsp);
      LOGDIAG3("Engine's Response Move: '%s'.", UciEngineMoveRspBuf);

      rc = UciParseMoveRsp(UciEngineMoveRspBuf, NULL, bufAlgSqFrom, bufAlgSqTo);

      if( rc == CE_OK )
      {
        // hekateros is black - advace the move pair count
        if( !session.m_game.bHekHasWhite )
        {
          UciMovePairCnt++;
        }

        UciWaitingForEngineMoveRsp = false;
      }
      return true;

    case CE_ECODE_TIMEDOUT:
      strcat(UciEngineMoveRspBuf, bufRsp);
      LOGDIAG3("Engine's Response Move (partial): '%s'.", UciEngineMoveRspBuf);
      return false;

    default:
      UciEngineMoveRspBuf[0] = 0;
      return false;
  }
}

int StaleMateUciPlayersMove(StaleMateSession &session,
                            const char       *sAlgSqFrom,
                            const char       *sAlgSqTo,
                            char             *pUciCode)
{
  char  bufMove[GC_MAX_BUF_SIZE];
  char  bufRsp[GC_MAX_BUF_SIZE];
  char  bufAlgSqFrom[GC_MOVE_BUF_SIZE];
  char  bufAlgSqTo[GC_MOVE_BUF_SIZE];
  int   rc;

  sprintf(bufMove, "%s%s", sAlgSqFrom, sAlgSqTo);

  rc = StaleMateUciTrans(session, bufMove, bufRsp, sizeof(bufRsp));

  if( rc != DYNA_OK )
  {
    LOGERROR("CBG: No response to opponent's move %s.", bufMove);
    return rc;
  }

  rc = UciParseMoveRsp(bufRsp, bufMove, bufAlgSqFrom, bufAlgSqTo);

  if( rc == CE_OK )
  {
    // move was not a valid move
    switch( bufAlgSqFrom[0] )
    {
      case UciCodeInvalid:
      case UciCodeError:
        *pUciCode = UciCodeInvalid;
        return -CE_ECODE_NO_EXEC;
      case UciCodeDraw:
      case UciCodeResign:
      case UciCodeCheckMate:
        *pUciCode = bufAlgSqFrom[0];
      default:
        *pUciCode = 0;
        break;
    }

    // opponent is black - advace the move pair count
    if( session.m_game.bHekHasWhite )
    {
      UciMovePairCnt++;
    }

    // heks move - wait for engine's response
    UciWaitingForEngineMoveRsp = true;
    UciEngineMoveRspBuf[0] = 0;
  }

  return rc;
}

void StaleMateUciSelfPlay(StaleMateSession &session, bool bIsNewGame)
{
  static int nMoves;

  char    bufRsp[GC_MAX_BUF_SIZE];

  if( bIsNewGame )
  {
    session.m_game.bHekHasWhite = true;
    StaleMateUciTrans(session, "new", NULL, 0);
    StaleMateUciTrans(session, "depth 4", bufRsp, sizeof(bufRsp));
    UciMovePairCnt = 0;
    nMoves = 0;
    UciFlushIn();
  }

  UciWaitingForEngineMoveRsp = true;
  UciEngineMoveRspBuf[0] = 0;

  StaleMateUciTrans(session, "go", NULL, 0);

  ++nMoves;
  if( nMoves & 0x01 )
  {
    ++UciMovePairCnt;
  }
}
#endif // RDK
