////////////////////////////////////////////////////////////////////////////////
//
// Package:     RoadNarrows Robotics ROS Chess Engine Package
//
// Link:        https://github.com/roadnarrows-robotics/chess_engine
//
// Application: chess_parser
//
// File:        chess_parser.cpp
//
/*! \file
 *
 * \brief Chess Algebraic Notation command-line parser.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

#ifdef USE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif // USE_READLINE

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceParser.h"

using namespace std;
using namespace chess_engine;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

//
// Application exit codes
//
#define APP_EC_OK   0   ///< success
#define APP_EC_INIT 2   ///< initialization fatal error
#define APP_EC_EXEC 4   ///< execution fatal error


#if 0 // RDK
static bool makeSAN(const string &strSAN, Move &move)
{
  if( strSAN.size() < 4 )
  {
    return false;
  }

  // fake ROS request arguments
  move.fromSAN(strSAN);

  if( (move.m_posFrom.m_file < ChessFileA) || 
      (move.m_posFrom.m_file > ChessFileH) )
  {
    return false;
  }
  if( (move.m_posFrom.m_rank < ChessRank1) || 
      (move.m_posFrom.m_rank > ChessRank8) )
  {
    return false;
  }

  return true;
}

static void cli_test_help()
{
  printf("  auto [MOVES]      - auto play for MOVE moves\n");
  printf("                        MOVES : maximum number of moves\n");
  printf("                                Default: 0 (go to end of game)\n");
  printf("  castling          - get available castling options\n");
  printf("  close             - close connection with chess engine\n");
  printf("  diff F            - set diffictuly F\n");
  printf("                        F : scale from 1.0 - 10.0\n");
  printf("  help              - print this help\n");
  printf("  flush             - flush input from chess engine\n");
  printf("  get               - get engine's move\n");
  printf("  new [COLOR]       - start new game with you playing COLOR\n");
  printf("                    -   COLOR : white | black\n");
  printf("                                Default: white\n");
  printf("  open [APP]        - open connection to chess engine\n");
  printf("                    -   APP : GNU chess path\n");
  printf("                              Default: gnuchess\n");
  printf("  SAN               - make your move\n");
  printf("  quit              - quit test\n");
  printf("  resign            - you resign (lose) from current game\n");
  printf("  show [gui|plain]  - show game board.\n");
  printf("                    -   MODE : gui | plain\n");
  printf("                               Default: current mode\n");
  printf("\n");
}

static void cli_test(int argc, char *argv[])
{
  const char     *app;
  ChessEngineGnu  engine;
  Game            game;
  ChessColor      colorPlayer;
  Move            move;
  char            line[80];
  int             cmdc;
  char            cmdv[2][80];
  int             rc;

  if( argc > 1 )
  {
    app = argv[1];
  }
  else
  {
    app = "gnuchess";
  }

  printf("chess_server:cli_test %s\n", app);

  cli_test_help();

  engine.openConnection(app);

  while( 1 )
  {
    printf("> ");

    cmdc = 0;

    if( fgets(line, 80, stdin) != NULL )
    {
      line[strlen(line)-1] = 0;

      cmdc = sscanf(line, "%s %s", cmdv[0], cmdv[1]);
    }

    // null command
    if( cmdc < 1 )
    {
      continue;
    }

    // quit command-line test
    else if( !strcmp(cmdv[0], "quit") )
    {
      break;
    }

    // help command-line test
    else if( !strcmp(cmdv[0], "help") )
    {
      cli_test_help();
    }

    // get castiling options
    else if( !strcmp(cmdv[0], "castling") )
    {
      string  strWhite, strBlack;

      if( (rc = engine.getCastlingOptions(strWhite, strBlack)) == CE_OK )
      {
        printf("white: %s\n", strWhite.c_str());
        printf("black: %s\n", strBlack.c_str());
      }
      else
      {
        printf("Error: %d\n", rc);
      }
    }

    // close connection to backend chess engine
    else if( !strcmp(cmdv[0], "close") )
    {
      if( engine.isConnected() )
      {
        engine.closeConnection();
        printf("Connection closed.\n");
      }
      else
      {
        printf("Connection already closed.\n");
      }
    }

    // open connection to backend chess engine
    else if( !strcmp(cmdv[0], "open") )
    {
      if( cmdc >= 2 )
      {
        app = cmdv[1];
      }
      else
      {
        app = "gnuchess";
      }
      if( !engine.isConnected() )
      {
        if( (rc = engine.openConnection(app)) == CE_OK )
        {
          printf("Connection to %s opened.\n", app);
        }
        else
        {
          printf("Error: %d\n", rc);
        }
      }
      else
      {
        printf("Connection already opened.\n");
      }
    }

    // flush input from chess engine
    else if( !strcmp(cmdv[0], "flush") )
    {
      engine.flushInput();
      printf("Input flushed.\n");
    }

    // difficulty F
    else if( !strcmp(cmdv[0], "diff") )
    {
      if( cmdc >= 2 )
      {
        engine.setGameDifficulty((float)atof(cmdv[1]));
      }
    }

    // simulate ROS request to start a new game
    else if( !strcmp(cmdv[0], "new") )
    {
      // fake ROS request arguments
      if( (cmdc < 2) || !strcmp(cmdv[1], "white") )
      {
        colorPlayer = White;
      }
      else
      {
        colorPlayer = Black;
      }

      if( (rc = engine.startNewGame(colorPlayer)) == CE_OK )
      {
        game.setupBoard();
      }
      else
      {
        printf("Error: %d\n", rc);
      }
    }

    // simulate ROS request for get engine's move
    else if( !strcmp(cmdv[0], "get") )
    {
      rc = engine.getEnginesMove(move);
      cout << "e: " << move << endl;
      if( rc == CE_OK )
      {
        rc = game.sync(move);
        cout << "g: " << move << endl;
      }
    }

    // simulate ROS request to auto-play. Node will publish
    else if( !strcmp(cmdv[0], "auto") )
    {
      int nMoves, n = 0;

      nMoves = cmdc < 2? 0: atoi(cmdv[1]);

      while( game.isPlaying() && ((nMoves == 0) || (n < nMoves)) )
      {
        usleep(500000);
        rc = engine.getEnginesMove(move, true);
        cout << "e: " << move << endl;
        rc = game.sync(move);
        cout << "g: " << move << endl;
        cout << endl;
        if( move.m_player == Black )
        {
          ++n;
        }
      }
    }

    else if( !strcmp(cmdv[0], "resign") )
    {
      engine.resign();
      game.stopPlaying(Resign, opponent(engine.getPlayersColor()));
    }

    else if( !strcmp(cmdv[0], "show") )
    {
      // fake ROS request arguments
      if( cmdc >= 2 )
      {
        if( !strcmp(cmdv[1], "gui") )
        {
          game.setGuiState(true);
        }
        else if( !strcmp(cmdv[1], "plain") )
        {
          game.setGuiState(false);
        }
      }
      cout << game;
    }

    // RDK: add other ROS requests here

    // simulate ROS request for player to make a move
    else if( makeSAN(cmdv[0], move) )
    {
        rc = engine.makePlayersMove(move);
        cout << "e: " << move << endl;
        if( rc == CE_OK )
        {
          rc = game.sync(move);
          cout << "g: " << move << endl;
        }
    }

    else
    {
      printf("%s: unknown command.\n", cmdv[0]);
    }
  }
}

//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

/*!
 * \breif Chess server main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line arguments.
 *
 * \return Exits with 0 on success, \>0 on failure.
 */
int main(int argc, char *argv[])
{
  string    strNodeName;
  double    fHz = 10.0;
  int       n;
  int       rc;

  ros::init(argc, argv, NodeName);

  ros::NodeHandle nh(NodeName);

  if( !ros::master::check() )
  {
    return APP_EC_EXEC;
  }

  // read back actual node name
  strNodeName = ros::this_node::getName();

  ROS_INFO("%s: Node started.",  strNodeName.c_str());

  // the chess server work horse
  ChessServer chessServer(nh);

  // initialize chess server and underlining class objects
  if( (rc = chessServer.initialize()) != chess_engine::CE_OK )
  {
    ROS_ERROR_STREAM(strNodeName << ": "
        << "Failed to initialize chess server: "
        << chess_engine::strecode(rc) << "(" << rc << ")");
    return APP_EC_INIT;
  }

  // advertise services
  n = chessServer.advertiseServices();

  ROS_INFO("%s: %d services advertised.", strNodeName.c_str(), n);

  // advertise publishers
  n = chessServer.advertisePublishers();

  ROS_INFO("%s: %d topic publishers advertised.", strNodeName.c_str(), n);

  // subscribe to topics (none for now)
  n = chessServer.subscribeToTopics();

  ROS_INFO("%s: %d topics subscribed.", strNodeName.c_str(), n);

  // start action servers
  n = chessServer.startActionServers();

  ROS_INFO("%s: %d action servers started.", strNodeName.c_str(), n);

  // set loop rate in hertz
  ros::Rate loop_rate(fHz);

  ROS_INFO("%s: Running at %.1lfHz.", strNodeName.c_str(), fHz);

  //
  // The loop.
  //
  while( ros::ok() )
  {
    // make any callbacks on pending ROS services
    ros::spinOnce(); 

    // publish any new data on advertised topics
    chessServer.publish();

    // sleep to keep loop hertz rate
    loop_rate.sleep();
  }

  return APP_EC_OK;
}
#endif // RDK

cmds = rnr::Commander();

cmds.add("help");
cmds.add("quit");
cmds.add("set parser {bnf | regex}");
cmds.add("get parser");
cmds.add("autoplay <int>");
cmds.add("compute");
cmds.add("<string>");


int main()
{
  cout << "/////////////////////////////////////////////////////////\n\n";
  cout << "\t\tAlgebraic Notation Parser\n\n";
  cout << "/////////////////////////////////////////////////////////\n\n";

  cout << "Enter CAN or SAN\n";
  cout << "Type q to quit\n\n";

  bool  trace = false;

  cmds.compile();

  ChessANParser parser;
  ChessMove     move;

  parser.setTracing(trace);

  string str;

  while( true )
  {
    cout << "an> ";

    if( !getline(cin, str) )
    {
      break;
    }
    else if (str.empty() || str[0] == 'q' )
    {
      break;
    }

    if( parser.parse(str, move) )
    {
      //cout << "good" << endl;
      cout << move;
    }
    else if( !trace )
    {
      cout << parser.getErrorStr() << endl;
    }
  }

  cout << "\nbye\n";

  return 0;
}
