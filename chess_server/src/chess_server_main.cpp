////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_server_main.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The ROS node chess_server main.
 *
 * The ROS chess_server node provides services and subscriptions to play 
 * chess using the GNU chess gnuchess as the backend engine.
 *
 * The gnuchess (http://www.gnu.org/software/chess) engine is unmodified
 * (and hence, apt-get installable).
 *
 * The GNU Chess Versions supported and tested are 5.07, 6.1.1.

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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

#include "ros/ros.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

#include "chess_engine_gnu.h"
#include "chess_server.h"
#include "chess_as_auto_play.h"
#include "chess_as_get_engines_move.h"

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

const char *NodeName = "chess_server";  ///< this ROS node's registered name


static bool makeSAN(const string &strSAN, Move &move)
{
  if( strSAN.size() < 4 )
  {
    return false;
  }

  // fake ROS request arguments
  move.fromSAN(strSAN);

  if( (move.m_sqFrom.m_file < ChessFileA) || 
      (move.m_sqFrom.m_file > ChessFileH) )
  {
    return false;
  }
  if( (move.m_sqFrom.m_rank < ChessRank1) || 
      (move.m_sqFrom.m_rank > ChessRank8) )
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

int main(int argc, char *argv[])
{
  uint32_t    seqnum;
  int         rc;

  ros::init(argc, argv, NodeName);

  ros::NodeHandle nh(NodeName);

  if( !ros::master::check() )
  {
    cli_test(argc, argv);
    return APP_EC_OK;
  }

  ROS_INFO("%s: Node started.",  ros::this_node::getName().c_str());

  ChessServer chess(nh);

  if( (rc = chess.connect()) != CE_OK )
  {
    ROS_ERROR_STREAM(chess.getEngine().getChessApp() << ": "
        << chess_engine::strerror(rc) << "(" << rc << ")");
    return APP_EC_INIT;
  }

  chess.advertiseServices();

  ROS_INFO("%s: Services registered.", ros::this_node::getName().c_str());

  chess.advertisePublishers();

  ROS_INFO("%s: Publishers registered.", ros::this_node::getName().c_str());

  ASAutoPlay       asAutoPlay("auto_play", chess);
  ASGetEnginesMove asGetEnginesMove("get_engines_move_action", chess);

  ROS_INFO("%s: Action servers registered and started.",
      ros::this_node::getName().c_str());

  seqnum = 0;

  ros::Rate loop_rate(2);

  while( ros::ok() )
  {
    ros::spinOnce(); 

    seqnum = chess.publish(seqnum);

    loop_rate.sleep();
  }

  return APP_EC_OK;
}
