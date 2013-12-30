////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_server.cpp
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 16:16:44 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3334 $
 *
 * \brief The chess_server main.
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <map>

#include "chess.h"

#include "chess_move.h"
#include "chess_game.h"
#include "chess_engine_gnu.h"
#include "chess_server.h"

using namespace std;
using namespace chess_engine;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

static map<int, string> NameColors;   ///< name of colors
static map<int, string> NamePieces;   ///< name of pieces
static map<int, string> NameCastling; ///< name of castle moves
static map<int, string> NameResults;  ///< name of results

static void cli_test_help()
{
  printf("  auto\n");
  printf("  close\n");
  printf("  difficulty F\n");
  printf("  help\n");
  printf("  e[ngine]\n");
  printf("  flush\n");
  printf("  new [black|white]\n");
  printf("  open [APP]\n");
  printf("  p[layer] SAN\n");
  printf("  quit\n");
  printf("\n");
}

static void cli_test(int argc, char *argv[])
{
  const char     *app;
  ChessEngineGnu  engine;
  ChessGame       game;
  ChessColor      colorPlayer;
  ChessMove       move;
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
        rc = engine.openConnection(app);
        printf("Connection to %s opened.\n", app);
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
    else if( !strcmp(cmdv[0], "difficulty") )
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

      rc = engine.startNewGame(colorPlayer);
      if( rc == CE_OK )
      {
        game.setupBoard();
      }
    }

    // simulate ROS request for player to make a move
    else if( !strncmp(cmdv[0], "player", strlen(cmdv[0])) )
    {
      if( (cmdc > 1) && (strlen(cmdv[1]) >= 4) )
      {
        // fake ROS request arguments
        move.fromSAN(cmdv[1]);
        move.m_sqFrom.m_file = cmdv[1][0];
        move.m_sqFrom.m_rank = cmdv[1][1];
        move.m_sqTo.m_file   = cmdv[1][2];
        move.m_sqTo.m_rank   = cmdv[1][3];

        rc = engine.makePlayersMove(move);
        cout << "e: " << move << endl;
        if( rc == CE_OK )
        {
          rc = game.sync(move);
          cout << "g: " << move << endl;
        }
      }
    }

    // simulate ROS request for get engine's move
    else if( !strncmp(cmdv[0], "engine", strlen(cmdv[0])) )
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
      while( game.isPlaying() )
      {
        usleep(500000);
        rc = engine.getEnginesMove(move, true);
        cout << "e: " << move << endl;
        rc = game.sync(move);
        cout << "g: " << move << endl;
        cout << endl;
      }
    }

    // RDK: add other ROS requests here

    else
    {
      printf("%s: unknown command.\n", cmdv[0]);
    }
  }
}

static void init_name_maps()
{
  NameColors[NoColor]     = "nocolor";
  NameColors[White]       = "white";
  NameColors[Black]       = "black";

  NamePieces[NoPiece]     = "nopiece";
  NamePieces[King]        = "King";
  NamePieces[Queen]       = "Queen";
  NamePieces[Rook]        = "Rook";
  NamePieces[Bishop]      = "Bishop";
  NamePieces[Knight]      = "Knight";
  NamePieces[Pawn]        = "Pawn";

  NameCastling[NoCastle]  = "nocastle";
  NameCastling[KingSide]  = "kingside";
  NameCastling[QueenSide] = "queenside";

  NameResults[NoResult]   = "noresult";
  NameResults[Ok]         = "ok";
  NameResults[BadMove]    = "badmove";
  NameResults[OutOfTurn]  = "outofturn";
  NameResults[Checkmate]  = "checkmate";
  NameResults[Draw]       = "draw";
  NameResults[Resign]     = "resign";
  NameResults[NoGame]     = "nogame";
  NameResults[GameFatal]  = "gamefatal";
}


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

string chess_engine::nameOfColor(ChessColor color)
{
  return NameColors[color];
}

string chess_engine::nameOfPiece(ChessPiece piece)
{
  return NamePieces[piece];
}

string chess_engine::nameOfCastling(ChessCastling side)
{
  return NameCastling[side];
}

string chess_engine::nameOfResult(ChessResult result)
{
  return NameResults[result];
}

int main(int argc, char *argv[])
{
  init_name_maps();

  cli_test(argc, argv);

  return 0;
}
