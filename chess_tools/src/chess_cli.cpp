////////////////////////////////////////////////////////////////////////////////
//
// Package:     RoadNarrows Robotics ROS Chess Engine Package
//
// Link:        https://github.com/roadnarrows-robotics/chess_engine
//
// Application: chess_cli
//
// File:        chess_cli.cpp
//
/*! \file
 *
 * \brief Chess command-line interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2017  RoadNarrows
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
#include <iostream>
#include <sstream>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"
#include "rnr/color.h"

#include "rnr/appkit/LogStream.h"
#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/Time.h"
#include "rnr/appkit/CmdExtArg.h"
#include "rnr/appkit/CommandLine.h"
#include "rnr/appkit/CmdAddOns.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceChess.h"
#include "chess_engine/ceGame.h"
#include "chess_engine/ceMove.h"

using namespace std;
using namespace rnr;
using namespace rnr::cmd;
using namespace chess_engine;


/*!
 * \ingroup apps
 * \defgroup appkit_eg rnr_eg_cli
 * \{
 */

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Application
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;      ///< the command

/*!
 * \brief Package information.
 */
static PkgInfo_T PkgInfo =
{
  "chess_cli",
  "1.0.0",
  "2017.12.06 10:53:13",
  "2017",
  "chess_cli-1.0.0",
  "Robin Knight (robin.knight@roadnarrows.com)",
  "RoadNarrows LLC",
  "(C) 2017 RoaddNarrows LLC\n"
  "MIT License"
};

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "A chess game command-line interface.",

  // long_desc = 
  "The %P command plays chess through the libchessengine interface.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  {NULL, }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Utilities
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Horizontal rule.
 *
 * \param os        Output stream.
 * \param winwidth  Window width (columns).
 */
static void hr(ostream &os, size_t winwidth=80)
{
  string  gray(ANSI_BG_CYAN);
  string  cyan(ANSI_BG_WHITE);
  string  reset(ANSI_COLOR_RESET);

  size_t  w = winwidth/2;

  for(size_t i = 0; i < w; ++i)
  {
    if( i & 0x01 )
    {
      cout << gray << "  ";
    }
    else
    {
      cout << cyan << "  ";
    }
  }
  cout << reset << endl;
}

/*!
 * \brief Center some text.
 *
 * \param os        Output stream.
 * \param text      (Multiple line) text to center.
 * \param winwidth  Window width (columns).
 */
static void centertext(ostream &os, string text, size_t winwidth=80)
{
  str::StringVec lines;

  size_t  ct = winwidth / 2;
  size_t  w;

  str::split(text, '\n', lines);

  for(size_t i = 0; i < lines.size(); ++i)
  {
    w = ct + lines[i].size() / 2;
    os << setw(w) << lines[i] << endl;
  }
}

/*!
 * \brief Center text helper macro.
 *
 * \param _os   Output stream.
 * \param _args (Multiple) insertion objects.
 */
#define CENTER(_os, _args) \
  do \
  { \
    stringstream ss; \
    ss << _args; \
    centertext(_os, ss.str()); \
  } while(0)


/*!
 * \brief Print starting banner.
 */
static void banner()
{
  //string  delim(80, '/');
  string title("The Game Of Chess");
  string desc("A command-line interface using the non-ROS chessengine "
              "library.");
  string  helpcue("('help' for help, 'quit' to quit)");

  // old ascii horizontal line
  //cout << delim << endl << endl;

  // upper horizontal line
  hr(cout);

  // spacer
  cout << endl;

  // title
  centertext(cout, title);

  // spacer
  cout << endl;

  // short description 
  centertext(cout, desc);

  // help cue
  centertext(cout, helpcue);

  // spacer
  cout << endl;

  // lower horizontal line
  hr(cout);

  // old ascii horizontal line
  //cout << delim << endl << endl;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// The Game of Chess Data Section Layout
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

typedef map<string, ChessPlayer>    PlayerMap;    // name-player map
typedef map<ChessPlayerId, string>  IdToNameMap;  // id-name map
typedef map<ChessColor, string>     ColorMap;     // color-string map

// fixed prompt strings
const string PromptNoGame("nogame> ");
const string PromptEndOfGame("endofgame> ");

//
// Class to hold chess meta-data used by command-line.
//
class GameOfChess
{
public:
  Chess         m_chess;      // the game
  bool          m_canMove;    // you, yes you, can make a move
  ChessColor    m_turn;       // whose turn is it to move
  ColorMap      m_players;    // player names
  ColorMap      m_prompt;     // cli player prompts
  ChessResult   m_result;     // result of last move or game state
  bool          m_autoshow;   // do [not] auto-display board after each play
  bool          m_trace;      // debug tracing on or off 

  PlayerMap     m_pool;       // pool of players
  IdToNameMap   m_idToName;   // player id to name mapping
  ChessPlayerId m_idCounter;  // new player id assignment counter

  /*!
   * \brief Get the player associated with id/name in the pool.
   *
   * \param   idname  String value of player's name or integer string of
   *                  player's identification.
   *
   * \return Reference to ChessPlayer. If player's id is NoPlayerId, then the
   * player wasn't found.
   */
  ChessPlayer &getPlayer(const string idname)
  {
    PlayerMap::iterator pos;

    // try name first to find the player in the pool
    if( (pos = m_pool.find(idname)) != m_pool.end() )
    {
      return pos->second;
    }

    long val;

    // convert to long
    if( str::tolong(idname, val) != OK )
    {
      return ChessPlayer::noplayer();
    }
    
    ChessPlayerId id = (ChessPlayerId)val;

    IdToNameMap::iterator idpos;

    // try to find player's id
    if( (idpos = m_idToName.find(id)) == m_idToName.end() )
    {
      return ChessPlayer::noplayer();
    }

    // now try again to find player in the pool
    if( (pos = m_pool.find(idpos->second)) != m_pool.end() )
    {
      return pos->second;
    }

    else
    {
      return ChessPlayer::noplayer();
    }
  }

  void addPlayer(const string name, ChessPlayerType type = PlayerTypeAnon)
  {
    m_pool[name] = ChessPlayer(m_idCounter, name, type);
    m_idToName[m_idCounter++] = name;
  }

  /*!
   * \brief Clear prompt stack.
   *
   * The stack is cleared up to, but not including, the 'no game' prompt.
   *
   * \param cli   Command-line interface.
   */
  void clearPromptStack(CommandLine &cli)
  {
    string curPrompt(cli.getPrompt());
  
    // pop prompts until reached game of chess base prompt
    while( cli.getPrompt() != PromptNoGame )
    {
      cli.popPrompt();
  
      // oops, bottomed out
      if( cli.getPrompt() == curPrompt )
      {
        cli.pushPrompt(PromptNoGame);
        break;
      }
  
      curPrompt = cli.getPrompt();
    }
  }
  
  /*!
   * \brief Initialize the game of chess meta-data.
   *
   * \param cli   Command-line interface.
   */
  void init(CommandLine &cli)
  {
    string name;

    m_canMove   = false;
    m_turn      = NoColor;
    m_result    = NoResult;
    m_autoshow  = false;
    m_trace     = false;

    m_idCounter   = 10;

    //
    // Anonymous players
    //
    addPlayer("white", PlayerTypeAnon);
    addPlayer("black", PlayerTypeAnon);

    //
    // Sweet Motown players
    //

    // Diana Ross
    addPlayer("ross", PlayerTypeHuman);

    // Wilson Pickett
    addPlayer("pickett", PlayerTypeHuman);

    // Marvin Gaye
    addPlayer("gaye", PlayerTypeHuman);
  }
  
  /*!
   * \brief Mark the game of chess meta-data for 'no game'.
   *
   * \param cli   Command-line interface.
   */
  void markNoGame(CommandLine &cli)
  {
    m_canMove = false;
    m_turn    = NoColor;
    m_result  = NoResult;
  
    clearPromptStack(cli);
  }
  
  /*!
   * \brief Mark the game of chess meta-data for 'new game'.
   *
   * \param cli           Command-line interface.
   * \param nameOfWhite   White player's name.
   * \param nameOfBlack   Black player's name.
   */
  void markNewGame(CommandLine &cli, string nameOfWhite, string nameOfBlack)
  {
    m_canMove = true;
    m_turn    = White;
  
    m_players[White] = nameOfWhite;
    m_players[Black] = nameOfBlack;

    m_prompt[White] = m_players[White] + "(white): ";
    m_prompt[Black] = m_players[Black] + "(black): ";
  
    cli.pushPrompt(m_prompt[m_turn]);
  }
  
  /*!
   * \brief Mark the game of chess meta-data for 'game ended'.
   *
   * \param cli   Command-line interface.
   */
  void markGameEnded(CommandLine &cli)
  {
    m_canMove = false;

    clearPromptStack(cli);
  
    cli.pushPrompt(PromptEndOfGame);
  }
  
  /*!
   * \brief Mark the game of chess meta-data for 'game ended'.
   *
   * \param cli     Command-line interface.
   * \param byUser  Move was [not] made by the user.
   * \param result  Command-line interface.
   */
  void markMoveMade(CommandLine &cli, bool byUser, ChessResult &result)
  {
    m_result = result;
  
    switch( m_result )
    {
      case Checkmate:
      case Draw:
      case Resign:
      case Disqualified:
      case GameFatal:
        markGameEnded(cli);
        break;
      case Ok:
        m_canMove = !byUser;
        m_turn    = m_chess.whoseTurn();
        cli.popPrompt();
        cli.pushPrompt(m_prompt[m_turn]);
        break;
    }
  }
}; // class GameOfChess


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// The CommandLine Interface.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! command-line data section namespace */
const string DataSectNsGoC("gameofchess");

/*!
 * \brief Retrieve the "gameofchess" section data from command-line interface.
 *
 * \param cli   Command-line interface.
 *
 * \return Reference to GameOfChess object.
 */
static inline GameOfChess &gocds(CommandLine &cli)
{
  *(GameOfChess *)cli.getDataSection(DataSectNsGoC);
}

/*!
 * \{
 * \brief Error and warning printing macros.
 */
#define PERROR(_cli, _err) \
  cout << _cli.getName() << ": Error: " << _err << endl

#define PWARN(_cli, _warn) \
  cout << _cli.getName() << ": Warning: " << _warn << endl

#define PCMDERROR(_cmd, _err) \
    cout << "Error: " << _cmd << ": " << _err << endl

#define PCMDWARN(_cmd, _warn) \
    cout << "Warning: " << _cmd << ": " << _warn << endl
/*!
 * \}
 */

static void printMove(GameOfChess &goc, ChessMove &move)
{
  if( goc.m_trace )
  {
    cerr << endl << "move " << move << endl;
  }

  cout << setw(3) << std::right << move.m_nMoveNum << ". "
      << nameOfColor(move.m_ePlayer) << " "
      << move.m_strAN << endl;
}

static void printMoveResult(GameOfChess &goc, ChessResult result)
{
  ChessColor eWinner  = goc.m_chess.getWinner();
  ChessColor eLoser   = opponent(eWinner);

  switch( result )
  {
    case OutOfTurn:
      cout << "Don't be an ass, it's not your turn." << endl;
      break;

    case BadMove:
      cout << "Bad move, tiger." << endl;
      break;

    case Checkmate:
      cout  << "Player " << goc.m_players[eLoser]
            << " playing " << nameOfColor(eLoser)
            << " has been checkmated."
            << endl
            << goc.m_players[eWinner]
            << " playing " << nameOfColor(eWinner)
            << " wins the game."
            << endl;
      break;

    case Draw:
      cout << "Game is a draw." << endl;
      break;

    case Resign:
      cout  << "Player " << goc.m_players[eLoser]
            << " playing " << nameOfColor(eLoser)
            << " has resigned."
            << endl
            << goc.m_players[eWinner]
            << " playing " << nameOfColor(eWinner)
            << " is the winner."
            << endl;
      break;

    case Disqualified:
      cout  << "Player " << goc.m_players[eLoser]
            << " playing " << nameOfColor(eLoser)
            << " has been disqualified."
            << endl
            << goc.m_players[eWinner]
            << " playing " << nameOfColor(eWinner)
            << " wins."
            << endl;
      break;

    case Ok:
      break;

    case GameFatal:
    default:
      cout << "Error: " << nameOfResult(result) << "." << endl;
      break;
  }
}

static string gameOutcome(GameOfChess &goc,
                          ChessColor  winner,
                          ChessResult result)
{
  ChessColor    loser = opponent(winner);
  stringstream  ss;

  switch( result )
  {
    case OutOfTurn:
    case BadMove:
    case Ok:
      ss << nameOfResult(result) << ": Not an end-of-game result";
      break;

    case Checkmate:
      ss << nameOfColor(winner) << " checkmated " << nameOfColor(loser);
      break;

    case Draw:
      ss << "Game is a draw";
      break;

    case Resign:
      ss  << nameOfColor(loser) << " resigned";
      break;

    case Disqualified:
      ss  << nameOfColor(loser) << " was disqualified";
      break;

    case Aborted:
    case GameFatal:
    default:
      ss << "Game was aborted";
      break;
  }

  return ss.str();
}

static int showHistory(GameOfChess &goc, const string &cmd)
{
  const ChessGame::ChessHistory &history =
                                        goc.m_chess.getGame().getGameHistory();

  bool    bFig =  goc.m_chess.getBoard().getGraphicState();
  string  strFig(" ");

  ChessGame::ChessHistory::const_iterator iter;

  if( goc.m_trace )
  {
    for(iter = history.begin(); iter != history.end(); ++iter)
    {
      cout << *iter << endl;
    }
  }
  else
  {
    // header
    cout  << setw(5) << ""
          << setw(12) << std::left << "white"
          << setw(12) << std::left << "black";

    for(iter = history.begin(); iter != history.end(); ++iter)
    {
      const ChessMove &move = *iter;

      if( bFig )
      {
        strFig = figurineOfPiece(move.m_ePlayer, move.m_ePieceMoved);
      }

      if( move.m_ePlayer == White )
      {
        cout << endl;
        cout << setw(3) << std::right << move.m_nMoveNum << ". ";
        if( bFig )
        {
          cout << strFig << " ";
          cout << setw(10) << std::left << move.m_strAN;
        }
        else
        {
          cout << setw(12) << std::left << move.m_strAN;
        }
      }
      else
      {
        if( bFig )
        {
          cout << strFig << " ";
          cout << setw(10) << std::left << move.m_strAN;
        }
        else
        {
          cout << setw(12) << std::left << move.m_strAN;
        }
      }

      if( move.m_eResult != Ok )
      {
        cout << endl;
        cout << setw(5) << "";
        cout << nameOfResult(move.m_eResult);
        if( goc.m_chess.getWinner() != NoColor )
        {
          cout << " - " << nameOfColor(goc.m_chess.getWinner()) << " wins.";
        }
      }
    }
    cout << endl;
  }

  return OK;
}

static int showAvailMoves(GameOfChess &goc, const string &cmd, const string &an)
{
  ChessPos    pos(an);
  ChessSquare sq;
  ChessPiece  ePiece;
  ChessColor  eColor;
  list_of_pos positions;

  if( !pos.isSpecified() )
  {
    PCMDERROR(cmd, "Invalid board square: '" << an << "'.");
    return RC_ERROR;
  }

  sq = goc.m_chess.getBoard().at(pos);

  ePiece = sq.getPieceType();
  eColor = sq.getPieceColor();

  goc.m_chess.getBoard().findAvailMovesInGame(ePiece, eColor, pos, positions);

  cout << nameOfColor(eColor) << " " << nameOfPiece(ePiece) << ": ";

  for(size_t i = 0; i < positions.size(); ++i)
  {
    cout << positions[i] << " ";
  }

  cout << endl;

  return OK;
}

static int showCapturedPieces(GameOfChess  &goc,
                              const string &cmd,
                              const string &strColor)
{
  ChessColor ePlayer;

  if( strColor == "white" )
  {
    ePlayer = White;
  }
  else if( strColor == "black" )
  {
    ePlayer = Black;
  }
  else
  {
    PCMDERROR(cmd, "Invalid player color: '" << strColor << "'.");
    return RC_ERROR;
  }

  //
  // The captured lies in the opponent's boneyard.
  //
  const ChessGame::ChessBoneYard &captured =
            goc.m_chess.getGame().getBoneYard(ePlayer == White? Black: White);

  cout << "Captured " << nameOfColor(ePlayer) << " pieces:" << endl;

  for(size_t i = 0; i < captured.size(); ++i)
  {
    const ChessFqPiece &piece = captured[i];

    cout  << setw(3) << std::right << i << ". " 
          << setw(8) << std::left << nameOfPiece(piece.m_ePieceType)
          << " " << std::left << piece.m_strPieceId
          << endl;
  }
  cout << captured.size() << " pieces." << endl;

  return OK;
}

static int showResult(GameOfChess  &goc, const string &cmd)
{
  string    indent("  ");

  //
  // Game in progress
  //
  if( goc.m_chess.isPlayingAGame() )
  {
    string strTurn(nameOfColor(goc.m_turn));

    cout << "Game in progress." << endl;
    cout << indent << setw(20) << std::left << "whose turn: " << strTurn
      << endl;
    cout << indent << setw(20) << std::left << "in check: "
      << nameOfCheckMod(goc.m_chess.isInCheck())
      << endl;
    cout << indent << setw(20) << std::left << "last move result: "
      << nameOfResult(goc.m_chess.getLastMoveResult())
      << endl;
    cout << indent << setw(20) << std::left << "moves made: "
      << goc.m_chess.getNumOfMovesPlayed() << " ("
      << goc.m_chess.getNumOfPliesPlayed() << " plies)"
      << endl;
    cout << indent << setw(20) << std::left << "captured white: "
      << goc.m_chess.getGame().getBoneYard(Black).size()
      << endl;
    cout << indent << setw(20) << std::left << "captured black: "
      << goc.m_chess.getGame().getBoneYard(White).size()
      << endl;
    cout << indent << setw(20) << std::left << "promotions: "
      << "TBD"
      << endl;
  }

  //
  // No game has ever been played.
  //
  else if( goc.m_chess.getPlayState() == NoGame )
  {
    cout << "No game." << endl;
  }

  //
  // Last game played.
  // 
  else
  {
    string    strWinner;

    if( goc.m_chess.getWinner() != NoColor )
    {
      strWinner = nameOfColor(goc.m_chess.getWinner());
    }
    else
    {
      strWinner = "no winner";
    }

    cout << "Last game played." << endl;
    cout << indent << setw(20) << std::left << "winner: "
      << strWinner << endl;
    cout << indent << setw(20) << std::left << "end-of-game result: "
      << nameOfResult(goc.m_chess.getPlayState())
      << endl;
    cout << indent << setw(20) << std::left << "moves made: "
      << goc.m_chess.getNumOfMovesPlayed() << " ("
      << goc.m_chess.getNumOfPliesPlayed() << " plies)" << endl;
    cout << indent << setw(20) << std::left << "captured white: "
      << goc.m_chess.getGame().getBoneYard(Black).size()
      << endl;
    cout << indent << setw(20) << std::left << "captured black: "
      << goc.m_chess.getGame().getBoneYard(White).size()
      << endl;
    cout << indent << setw(20) << std::left << "promotions: "
      << "TBD"
      << endl;
  }

  return OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// CommandLine Commands Definitions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Execute about command.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execAbout(CommandLine &cli, const CmdExtArgVec &argv)
{
  string space(" ");
  string url("(https://github.com/roadnarrows-robotics/chess_engine)");
  string upsidedown("Upside down\n"
                    "Boy, you turn me\n"
                    "Inside out\n"
                    "And round and round");

  // application name and version
  CENTER(cout, PkgInfo.m_sPkgName << space << PkgInfo.m_sPkgVersion);

  // spacer
  cout << endl;

  // legal
  //CENTER(cout, PkgInfo.m_sPkgOwners);
  CENTER(cout, PkgInfo.m_sPkgDisclaimer);
  CENTER(cout, url);

  // spacer
  cout << endl;

  // motown
  CENTER(cout, upsidedown);

  return OK;
}

/*!
 * \brief Execute new game command.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execNewGame(CommandLine &cli, const CmdExtArgVec &argv)
{
  size_t      argc = argv.size();   // number of arguments
  GameOfChess &goc = gocds(cli);    // game of chess data section
  string      nameidWhite("white"); // default white player's name
  string      nameidBlack("black"); // default black player's name
  int         rc;

  // white player's name/id specified
  if( argc > 1 )
  {
    nameidWhite = argv[1].s();
  }

  // black player's name/id specified
  if( argc > 2 )
  {
    nameidBlack = argv[2].s();
  }

  // get players from pool
  ChessPlayer &white = goc.getPlayer(nameidWhite);
  ChessPlayer &black = goc.getPlayer(nameidBlack);

  // white player is not in the pool of players
  if( white.isNoPlayer() )
  {
    PCMDERROR(argv[0].s(), "Player '" << nameidWhite << "' not found.");
    return RC_ERROR;
  }

  // black player is not in the pool of players
  if( black.isNoPlayer() )
  {
    PCMDERROR(argv[0].s(), "Player '" << nameidBlack << "' not found.");
    return RC_ERROR;
  }

  // start the game
  if( (rc = goc.m_chess.startNewGame(&white, &black)) != CE_OK )
  {
    PCMDERROR(argv[0].s(), "Failed to start new game: "
        << chess_engine::strecode(rc) << ".");
    return RC_ERROR;
  }

  // mark new game state
  goc.markNewGame(cli, white.name(), black.name());

  // autoshow board
  if( goc.m_autoshow )
  {
    cout << goc.m_chess.getBoard() << endl;
  }

  return OK;
}

/*!
 * \brief Execute resign command.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execResign(CommandLine &cli, const CmdExtArgVec &argv)
{
  GameOfChess &goc = gocds(cli);  // game of chess data section

  if( !goc.m_chess.isPlayingAGame() )
  {
    cout << "No game is being played, pal." << endl;
    return RC_ERROR;
  }

  else if( !goc.m_canMove )
  {
    cout << "Hey mac, resign on your turn." << endl;
    return RC_ERROR;
  }

  int rc = goc.m_chess.resign(goc.m_turn);

  if( rc != CE_OK )
  {
    PCMDERROR(argv[0].s(), "Failed to resigned: "
        << chess_engine::strecode(rc) << ".");
    return RC_ERROR;
  }

  goc.markGameEnded(cli);

  cout << "You resigned." << endl;


  if( goc.m_autoshow )
  {
    cout << goc.m_chess.getBoard() << endl;
  }

  return OK;
}

/*!
 * \brief Get a chess parameter.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execGet(CommandLine &cli, const CmdExtArgVec &argv)
{
  GameOfChess &goc = gocds(cli);  // game of chess data section

  if( argv[1].s() == "display" )
  {
    bool on = goc.m_chess.getBoard().getGraphicState();

    cout << (on? "figurine": "ascii") << endl;
  }
  else if( argv[1].s() == "autoshow" )
  {
    cout << nameOfBool(goc.m_autoshow) << endl;
  }
  else if( argv[1].s() == "difficulty" )
  {
    cout << goc.m_chess.getGameDifficulty() << endl;
  }
  else
  {
    PCMDERROR(argv[0].s(), "Bug: Unknown parameter '" << argv[1].s() << "'");
    return RC_ERROR;
  }

  return OK;
}

/*!
 * \brief Set a chess parameter.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execSet(CommandLine &cli, const CmdExtArgVec &argv)
{
  GameOfChess &goc = gocds(cli);  // game of chess data section

  if( argv[1].s() == "display" )
  {
    if( argv[2].s() == "ascii" )
    {
      goc.m_chess.getBoard().setGraphicState(false);
    }
    else if( argv[2].s() == "figurine" )
    {
      goc.m_chess.getBoard().setGraphicState(true);
    }
    else
    {
      PCMDERROR(argv[0].s(), "Bug: Unknown parameter " << argv[1].s() 
          << " value '" << argv[1].s() << "'");
      return RC_ERROR;
    }
  }
  else if( argv[1].s() == "autoshow" )
  {
    goc.m_autoshow = argv[2].b();
  }
  else if( argv[1].s() == "difficulty" )
  {
    goc.m_chess.setGameDifficulty((float)argv[2].f());
  }
  else
  {
    PCMDERROR(argv[0].s(), "Bug: Unknown parameter '" << argv[1].s() << "'");
    return RC_ERROR;
  }

  return OK;
}

/*!
 * \brief Show game state.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execShow(CommandLine &cli, const CmdExtArgVec &argv)
{
  GameOfChess &goc = gocds(cli);  // game of chess data section
  int         rc   = OK;

  if( argv[1].s() == "board" )
  {
    cout << goc.m_chess.getBoard() << endl;
  }
  else if( argv[1].s() == "history" )
  {
    rc = showHistory(goc, argv[0].s());
  }
  else if( argv[1].s() == "avail" )
  {
    rc = showAvailMoves(goc, argv[0].s(), argv[2].s());
  }
  else if( argv[1].s() == "captured" )
  {
    rc = showCapturedPieces(goc, argv[0].s(), argv[2].s());
  }
  else if( argv[1].s() == "result" )
  {
    rc = showResult(goc, argv[0].s());
  }
  else
  {
    PCMDERROR(argv[0].s(), "Bug: Unknown parameter '" << argv[1].s() << "'");
    rc = RC_ERROR;
  }

  return rc;
}

/*!
 * \brief Player commands
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execPlayer(CommandLine &cli, const CmdExtArgVec &argv)
{
  GameOfChess &goc  = gocds(cli);  // game of chess data section
  int         rc    = OK;

  // player list
  if( argv[1].s() == "list" )
  {
    IdToNameMap::iterator iter;

    cout << " id  name" << endl;
    //cout << " --  ----" << endl;

    for(iter = goc.m_idToName.begin(); iter != goc.m_idToName.end(); ++iter)
    {
      ChessPlayer &player = goc.m_pool[iter->second];
      cout << setw(3) << player.id() << ". " << player.name() << "" << endl;
    }
  }

  // player new <name>
  else if( argv[1].s() == "add" )
  {
    ChessPlayer &player = goc.getPlayer(argv[2].s());

    if( player.id() == NoPlayerId )
    {
      goc.addPlayer(argv[2].s(), PlayerTypeRobot);
    }
    else
    {
      PCMDERROR(argv[0].s(), "Player '" << argv[2].s() << "' already exists.");
      rc = RC_ERROR;
    }
  }

  // player cv <nameid>
  else if( argv[1].s() == "cv" )
  {
    ChessPlayer &player = goc.getPlayer(argv[2].s());

    if( player.id() != NoPlayerId )
    {
      cout << player << endl;
    }
    else
    {
      PCMDERROR(argv[0].s(), "Player '" << argv[2].s() << "' not found.");
    }
  }

  // player summary <nameid>
  else if( argv[1].s() == "summary" )
  {
    ChessPlayer &player = goc.getPlayer(argv[2].s());

    if( player.id() != NoPlayerId )
    {
      const PlayerSummary &summary = player.summary();

      cout << "Player " << player.name() << endl;
      cout << "  wins:   " << setw(3) << std::right << summary.wins() << endl;
      cout << "  losses: " << setw(3) << std::right << summary.losses() << endl;
      cout << "  draws:  " << setw(3) << std::right << summary.draws() << endl;
      cout << "  ------- ---" << endl;
      cout << "  games:  " << setw(3) << std::right << summary.games() << endl;
    }
    else
    {
      PCMDERROR(argv[0].s(), "Player '" << argv[2].s() << "' not found.");
    }
  }

  // player summary <nameid>
  else if( argv[1].s() == "history" )
  {
    ChessPlayer &player = goc.getPlayer(argv[2].s());

    if( player.id() != NoPlayerId )
    {
      chronos::Time tstart, tend;
      string        strWinner;

      cout << "Player " << player.name() << " History" << endl;
      for(ssize_t i = (ssize_t)player.sizeHistory() - 1; i >= 0; --i)
      {
        const GameRecord &rec = player.recordAt(i);

        rec.gameTimes(tstart, tend);
        if( rec.gameWinner() == NoColor )
        {
          strWinner = "no winner";
        }
        else
        {
          strWinner = nameOfColor(rec.gameWinner());
        }

        cout << " Game " << i + 1 << endl;
        cout << "  start:    " << tstart.calendarTime() << endl;
        cout << "  end:      " << tend.calendarTime() << endl;
        cout << "  played:   " << nameOfColor(rec.colorPlayed()) << endl;
        cout << "  opponent: " << rec.opponentInfo().name() << endl;
        cout << "  winner:   " << strWinner << endl;
        cout << "  result:   "
          << gameOutcome(goc, rec.gameWinner(), rec.gameResult()) << endl;
      }
      cout << " " << player.sizeHistory() << " records." << endl;
    }
    else
    {
      PCMDERROR(argv[0].s(), "Player '" << argv[2].s() << "' not found.");
    }
  }

  else
  {
    PCMDERROR(argv[0].s(), "Bug: Unknown parameter '" << argv[1].s() << "'");
    rc = RC_ERROR;
  }

  return OK;
}

/*!
 * \brief Execute a trace enable/disable command.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execTrace(CommandLine &cli, const CmdExtArgVec &argv)
{
  gocds(cli).m_trace = argv[1].b();

  return OK;
}

/*!
 * \brief Make a move.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execMakeAMove(CommandLine &cli, const CmdExtArgVec &argv)
{
  GameOfChess &goc = gocds(cli);  // game of chess data section

  if( !goc.m_chess.isPlayingAGame() )
  {
    cout << "Geez, do you see a game being played, cuz I sure don't." << endl;
    return RC_ERROR;
  }

  else if( !goc.m_canMove )
  {
    cout << "Kibitzer, keep your hands to yourself." << endl;
    return RC_ERROR;
  }

  ChessMove move;

  int rc = goc.m_chess.makeAMove(goc.m_turn, argv[0].s(), move); 

  if( rc != CE_OK )
  {
    PCMDERROR(argv[0].s(), chess_engine::strecode(rc) << ".");
    return RC_ERROR;
  }

  printMove(goc, move);
  printMoveResult(goc, move.m_eResult);

  goc.markMoveMade(cli, true, move.m_eResult);

  if( goc.m_autoshow )
  {
    cout << goc.m_chess.getBoard() << endl;
  }

  return OK;
}

/*!
 * \brief Let the chess engine make some moves.
 *
 * \param cli   Command line interface.
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execGo(CommandLine &cli, const CmdExtArgVec &argv)
{
  GameOfChess &goc = gocds(cli);  // game of chess data section

  int       maxPlies;
  int       numPlies;
  ChessMove move;
  int       rc;

  if( !goc.m_chess.isPlayingAGame() )
  {
    cout << "No game in play." << endl;
    return RC_ERROR;
  }

  maxPlies = argv.size() < 2? 1: argv[1].i();
  numPlies = 0;

  while( ((maxPlies == 0) || (numPlies < maxPlies)) &&
          goc.m_chess.isPlayingAGame() )
  {
    rc = goc.m_chess.computeEnginesMove(move);

    if( rc != CE_OK )
    {
      PCMDERROR("next", chess_engine::strecode(rc) << ".");
      return RC_ERROR;
    }

    printMove(goc, move);
    printMoveResult(goc, move.m_eResult);

    goc.markMoveMade(cli, false, move.m_eResult);

    if( goc.m_autoshow )
    {
      cout << goc.m_chess.getBoard() << endl;
    }

    ++numPlies;

    if( cli.kbhit() )
    {
      break;
    }
  }

  return OK;
}

/*!
 * \brief Execute a blank line.
 *
 * For chess engine moves, simply press <ENTER>.
 *
 * \param cli   Command line interface.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execBlankLine(CommandLine &cli)
{
  GameOfChess &goc = gocds(cli);  // game of chess data section

  if( !goc.m_chess.isPlayingAGame() )
  {
    return OK;
  }

  ChessMove move;

  int rc = goc.m_chess.computeEnginesMove(move);

  if( rc != CE_OK )
  {
    PCMDERROR("next", chess_engine::strecode(rc) << ".");
    return RC_ERROR;
  }

  printMove(goc, move);
  printMoveResult(goc, move.m_eResult);

  goc.markMoveMade(cli, false, move.m_eResult);

  if( goc.m_autoshow )
  {
    cout << goc.m_chess.getBoard() << endl;
  }

  return OK;
}

/*!
 * \brief Command description and exectuion structure.
 */
struct AppCmd
{
  CmdDesc       m_desc;     ///< command description and syntax specification
  CmdExec3Func  m_fnExec;   ///< command execution function
};

/*!
 * \brief The command descriptions.
 */
AppCmd Commands[] =
{
  { { "about",
      "about",
      "About this application."
    },
    execAbout
  },

  { { "newgame",
      "newgame [<white:multiword>] [<black:multiword>]",
      "Start a new chess game.",
      "Start a new game of chess where:\n"
      "  <white> - name or id of player in pool.\n"
      "  <black> - name or id of player in pool."
    },
    execNewGame
  },

  { { "resign",
      "resign",
      "Resign from the current game.",
      "Resign from the current game. A game must be in progress and it must "
      "be your turn."
    },
    execResign
  },

  { { "AN",
      "<AN:re(^[a-h1-8QKBNRPxe\\.p=/0O-]+)>",
      "Make a move.",
      "The move can be specified in either Coordinate or Standard "
      "Algebraic Notation."
    },
    execMakeAMove
  },

  { { "go",
      "go [<n:int>]",
      "Auto-play for <n> plies.",
      "The chess engine will generate the next <n> plies (half moves). "
      "One chess move is comprised of two plies: White, then Black\n"
      "  <n>  Number of plies to play. Default: 1\n\n"
      "Note: Pressing <Enter> when it's the engine's turn to play is "
      "short hand for 'go 1'."
    },
    execGo
  },

  { { "get",
      "get {autoshow | display | difficulty}",
      "Get a chess environment parameter's value."
    },
    execGet
  },

  { { "set",
      "set autoshow <tf:bool>\n"
      "set difficulty <level:fpn(1.0:10.0)>\n"
      "set display {ascii | figurine}\n"
      "set engine <name:multiword>\n"
      "set player <name:multiword>",

      "Set a chess environment parameter's value.",

      "Set a chess parameter that improves the ambience of game play.\n\n"
      "set autoshow <tf>      - Enable/disable board auto-show after moves.\n"
      "set difficulty <level> - Set the engine's difficulty level.\n"
      "                           [1.0,10.0] with 1 = the easiest level.\n"
      "set display <encode>   - Set how the chess board is displayed.\n"
      "                           ascii    - ASCII UTF-8\n"
      "                           figurine - Unicode figurines"
    },
    execSet
  },

  { { "show",

      "show {board | history | result}\n"
      "show avail <square:re([a-h][1-8])>\n"
      "show captured {white | black}",

      "Show game state.",

      "Show the state of the current game.\n\n"
      "show board             - Show the the chess board.\n"
      "show captured <player> - Show captured pieces.\n"
      "show history           - Show game history.\n"
      "show result            - Show game results.\n"
      "show avail <square>    - Show available moves."
    },
    execShow
  },

  { { "player",

      "player list\n"
      "player add <name:multiword>\n"
      "player {cv | summary | history} <nameid:multiword>",

      "Manage chess player pool.",

      "Manage the pool of chess players.\n\n"
      "player list             - List players in pool.\n"
      "player add <name>       - Add new player to pool.\n"
      "player cv <nameid>      - Dump player's full curriculum vitae.\n"
      "player summary <nameid> - Show player's game summary.\n"
      "player history <nameid> - Show player's game history.\n"
      "where:\n"
      "  <name>   - new player's unique name.\n"
      "  <nameid> - name or id of player in pool."
    },
    execPlayer
  },

  { { "trace",
      "trace <enable:bool>",
      "Enable/disable (limited) debug tracing."
    },
    execTrace
  }
};

/*!
 * \brief Number of commands.
 */
const size_t NumOfCmds = arraysize(Commands);


//------------------------------------------------------------------------------
// Main Functions
//------------------------------------------------------------------------------

/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInit(int argc, char *argv[])
{
  // name of this executable
  Argv0 = basename(argv[0]);

  // parse input options (may not return on error)
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);
}

/*!
 * \brief Load commands into command line.
 *
 * Loading involves adding all commands and then compiling.
 *
 * \param cli Command line interface.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int loadCommands(CommandLine &cli)
{
  int   nUid;
  int   rc;

  //
  // Add all application-specific commands to command-line interface.
  //
  for(size_t i = 0; i < NumOfCmds; ++i)
  {
    nUid = cli.addCommand(Commands[i].m_desc, Commands[i].m_fnExec);

    if( nUid == NoUid )
    {
      PERROR(cli, "Failed to add command '" << Commands[i].m_desc.name << "'.");
      return RC_ERROR;
    }
  }

  //
  // Add built-in commands to interface.
  //
  addons::addHelpCommand(cli);
  addons::addQuitCommand(cli);

  //
  // Compile command-line interface.
  //
  if( (rc = cli.compile()) != OK )
  {
    PERROR(cli, "Compile failed.");

    // backtrace the problem
    cerr << "(backtrace)" << endl;
    cli.backtrace(cerr, true);
  }

  return rc;
}

/*!
 * \brief Initialize the game of chess.
 *
 * \param cli   Command-line interface.
 * \param goc   Game of chess.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int initChess(CommandLine &cli, GameOfChess &goc)
{
  int   rc;

  if( (rc = goc.m_chess.initialize()) != CE_OK )
  {
    PERROR(cli, chess_engine::strecode(rc));
    return RC_ERROR;
  }

  goc.init(cli);

  return OK;
}

/*!
 * \brief Command line interface main loop.
 *
 * \param cli Command line interface.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int run(CommandLine &cli)
{
  CmdExtArgVec  argv;     // vector of string input arguments
  int           rc;       // return code

  while( cli.ok() )
  {
    rc = cli.readCommand(argv);

    if( rc == OK )
    {
      // see the results of a good command match
      // cli.backtrace(cerr);

      if( argv.size() > 0 ) 
      {
        rc = cli.execute(argv);

        if( rc == OK )
        {
          cli.addToHistory(argv);
        }
      }
      else
      {
        execBlankLine(cli);
      }
    }
    else
    {
      PERROR(cli, cli.getErrorStr());
      //cli.backtrace(cout);
    }
  }

  return OK;
}

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns 0 on succes, non-zero on failure.
 */
int main(int argc, char *argv[])
{
  // initialize main
  mainInit(argc, argv);

  // show banner
  banner();
  cout << endl;

  // the command-line interface
  CommandLine cli("GameOfChess", "chess> ");

  // the data (Engine, engine, number nine... - Wilson Pickett)
  GameOfChess engine9;

  // load commands to interface
  LOGDIAG1("Loading commands to interface.");
  if( loadCommands(cli) != OK )
  {
    PERROR(cli, "Failed to load commands.");
    return APP_EC_EXEC;
  }

  // add game of chess data section to interface
  LOGDIAG1("Adding chess data section to interface.");
  if( cli.addDataSection(DataSectNsGoC, &engine9) != OK )
  {
    PERROR(cli, "Failed to add" << DataSectNsGoC << " data section.");
    return APP_EC_EXEC;
  }

  // debug
  //cerr << cli << endl;

  // initialize the game of chess
  LOGDIAG1("Initializing chess.");
  if( initChess(cli, engine9) != OK )
  {
    return RC_ERROR;
  }

  // run interface
  LOGDIAG1("Run interface.");
  if( run(cli) != OK )
  {
    PERROR(cli, "Failed to run commands.");
    return APP_EC_EXEC;
  }

  // normal exit
  cout << endl << "bye" << endl;

  return APP_EC_OK;
}
