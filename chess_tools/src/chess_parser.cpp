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
 * (C) 2016-2017  RoadNarrows
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "rnr/appkit/CmdExtArg.h"
#include "rnr/appkit/CommandLine.h"
#include "rnr/appkit/CmdAddOns.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceParser.h"

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
  "chess_parser",
  "1.0.0",
  "2017.11.06 10:11:09",
  "2017",
  "chess_parser-1.0.0",
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
  "A chess engine move parser.",

  // long_desc = 
  "The %P command parses chess Algebraic Notation moves. The notation can be "
  "in either Cannonical or Standard notation.",

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
// The CommandLine Interface.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

//
// Command line interface.
//
const string    CliName("ChessParser");     ///< CLI name 
const string    CliPrompt("AN> ");          ///< CLI prompt
CommandLine     Cli(CliName, CliPrompt);    ///< the CLI

/*!
 * \{
 * \brief Error and warning printing macros.
 */
#define PERROR(_err) \
  cout << CliName << ": " << _err << endl

#define PWARN(_warn) \
  cout << CliName << ": " << _warn << endl

#define PCMDERROR(_cmd, _err) \
    cout << CliName << ": '" << _cmd << "': " << _err << endl

#define PCMDWARN(_cmd, _warn) \
    cout << CliName << ": '" << _cmd << "': " << _warn << endl
/*!
 * \}
 */

//
// State
//
int           MoveNum;
ChessColor    PlayersTurn;
ChessANParser Parser;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// CommandLine Commands Definitions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Execute 'set' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execSetParser(const CmdExtArgVec &argv)
{
  if( argv[1].s() == "bnf" )
  {
    Parser.setParserMethod(ChessANParser::MethodBNF);
  }
  else if( argv[1].s() == "regex" )
  {
    Parser.setParserMethod(ChessANParser::MethodRegEx);
  }
  else
  {
    PCMDERROR(argv[0].s(), "Unknown parser '" << argv[1].s() << "'");
    return RC_ERROR;
  }

  return OK;
}

/*!
 * \brief Execute 'get' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execGetParser(const CmdExtArgVec &argv)
{
  switch( Parser.getParserMethod() )
  {
    case ChessANParser::MethodBNF:
      cout << "bnf" << endl;
      break;
    case ChessANParser::MethodRegEx:
      cout << "regex" << endl;
      break;
    default:
      cout << "?" << endl;
      break;
  }

  return OK;
}

/*!
 * \brief Execute 'run' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execRunFile(const CmdExtArgVec &argv)
{
  string  filename = argv[1].s();

  cout << "Running " << filename << endl;

  return OK;
}

/*!
 * \brief Execute 'trace' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execTrace(const CmdExtArgVec &argv)
{
  // toggle
  bool trace = Parser.getTracing()? false: true;

  Parser.setTracing(trace);

  cout << "Tracing " << (trace? "enabled": "disabled") << "." << endl;

  return OK;
}

/*!
 * \brief Execute '<AN>' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execParse(const CmdExtArgVec &argv)
{
  ChessANDecomp decomp;
  ChessMove     move;

  const string &strAN = argv[0].s();

  move.m_nMoveNum = MoveNum;
  move.m_ePlayer  = PlayersTurn;

  if( Parser.parse(strAN, PlayersTurn, decomp) == CE_OK )
  {
    // redundant if tracing enabled.
    if( !Parser.getTracing() )
    {
      cout << "parsed decomposition " << decomp << endl;
    }

    // add more tracing info
    else
    {
      Parser.toChessMove(decomp, move);
      cout << "chess move " << move << endl;
    }

    PlayersTurn = opponent(PlayersTurn);

    if( PlayersTurn == White )
    {
      Cli.popPrompt();
      ++MoveNum;
    }
    else
    {
      Cli.pushPrompt("black> ");
    }
  }
  else
  {
    PERROR(Parser.getErrorStr());
  }

  return OK;
}

/*!
 * \brief Command description and exectuion structure.
 */
struct AppCmds
{
  CmdDesc       m_desc;     ///< command description and syntax specification
  CmdExec2Func  m_fnExec;   ///< command execution function
};

/*!
 * \brief The command descriptions.
 */
AppCmds Commands[] =
{
  { { "set",
      "set {bnf | regex}",
      "Set the chess engine's Algebraic Notation (AN) parser.",
      "Set the chess engine's Algebraic Notation (AN) parser.\n"
      "Supported parsers:\n"
      "  bnf    The parser is specified in the Backus–Naur Form using the\n"
      "         Boost Spirit library. (Default)\n"
      "  regex  The parser is specified as a series of regular expressions\n"
      "         using the Boost::regex features. (Experimental)"
    },
    execSetParser
  },
  
  { { "get",
      "get",
      "Get the active chess engine's Algebraic Notation (AN) parser."
    },
    execGetParser
  },

  { { "run",
      "run <filename:file>",
      "Run commands from a file. TBD",
      "Run AN parser commands from a file.\n"
      "  <filename>  Name of file holding the commands."
    },
    execRunFile
  },

  { { "trace",
      "trace",
      "Toggle parser tracing."
    },
    execTrace
  },

  { { "AN",
      "<AN:re(^[a-h1-8QKBNRPxe\\.p=/0O-]+)>",
      "Parse Algebraic Notation string.",
      "Parse Algebraic Notation string. Can be specified in either Coordinate\n"
      " or Standard Algebraic Notation."
    },
    execParse
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
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
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
  // Add command to command-line interface.
  //
  for(size_t i = 0; i < NumOfCmds; ++i)
  {
    nUid = cli.addCommand(Commands[i].m_desc, Commands[i].m_fnExec);

    if( nUid == NoUid )
    {
      PERROR("Failed to add command '" << Commands[i].m_desc.name << "'.");
      return RC_ERROR;
    }
  }

  //
  // Add built-in command to interface.
  //
  addons::addHelpCommand(cli);
  addons::addQuitCommand(cli);

  //
  // Compile command-line interface.
  //
  if( (rc = cli.compile()) != OK )
  {
    PERROR("Compile failed.");

    // backtrace problem
    cerr << "(backtrace)" << endl;
    cli.backtrace(cerr, true);
  }

  return rc;
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

  // state
  MoveNum     = 1;
  PlayersTurn = White;
  cli.pushPrompt("white> ");

  while( cli.ok() )
  {
    rc = cli.readCommand(argv);

    if( rc == OK )
    {
      // see the results of a good command match
      //cli.backtrace(cerr);

      if( argv.size() > 0 ) 
      {
        rc = cli.execute(argv);

        if( rc == OK )
        {
          cli.addToHistory(argv);
        }
      }
    }
    else
    {
      PERROR(cli.getErrorStr());
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
  mainInit(argc, argv);

  if( loadCommands(Cli) != OK )
  {
    PERROR("Failed to load commands.");
    return APP_EC_EXEC;
  }

  // debug
  //cerr << Cli << endl;

  string delim(80, '/');

  cout << delim << endl << endl;
  cout << "\t\t\tAlgebraic Notation Parser" << endl << endl;
  cout << delim << endl << endl;
  cout << "Enter CAN or SAN (enter 'quit' to quit, 'help' for help)" << endl;

  if( run(Cli) != OK )
  {
    PERROR("Failed to run commands.");
    return APP_EC_EXEC;
  }

  cout << endl << "bye" << endl;

  return APP_EC_OK;
}
