////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// ROS Node:  chess_server
//
// File:      chess_as.cpp
//
/*! \file
 *
 * \brief The ROS chess_server action servers container implementation.
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

#include <sstream>
#include <string>
#include <map>

#include <boost/bind.hpp>

#include "ros/ros.h"

// chess engine library
#include "chess_engine/ceTypes.h"

// chess_server
#include "chess_server.h"
#include "chess_as.h"
#include "chess_as_autoplay.h"
#include "chess_as_cem.h"
#include "chess_as_mam.h"

using namespace std;
using namespace chess_server;

/*!
 * \brief Registered action server names.
 */
static const string AsNameAutoPlay("auto_play_action");
static const string AsNameComputeEnginesMove("compute_engines_move_action");
static const string AsNameMakeAMoveAN("make_a_move_an_action");

/*
 * \brief Helpful values.
 */
static const int CS_OK = chess_engine::CE_OK; ///< a okay


//------------------------------------------------------------------------------
// ChessActionServers Class
//------------------------------------------------------------------------------

ChessActionServers::ChessActionServers(ChessServer &chess_server) :
  m_asAutoPlay(AsNameAutoPlay, chess_server),
  m_asComputeEnginesMove(AsNameComputeEnginesMove, chess_server),
  m_asMakeAMoveAN(AsNameMakeAMoveAN, chess_server)
{
}

ChessActionServers::~ChessActionServers()
{
}

int ChessActionServers::start()
{
  int   cnt = 0;

  m_asAutoPlay.start();

  ++cnt;

  m_asComputeEnginesMove.start();

  ++cnt;

  m_asMakeAMoveAN.start();

  ++cnt;

  return cnt;
}
