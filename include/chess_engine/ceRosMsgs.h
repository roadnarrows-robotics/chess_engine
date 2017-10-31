////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceRosMsgs.h
//
/*! \file
 *
 * \brief Conversion between library objects and ROS messages.
 *
 * Note that all functions are defined as statics in this header. Since
 * libchessengine is linked against chess applications (e.g. chess_server),
 * it must be built first. However, chess_server package contain the
 * definitions of the messages.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016-2017  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 *
 * \par License:
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

#ifndef _CE_ROS_MSGS_H
#define _CE_ROS_MSGS_H

#include <stdint.h>

#include "chess_server/ChessColor.h"
#include "chess_server/ChessPiece.h"
#include "chess_server/ChessCastling.h"
#include "chess_server/ChessCheckMod.h"
#include "chess_server/ChessPos.h"
#include "chess_server/ChessResult.h"
#include "chess_server/ChessMove.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceMove.h"

namespace chess_engine
{
  // ---------------------------------------------------------------------------
  // Inliners to convert simple chess engine types to chess server messages.
  // ---------------------------------------------------------------------------

  /*!
   * \brief Make chess color message.
   *
   * \param eColor  Chess engine enum.
   *
   * \return ROS message.
   */
  static inline chess_server::ChessColor msg(const ChessColor &eColor)
  {
    chess_server::ChessColor msg;
    msg.color = (uint8_t)eColor;
    return msg;
  }

  /*!
   * \brief Make chess piece message.
   *
   * \param ePiece  Chess engine enum.
   *
   * \return ROS message.
   */
  static inline chess_server::ChessPiece msg(const ChessPiece &ePiece)
  {
    chess_server::ChessPiece msg;
    msg.piece = (uint8_t)ePiece;
    return msg;
  }

  /*!
   * \brief Make castling message.
   *
   * \param eCastling Chess engine enum.
   *
   * \return ROS message.
   */
  static inline chess_server::ChessCastling msg(const ChessCastling &eCastling)
  {
    chess_server::ChessCastling msg;
    msg.side = (uint8_t)eCastling;
    return msg;
  }

  /*!
   * \brief Make chess position message.
   *
   * \param pos   Chess engine struct.
   *
   * \return ROS message.
   */
  static chess_server::ChessPos msg(const ChessPos &pos)
  {
    chess_server::ChessPos msg;

    msg.file  = (uint8_t)pos.m_file;
    msg.rank  = (uint8_t)pos.m_rank;

    return msg;
  }

  /*!
   * \brief Make chess check modifier message.
   *
   * \param eCheck Chess engine enum.
   *
   * \return ROS message.
   */
  static inline chess_server::ChessCheckMod msg(const ChessCheckMod &eCheck)
  {
    chess_server::ChessCheckMod msg;
    msg.mod = (uint8_t)eCheck;
    return msg;
  }

  /*!
   * \brief Make chess result message.
   *
   * \param eResult Chess engine enum.
   *
   * \return ROS message.
   */
  static inline chess_server::ChessResult msg(const ChessResult &eResult)
  {
    chess_server::ChessResult msg;
    msg.code = (uint8_t)eResult;
    return msg;
  }


  // ---------------------------------------------------------------------------
  // Inliners to convert simple chess server messages to chess engine types.
  // ---------------------------------------------------------------------------

  /*!
   * \brief Translate color message to type.
   *
   * \param msg  Chess server message.
   *
   * \return Chess engine type.
   */
  static inline ChessColor tr(const chess_server::ChessColor &msg)
  {
    return (ChessColor)msg.color;
  }

  /*!
   * \brief Translate piece message to type.
   *
   * \param msg  Chess server message.
   *
   * \return Chess engine type.
   */
  static inline ChessPiece tr(const chess_server::ChessPiece &msg)
  {
    return (ChessPiece)msg.piece;
  }

  /*!
   * \brief Translate castling message to type.
   *
   * \param msg  Chess server message.
   *
   * \return Chess engine type.
   */
  static inline ChessCastling tr(const chess_server::ChessCastling &msg)
  {
    return (ChessCastling)msg.side;
  }

  /*!
   * \brief Translate position message to type.
   *
   * \param msg  Chess server message.
   *
   * \return Chess engine type.
   */
  static inline ChessPos tr(const chess_server::ChessPos &msg)
  {
    ChessPos  pos;

    pos.m_file = (ChessFile)msg.file;
    pos.m_rank = (ChessRank)msg.rank;

    return pos;
  }

  /*!
   * \brief Translate check modifier message to type.
   *
   * \param msg  Chess server message.
   *
   * \return Chess engine type.
   */
  static inline ChessCheckMod tr(const chess_server::ChessCheckMod &msg)
  {
    return (ChessCheckMod)msg.mod;
  }

  /*!
   * \brief Translate result message to type.
   *
   * \param msg  Chess server message.
   *
   * \return Chess engine type.
   */
  static inline ChessResult tr(const chess_server::ChessResult &msg)
  {
    return (ChessResult)msg.code;
  }


  // ---------------------------------------------------------------------------
  // Copy chess engine types to/from to chess server messages.
  // ---------------------------------------------------------------------------

  /*!
   * \brief Copy chess position object to ROS message object.
   *
   * \param pos     Source chess position object.
   * \param msg     Destination ROS message object.
   */
  static void copyPosToMsg(const ChessPos &pos, chess_server::ChessPos &msg)
  {
    msg.file  = (uint8_t)pos.m_file;
    msg.rank  = (uint8_t)pos.m_rank;
  }

  /*!
   * \brief Copy chess position object to ROS message object.
   *
   * \param pos     Source chess position object.
   * \param msg     Destination ROS message object.
   */
  static void copyMsgToPos(const chess_server::ChessPos &msg, ChessPos &pos)
  {
    pos.m_file = (ChessFile)msg.file;
    pos.m_rank = (ChessRank)msg.rank;
  }

  /*!
   * \brief Copy chess move object to ROS message object.
   *
   * \param move    Source chess move object.
   * \param msg     Destination ROS message object.
   */
  static void copyMoveToMsg(const ChessMove &move, chess_server::ChessMove &msg)
  {
    msg.move_num              = move.m_nMoveNum;
    msg.player.color          = (uint8_t)move.m_ePlayer;
    msg.AN                    = move.m_strAN;
    copyPosToMsg(move.m_posSrc, msg.src);
    copyPosToMsg(move.m_posDst, msg.dst);
    msg.moved.piece           = (uint8_t)move.m_ePieceMoved;
    msg.captured.piece        = (uint8_t)move.m_ePieceCaptured;
    msg.promoted.piece        = (uint8_t)move.m_ePiecePromoted;
    msg.en_passant            = move.m_bIsEnPassant;
    msg.castling.side         = (uint8_t)move.m_eCastling;
    msg.check.mod             = (uint8_t)move.m_eCheck;
    msg.result.code           = (uint8_t)move.m_eResult;
  }

  /*!
   * \brief Copy ROS message object to chess move object.
   *
   * \param msg     Source ROS message object.
   * \param move    Destination chess move object.
   */
  static void copyMsgToMove(const chess_server::ChessMove &msg, ChessMove &move)
  {
    move.m_nMoveNum       = msg.move_num;
    move.m_ePlayer        = (ChessColor)msg.player.color;
    move.m_strAN          = msg.AN;
    copyMsgToPos(msg.src, move.m_posSrc);
    copyMsgToPos(msg.dst, move.m_posDst);
    move.m_ePieceMoved    = (ChessPiece)msg.moved.piece;
    move.m_ePieceCaptured = (ChessPiece)msg.captured.piece;
    move.m_ePiecePromoted = (ChessPiece)msg.promoted.piece;
    move.m_bIsEnPassant   = msg.en_passant;
    move.m_eCastling      = (ChessCastling)msg.castling.side;
    move.m_eCheck         = (ChessCheckMod)msg.check.mod;
    move.m_eResult        = (ChessResult)msg.result.code;
  }

} // namespace chess_engine


#endif // _CE_ROS_MSGS_H
