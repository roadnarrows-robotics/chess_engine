////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceParser.h
//
/*! \file
 *
 * \brief Chess Algegraic Notation (AN) parser.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows
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

#ifndef _CE_PARSER_H
#define _CE_PARSER_H

#include  <string>

#include  "chess_engine/ceChess.h"
#include  "chess_engine/ceMove.h"

namespace chess_engine
{
  //----------------------------------------------------------------------------
  // Class ChessANParser
  //----------------------------------------------------------------------------

  /*!
   * \brief Chess Algebraic Notation string parser class.
   */
  class ChessANParser
  {
  public:

    /*!
     * \brief Default constructor.
     */
    ChessANParser();

    /*!
     * \brief Destructor.
     */
    virtual ~ChessANParser();

    /*!
     * \brief Parse AN string.
     *
     * Supported algebras:
     *  - Coordinate Algebra Notation
     *  - Standard Algebra Notation
     *
     * \param [in] strAN    Input AN string.
     * \param [out] move    Chess move.
     *
     * \return Returns true on success, false otherwise.
     */
    bool parse(const std::string &strAN, ChessMove &move);

    /*!
     * \brief Get parse error string.
     *
     * If a parse fails, an error string will be set to indicate the error.
     *
     * \return String.
     */
    std::string getErrorStr()
    {
      return m_strError;
    }

    /*!
     * \brief Enable/disable limited parse tracing.
     *
     * Tracing will be written to stdout.
     *
     * \param bState  Enable(true) or disable(false).
     */
    void setTracing(bool bState)
    {
      m_bTrace = bState;
    }

  protected:
    std::string   m_strError; ///< error string
    bool          m_bTrace;   ///< trace enable/disable state

    /*!
     * \brief Post-process parsed AN.
     *
     * Any additional move fields are set if possible and cross-check
     * verification is performed.
     *
     * \return Returns true on success, false otherwise.
     */
    bool postprocess();

    /*!
     * \brief Post-process castling move.
     *
     * Without piece color, only source and destinaion files are known.
     * 
     * \return Returns true on success, false otherwise.
     */
    bool postprocCastling();

    /*!
     * \brief Post-process en passant move.
     *
     * Without game state, only the source rank can be determined.
     * 
     * \return Returns true on success, false otherwise.
     */
    bool postprocEnPassant();

    /*!
     * \brief Post-process pawn promotion move.
     *
     * Without game state the source can only be partially determined.
     * 
     * \return Returns true on success, false otherwise.
     */
    bool postprocPromotion();

    /*!
     * \brief Clear parse errors.
     *
     * \return Returns true on success, false otherwise.
     */
    void clearErrors();

  private:
    void         *m_impl;   ///< private implementation interface

  }; // class ChessANParser

} // namespace chess_engine


#endif // _CE_PARSER_H
