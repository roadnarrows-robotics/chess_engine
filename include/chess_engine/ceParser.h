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

#include  "chess_engine/ceTypes.h"
#include  "chess_engine/ceMove.h"

namespace chess_engine
{
  //----------------------------------------------------------------------------
  // Struct ChessANDecomp
  //----------------------------------------------------------------------------

  /*!
   * \brief Chess Algebraic Notation parsed decomposition.
   *
   * \note Keep as struct since the Boost::Spirit library requires it.
   */
  struct ChessANDecomp
  {
    // parse source input
    std::string   m_strAN;          ///< original algebraic notation
    ChessColor    m_ePlayer;        ///< player (optional)

    // resultant parsed output decompostion (not all fields are determinable)
    ChessAlgebra  m_eAlgebra;       ///< CAN or SAN
    ChessPiece    m_ePieceMoved;    ///< moved piece type
    ChessColor    m_eColorMoved;    ///< moved piece color
    ChessPos      m_posSrc;         ///< moved piece source chess square
    ChessPos      m_posDst;         ///< moved piece destination chess square
    bool          m_bHasCaptured;   ///< [no] captured piece
    ChessPiece    m_ePieceCaptured; ///< captured piece type, if any
    ChessPiece    m_ePiecePromoted; ///< promoted piece, if any
    bool          m_bIsEnPassant;   ///< is [not] an en passant move
    ChessCastling m_eCastling;      ///< castling side, if any
    ChessCheckMod m_eCheck;         ///< opponent placed in check or mate

    /*!
     * \brief Default constructor.
     */
    ChessANDecomp();

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    ChessANDecomp(const ChessANDecomp &src);

    /*!
     * \brief Destructor.
     */
    ~ChessANDecomp() { }

    /*!
     * \brief Clear data.
     */
    void clear();


    //
    // Friends
    //

    /*!
     * \brief The ChessANDecomp output stream insertion operator.
     *
     * \brief os    Output stream object.
     * \brief obj   Object to insert.
     *
     * \return os
     */
    friend std::ostream &operator<<(std::ostream &os,const ChessANDecomp &obj);
  };


  //----------------------------------------------------------------------------
  // Class ChessANParser
  //----------------------------------------------------------------------------

  /*!
   * \brief Chess Algebraic Notation string parser class.
   * 
   * There are two parsers supported:
   *  - BNF (default)
   *  - RegEx
   *
   * The parsers infer extra state information based on the rules of chess.
   * But no board or games state is known, except for an optional hint of 
   * which player.
   */
  class ChessANParser
  {
  public:
    /*!
     * \brief AN parser method.
     */
    enum Method
    {
      MethodBNF,      ///< Backus-Naur Form parser (default)
      MethodRegEx     ///< regular expression parser
    };

    /*!
     * \brief Default constructor.
     */
    ChessANParser();

    /*!
     * \brief Destructor.
     */
    virtual ~ChessANParser();

    /*!
     * \brief Parse AN string into it's decomposition.
     *
     * Supported algebras:
     *  - Coordinate Algebra Notation
     *  - Standard Algebra Notation
     *
     * This is the parser work horse method. All other parse variations call
     * this member function.
     *
     * \param [in]  strAN   Input AN string.
     * \param [in]  ePlayer Player's color (NoColor if not known).
     * \param [out] decomp  Parsed AN decomposition.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int parse(const std::string &strAN,
              const ChessColor  ePlayer,
              ChessANDecomp     &decomp);

    /*!
     * \brief Parse AN string into it's decomposition.
     *
     * \param [in]  strAN   Input AN string.
     * \param [out] decomp  Parsed AN decomposition.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int parse(const std::string &strAN, ChessANDecomp &decomp)
    {
      return parse(strAN, NoColor, decomp);
    }

    /*!
     * \brief Parse AN string into it's decomposition and convert to chess move.
     *
     * \param [in]  strAN   Input AN string.
     * \param [in]  ePlayer Player's color (NoColor if not known).
     * \param [out] move    Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int parse(const std::string &strAN,
              const ChessColor  ePlayer,
              ChessMove         &move);

    /*!
     * \brief Parse AN string into it's decomposition and convert to chess move.
     *
     * \param [in] strAN    Input AN string.
     * \param [out] move    Chess move.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int parse(const std::string &strAN, ChessMove &move)
    {
      return parse(strAN, NoColor, move);
    }

    /*!
     * \brief Convert parsed decompostion structure to move object.
     *
     * Note that SAN parsing gives more information than CAN.
     *
     * Move Fields:
     * - piece moved
     * - source position
     * - destination position
     * - captured piece
     * - promoted piece
     * - en passant
     * - castling side
     * - check modifier
     *
     * \param [out] move    Chess move object.
     */
    void toChessMove(const ChessANDecomp &decomp, ChessMove &move);

    /*!
     * \brief Get parse error string.
     *
     * If a parse fails, an error string will be set to indicate the error.
     *
     * \return String.
     */
    std::string getErrorStr() const
    {
      return m_strError;
    }

    /*!
     * \brief Set AN parser method.
     *
     * \param eMethod   New parser method.
     *
     * \return New method if valid, current otherwise.
     */
    Method setParserMethod(Method eMethod);

    /*!
     * \brief Get AN parser method.
     *
     * \return Current method.
     */
    Method getParserMethod() const
    {
      return m_eMethod;
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

    /*!
     * \brief Get parse tracing setting.
     *
     * \return Enable(true) or disable(false).
     */
    bool getTracing() const
    {
      return m_bTrace;
    }

  protected:
    std::string   m_strError; ///< error string
    bool          m_bTrace;   ///< trace enable/disable state
    Method        m_eMethod;  ///< AN parser method

    /*!
     * \brief Run an AN parser with the grammar specified in Backus-Naur Form.
     *
     * \param [in]  strAN   Input AN string.
     * \param [out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int runParserBNF(const std::string &strAN, ChessANDecomp &decomp);

    /*!
     * \brief Run an AN parser with the grammar specified as regular
     * expressions.
     *
     * \param [in]  strAN   Input AN string.
     * \param [out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int runParserRegEx(const std::string &strAN, ChessANDecomp &decomp);

    /*!
     * \brief Parse Coordinate Algebraic Notation using regular expression
     * matching.
     *
     * \param [in]  strAN   Input AN string.
     * \param [out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int parseCANRegEx(const std::string &strAN, ChessANDecomp &decomp);

    /*!
     * \brief Parse Standard Algebraic Notation using regular expression
     * matching.
     *
     * \param [in]  strAN   Input AN string.
     * \param [out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int parseSANRegEx(const std::string &strAN, ChessANDecomp &decomp);

    /*!
     * \brief Post-process parsed AN.
     *
     * The rules of chess are applied to the syntactically parsed AN string
     * to set additional fields and for verification.
     *
     * \param[in,out] decomp  Parsed AN decomposition.
     *
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int postprocess(ChessANDecomp &decomp);

    /*!
     * \brief Post-process castling move.
     *
     * AN: 0-0 O-O 0-0-0 O-O-O
     *
     * Determine:
     * - move source rank
     * - move source file (requires known player color)
     *
     * Verify move.
     *
     * \param[in,out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int postprocCastling(ChessANDecomp &decomp);

    /*!
     * \brief Post-process en passant move.
     *
     * AN: exd6e.p cxd6e.p exd3e.p cxd3.p e5xd6e.p ...
     *
     * Determine:
     * - moved piece color
     * - source rank
     *
     * Verify move.
     *
     * \param[in,out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int postprocEnPassant(ChessANDecomp &decomp);

    /*!
     * \brief Post-process pawn promotion move.
     *
     * AN: e8=Q a1Q d7e8(N) dxe8/N ...
     *
     * Determine:
     * - moved piece color
     * - capture state (diagonal move)
     * - source rank
     * - source file (if no capture)
     *
     * Verify move.
     *
     * \param[in,out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int postprocPromotion(ChessANDecomp &decomp);

    /*!
     * \brief Post-process pawn basic move.
     *
     * AN: h4 f4f5 f4e5 ...
     *
     * Determine:
     * - moved piece color
     * - capture state (diagonal move)
     * - source rank (non first moves rank 4/5 ambiguity)
     *
     * Verify move.
     *
     * \param[in,out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int postprocPawn(ChessANDecomp &decomp);

    /*!
     * \brief Post-process major piece move.
     *
     * AN: Rh4 Bxa1 Qf4f7 Kcxd2 N3xd2 ...
     *
     * Determine:
     * - unspecified rank or file if possible
     *
     * Verify move.
     *
     * \param[in,out] decomp  Parsed AN decomposition.
     * 
     * \return Returns CE_OK on success, negative error code on failure.
     */
    int postprocMajorPiece(ChessANDecomp &decomp);

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
