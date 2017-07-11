////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceParser.cpp
//
/*! \file
 *
 * \brief Chess Algebraic Notation (AN) parser implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016-2017  RoadNarrows LLC
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

#include <boost/config/warning_disable.hpp>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>

#include <boost/fusion/include/adapt_struct.hpp>

#include <boost/regex.hpp>

#include <iostream>
#include <string>
#include <map>

#include <ros/console.h>

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceBoard.h"
#include "chess_engine/ceParser.h"

using namespace std;

// --------------------------------------------------------------------------
// Chess Algebraic Notation in Backus Naur Form
// --------------------------------------------------------------------------

//
// Adapt ChessPos structure for parser. Keep in global scope.
//
BOOST_FUSION_ADAPT_STRUCT(
    chess_engine::ChessPos,
    (chess_engine::ChessFile, m_file)   // 0
    (chess_engine::ChessRank, m_rank)   // 1
)

//
// Adapted ChessPos member access helpers.
//
#define AT_POS_FILE(_p) at_c<0>(_p)  ///< position file
#define AT_POS_RANK(_p) at_c<1>(_p)  ///< position rank

//
// Adapt ChessANDecomp structure for parser. Keep in global scope.
//
BOOST_FUSION_ADAPT_STRUCT(
    chess_engine::ChessANDecomp,
    (chess_engine::ChessAlgebra, m_eAlgebra)      // 0
    (chess_engine::ChessPiece, m_ePieceMoved)     // 1
    (chess_engine::ChessPiece, m_eColorMoved)     // 2
    (chess_engine::ChessPos, m_posSrc)            // 3
    (chess_engine::ChessPos, m_posDst)            // 4
    (bool, m_bHasCaptured)                        // 5
    (chess_engine::ChessPiece, m_ePieceCaptured)  // 6
    (chess_engine::ChessPiece, m_ePiecePromoted)  // 7
    (bool, m_bIsEnPassant)                        // 8
    (chess_engine::ChessCastling, m_eCastling)    // 9
    (chess_engine::ChessCheckMod, m_eCheck)       // 10
)

//
// Adapted ChessANDecomp member access helpers.
//
#define AT_ALGEBRA(_d)        at_c<0>(_d)  ///< algebra notation syntax
#define AT_MOVED_PIECE(_d)    at_c<1>(_d)  ///< move piece
#define At_MOVED_COLOR(_d)    at_c<2>(_d)  ///< moved piece color
#define AT_SRC_POS(_d)        at_c<3>(_d)  ///< move source position
#define AT_DST_POS(_d)        at_c<4>(_d)  ///< move destination position
#define AT_HAS_CAPTURED(_d)   at_c<5>(_d)  ///< [no] captured piece
#define AT_CAPTURED_PIECE(_d) at_c<6>(_d)  ///< captured piece
#define AT_PROMOTED(_d)       at_c<7>(_d)  ///< move promotion piece
#define AT_EN_PASSANT(_d)     at_c<8>(_d)  ///< move en passant
#define AT_CASTLING(_d)       at_c<9>(_d)  ///< move castling type
#define AT_CHECK(_d)          at_c<10>(_d)  ///< move check modifier

//
// Rule helpers.
//
#define RULE_NAME(rule) rule.name(#rule)

namespace chess_engine
{
  namespace qi = boost::spirit::qi;
  namespace ascii = boost::spirit::ascii;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Token,Value Symbols
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Chess file token,value pairs.
   */
  struct file_ : qi::symbols<char, ChessFile>
  {
    file_()
    {
      add
        ("a", ChessFileA)
        ("b", ChessFileB)
        ("c", ChessFileC)
        ("d", ChessFileD)
        ("e", ChessFileE)
        ("f", ChessFileF)
        ("g", ChessFileG)
        ("h", ChessFileH)
      ;
    }
  } file;

  /*!
   * \brief Chess rank token,value pairs.
   */
  struct rank_ : qi::symbols<char, ChessRank>
  {
    rank_()
    {
      add
        ("1", ChessRank1)
        ("2", ChessRank2)
        ("3", ChessRank3)
        ("4", ChessRank4)
        ("5", ChessRank5)
        ("6", ChessRank6)
        ("7", ChessRank7)
        ("8", ChessRank8)
      ;
    }
  } rank;

  /*!
   * \brief Chess en passant source rank token,value pairs.
   */
  struct en_passant_src_rank_ : qi::symbols<char, ChessRank>
  {
    en_passant_src_rank_()
    {
      add
        ("5", ChessRank5)   // white
        ("4", ChessRank4)   // black
      ;
    }
  } en_passant_src_rank;

  /*!
   * \brief Chess en passant destination rank token,value pairs.
   */
  struct en_passant_dst_rank_ : qi::symbols<char, ChessRank>
  {
    en_passant_dst_rank_()
    {
      add
        ("6", ChessRank6)   // white
        ("3", ChessRank3)   // black
      ;
    }
  } en_passant_dst_rank;

  /*!
   * \brief Chess penultimate rank token,value pairs.
   */
  struct penult_rank_ : qi::symbols<char, ChessRank>
  {
    penult_rank_()
    {
      add
        ("7", ChessRank7)   // white
        ("2", ChessRank2)   // black 
      ;
    }
  } penult_rank;

  /*!
   * \brief Chess ultimate rank token,value pairs.
   */
  struct ult_rank_ : qi::symbols<char, ChessRank>
  {
    ult_rank_()
    {
      add
        ("8", ChessRank8)   // white
        ("1", ChessRank1)   // black 
      ;
    }
  } ult_rank;

  /*!
   * \brief Chess piece token,value pairs.
   */
  struct piece_ : qi::symbols<char, ChessPiece>
  {
    piece_()
    {
      add
        ("K", King)
        ("Q", Queen)
        ("R", Rook)
        ("B", Bishop)
        ("N", Knight)
        ("P", Pawn)
      ;
    }
  } piece;

  /*!
   * \brief Chess major piece token,value pairs.
   */
  struct major_piece_ : qi::symbols<char, ChessPiece>
  {
    major_piece_()
    {
      add
        ("K", King)
        ("Q", Queen)
        ("R", Rook)
        ("B", Bishop)
        ("N", Knight)
      ;
    }
  } major_piece;

  /*!
   * \brief Chess promotable piece token,value pairs.
   */
  struct promotion_piece_ : qi::symbols<char, ChessPiece>
  {
    promotion_piece_()
    {
      add
        ("Q", Queen)
        ("R", Rook)
        ("B", Bishop)
        ("N", Knight)
      ;
    }
  } promotion_piece;

  /*!
   * \brief Chess castling token,value pairs.
   */
  struct castling_ : qi::symbols<char, ChessCastling>
  {
    castling_()
    {
      add
        ("0-0",   KingSide)
        ("O-O",   KingSide)
        ("0-0-0", QueenSide)
        ("O-O-O", QueenSide)
      ;
    }
  } castling;

  /*!
   * \brief Chess capture modifier token,value pairs.
   */
  struct capture_mod_ : qi::symbols<char, bool>
  {
    capture_mod_()
    {
      add
        ("x", true)
        (":", true)
      ;
    }
  } capture_mod;

  /*!
   * \brief Chess en passant modifier token,value pairs.
   */
  struct en_passant_mod_ : qi::symbols<char, bool>
  {
    en_passant_mod_()
    {
      add
        ("e.p", true);
    }
  } en_passant_mod;

  /*!
   * \brief Chess pawn promotion modifier token,value pairs.
   */
  struct promotion_mod_ : qi::symbols<char, bool>
  {
    promotion_mod_()
    {
      add
        ("=", true) 
        ("/", true)
      ;
    }
  } promotion_mod;

  /*!
   * \brief Chess check modifier token,value pairs.
   */
  struct check_mod_ : qi::symbols<char, ChessCheckMod>
  {
    check_mod_()
    {
      add
        ("+",   ModCheck)
        ("++",  ModDoubleCheck)
        ("#",   ModCheckmate)
      ;
    }
  } check_mod;


  // not used, but cool
  template<typename T> struct SymbolSearcher
  {
    SymbolSearcher(T searchFor, std::string& result) :
      _sought(searchFor), _found(result)
    {
    }

    void operator() (std::basic_string<char> s, T ct)
    {
      if (_sought == ct)
      {
        _found = s;
      }
    }

    std::string found() const { return _found; }

  private:
    T             _sought;
    std::string&  _found;
  };
  

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Grammar Template
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  template <typename Iterator> struct an_grammar :
      qi::grammar<Iterator, ChessANDecomp()>
  {
    an_grammar() : an_grammar::base_type(an)
    {
        using namespace qi::labels;
        namespace phoenix = boost::phoenix;

        using qi::eps;
        using qi::on_error;
        using qi::fail;
        //using qi::lit;
        //using qi::lexeme;
        //using qi::int_;
        //using qi::double_;
        //using ascii::char_;

        using phoenix::at_c;
        using phoenix::ref;
        using phoenix::construct;
        using phoenix::val;

        // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
        // Grammar rules.
        // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
        
        //
        // Algebraic Notation
        //
        an =
          eps >>
          (
              can [_val = _1, AT_ALGEBRA(_val) = CAN]
            | san [_val = _1, AT_ALGEBRA(_val) = SAN]
          )
        ;

        RULE_NAME(an);

        //
        // Coordinate Algebraic Notation
        //
        can =
            can_promotion  [_val = _1]
          |
                pos [AT_SRC_POS(_val) = _1]
            >>  pos [AT_DST_POS(_val) = _1]
        ;

        RULE_NAME(can);

        //
        // CAN pawn promotion. Examples: "Q" "(Q) "=N".
        //
        can_promotion = 
                promotion_src_pos [AT_SRC_POS(_val) = _1]
            >>  promotion_dst_pos [AT_DST_POS(_val) = _1]
            >>  promotion         [AT_PROMOTED(_val) = _1,
                                   AT_MOVED_PIECE(_val) = Pawn]
        ;

        RULE_NAME(can_promotion);

        //
        // Standard Algebraic Notation
        //
        san =
          (
              san_castling_move   [_val = _1] 
            | san_en_passant_move [_val = _1] 
            | san_promotion_move  [_val = _1]
            | san_capture_move    [_val = _1] 
            | san_basic_move      [_val = _1]
          )
          >   -(check_or_mate) [AT_CHECK(_val) = _1]
        ;

        RULE_NAME(san);

        //
        // SAN castling move. Examples: "0-0" "0-0-0" "O-O".
        //
        san_castling_move =
          castling [AT_CASTLING(_val) = _1, AT_MOVED_PIECE(_val) = King]
        ;

        RULE_NAME(san_castling_move);

        //
        // SAN en passant move. Examples: "dxc6e.p" "hxg3e.p".
        //
        san_en_passant_move =
              disambig            [AT_SRC_POS(_val) = _1]
          >>  capture_mod         [AT_HAS_CAPTURED(_val) = true]
          >>  en_passant_dst_pos  [AT_DST_POS(_val) = _1]
          >>  en_passant_mod      [AT_EN_PASSANT(_val) = true,
                                   AT_MOVED_PIECE(_val) = Pawn,
                                   AT_CAPTURED_PIECE(_val) = Pawn]
        ;

        RULE_NAME(san_en_passant_move);

        //
        // SAN pawn promotion move.
        // Examples: "h1Q" "b8=Q" "b1/Q" "h8N" "gxf8(N)".
        //
        san_promotion_move =
                promotion_dst_pos   [AT_DST_POS(_val) = _1]
            >>  promotion           [AT_PROMOTED(_val) = _1, 
                                     AT_MOVED_PIECE(_val) = Pawn]
          |
                disambig            [AT_SRC_POS(_val) = _1]
            >>  promotion_dst_pos   [AT_DST_POS(_val) = _1]
            >>  promotion           [AT_PROMOTED(_val) = _1, 
                                     AT_MOVED_PIECE(_val) = Pawn]
          |
                -(disambig)         [AT_SRC_POS(_val) = _1]
            >>  capture_mod         [AT_HAS_CAPTURED(_val) = true,
                                     AT_CAPTURED_PIECE(_val) = UndefPiece]
            >>  promotion_dst_pos   [AT_DST_POS(_val) = _1]
            >>  promotion           [AT_PROMOTED(_val) = _1, 
                                     AT_MOVED_PIECE(_val) = Pawn]
        ;

        RULE_NAME(san_promotion_move);

        //
        // SAN capture move.
        //
        san_capture_move = 
            san_pawn_capturing   [_val = _1, AT_MOVED_PIECE(_val) = Pawn]
          | san_major_capturing  [_val = _1]
        ;

        RULE_NAME(san_capture_move);

        //
        // SAN pawn capturing move. Examples: "exd7" "axb4".
        //
        san_pawn_capturing =
              -(disambig)   [AT_SRC_POS(_val) = _1]
          >>  capture_mod   [AT_HAS_CAPTURED(_val) = true,
                             AT_CAPTURED_PIECE(_val) = UndefPiece]
          >   pos           [AT_DST_POS(_val) = _1,
                             AT_MOVED_PIECE(_val) = Pawn]
        ;

        RULE_NAME(san_pawn_capturing);

        //
        // SAN major piece capturing move. Examples: "Bxe5" "Ngxf3".
        //
        san_major_capturing =
              major_piece [AT_MOVED_PIECE(_val) = _1]
          >>  -(disambig) [AT_SRC_POS(_val) = _1]
          >>  capture_mod [AT_HAS_CAPTURED(_val) = true,
                           AT_CAPTURED_PIECE(_val) = UndefPiece]
          >   pos         [AT_DST_POS(_val) = _1]
        ;

        RULE_NAME(san_major_capturing);

        //
        // SAN basic move (i.e. no capture, not special move).
        //
        san_basic_move = 
            san_pawn_move   [_val = _1, AT_MOVED_PIECE(_val) = Pawn]
          | san_major_move  [_val = _1]
        ;

        RULE_NAME(san_basic_move);

        //
        // SAN pawn basic move. Examples: "c3" "e5" "de3".
        //
        san_pawn_move =
            pos               [AT_DST_POS(_val) = _1]
          |
                file_or_rank  [AT_SRC_POS(_val) = _1]
            >>  pos           [AT_DST_POS(_val) = _1,
                               AT_MOVED_PIECE(_val) = Pawn]
        ;
        
        RULE_NAME(san_pawn_move);

        //
        // SAN major piece basic move. Examples: "Nc3" "Kd2" "Rdd5" "N5f3".
        //
        san_major_move =
                major_piece [AT_MOVED_PIECE(_val) = _1]
            >>  disambig    [AT_SRC_POS(_val) = _1]
            >>  pos         [AT_DST_POS(_val) = _1]
          |
                major_piece [AT_MOVED_PIECE(_val) = _1]
            >>  pos         [AT_SRC_POS(_val) = NoPos,
                             AT_DST_POS(_val) = _1]
        ;

        RULE_NAME(san_major_move);

        //
        // Pawn promotion. Examples "=R" "/N" "(Q)" "Q".
        //
        promotion =
            promotion_mod > promotion_piece [_val = _1]
          | '(' > promotion_piece           [_val = _1] > ')'
          | promotion_piece                 [_val = _1]
        ;

        RULE_NAME(promotion);

        //
        // File only position.
        //
        file_only_pos = file  [AT_POS_FILE(_val) = _1];

        RULE_NAME(file_only_pos);

        //
        // Rank only position.
        //
        rank_only_pos = rank  [AT_POS_RANK(_val) = _1];

        RULE_NAME(rank_only_pos);

        //
        // File or rank position, but not both.
        //
        file_or_rank =
            file_only_pos [_val = _1]
          | rank_only_pos [_val = _1] 
        ;

        RULE_NAME(file_or_rank);

        //
        // Position.
        //
        pos = file >> rank;

        RULE_NAME(pos);

        //
        // SAN piece source file, rank, or position disambiguation.
        //
        disambig =
            pos           [_val = _1]
          | file_or_rank  [_val = _1]
        ;

        RULE_NAME(disambig);

        //
        // SAN pawn source file or position disambiguation.
        //
        disambig_pawn =
            pos           [_val = _1]
          | file_only_pos [_val = _1]
        ;

        RULE_NAME(disambig_pawn);

        //
        // En passant source position.
        //
        en_passant_src_pos = file >> en_passant_src_rank;

        RULE_NAME(en_passant_src_pos);

        //
        // En passant destination position.
        //
        en_passant_dst_pos = file >> en_passant_dst_rank;

        RULE_NAME(en_passant_dst_pos);

        //
        // Pawn promotion source position.
        //
        promotion_src_pos = file >> penult_rank;

        RULE_NAME(promotion_src_pos);

        //
        // Pawn promotion destination position.
        //
        promotion_dst_pos = file >> ult_rank;

        RULE_NAME(promotion_dst_pos);

        //
        // Check(mate) modifier.
        //
        check_or_mate = check_mod [_val = _1];

        RULE_NAME(check_or_mate);

        //
        // On parse error handler.
        //
        on_error<fail>
        (
            an
          , m_ssError
              << val("Expecting ")
              << _4                             // what failed?
              << val(" token, but got \"")
              << construct<std::string>(_3, _2) // iterators to error-pos, end
              << val("\"")
        );
    }

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Rule signatures.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    // start
    qi::rule<Iterator, ChessANDecomp()> an;

    // CAN
    qi::rule<Iterator, ChessANDecomp()> can;
    qi::rule<Iterator, ChessANDecomp()> can_promotion;

    // SAN
    qi::rule<Iterator, ChessANDecomp()> san;
    qi::rule<Iterator, ChessANDecomp()> san_castling_move;
    qi::rule<Iterator, ChessANDecomp()> san_en_passant_move;
    qi::rule<Iterator, ChessANDecomp()> san_promotion_move;
    qi::rule<Iterator, ChessANDecomp()> san_capture_move;
    qi::rule<Iterator, ChessANDecomp()> san_pawn_capturing;
    qi::rule<Iterator, ChessANDecomp()> san_major_capturing;
    qi::rule<Iterator, ChessANDecomp()> san_basic_move;
    qi::rule<Iterator, ChessANDecomp()> san_pawn_move;
    qi::rule<Iterator, ChessANDecomp()> san_major_move;


    // piece
    qi::rule<Iterator, ChessPiece()>  promotion;

    // position
    qi::rule<Iterator, ChessPos()> file_only_pos;
    qi::rule<Iterator, ChessPos()> rank_only_pos;
    qi::rule<Iterator, ChessPos()> file_or_rank;
    qi::rule<Iterator, ChessPos()> pos;
    qi::rule<Iterator, ChessPos()> disambig;
    qi::rule<Iterator, ChessPos()> disambig_pawn;
    qi::rule<Iterator, ChessPos()> en_passant_src_pos;
    qi::rule<Iterator, ChessPos()> en_passant_dst_pos;
    qi::rule<Iterator, ChessPos()> promotion_src_pos;
    qi::rule<Iterator, ChessPos()> promotion_dst_pos;

    // modifier
    qi::rule<Iterator, ChessCheckMod()> check_or_mate;

    std::stringstream m_ssError;
  };

} // namespace chess_engine


// --------------------------------------------------------------------------
// Chess Algebraic Notation in Regular Expressions
// --------------------------------------------------------------------------

/*!
 * \brief Regular expression helper macros.
 */
#define SOL "^"                           ///< start of line
#define EOL "$"                           ///< end of line
#define G(expr) string("(" + expr + ")")  ///< match group expression

namespace chess_engine
{
  /*!
   * \brief Helper (marked) sub-expression strings used in regular expressions.
   * \{
   */
  // file and rank position expressions
  static const string exprFile("[a-h]");
  static const string exprRank("[1-8]");
  static const string exprRankPenult("[27]");
  static const string exprRankUlt("[18]");
  
  // piece expressions
  static const string exprPiece("[KQRBNP]");
  static const string exprPieceMajor("[KQRBN]");
  static const string exprPiecePromo("[QRBN]");
  
  // castlin expressions
  static const string exprCastlingKingSide("0-0|O-O");
  static const string exprCastlingQueenSide("0-0-0|O-O-O");
  
  // modifier expressions
  static const string exprModCapture("x|:");
  static const string exprModEnPassant("e\\.p");
  static const string exprModPromotion("[=/]");
  static const string exprModCheck("\\+{1,2}|#");
  
  // groups
  static const string grpDisambig(G(exprFile + "?" + exprRank + "?"));
  static const string grpPos(G(exprFile+exprRank));
  static const string grpPosPenult(G(exprFile + exprRankPenult));
  static const string grpPosUlt(G(exprFile + exprRankUlt));
  static const string grpModCheckOpt(G(exprModCheck) + "?");
  /*! \} */
  
  /*!
   * \brief Coordinate and Standard Algebraic Notation regular expressions.
   * \{
   */
  
  // 2 groups. "a2a4" "g6a6" ...
  static boost::regex reCAN(SOL + grpPos + grpPos + EOL);
  
  // 3 groups. "c7c8=Q" "b2a1/N" "d2d1Q" ...
  static boost::regex reCANPromotion1(
                        SOL
                          + grpPosPenult + grpPosUlt
                          + exprModPromotion
                          + G(exprPiecePromo)
                        + EOL);
  
  // 3 groups. "c7c8(Q)" ...
  static boost::regex reCANPromotion2(
                        SOL
                          + grpPosPenult + grpPosUlt
                          + "\\("+G(exprPiecePromo)+"\\)"
                        + EOL);
  
  // 1 group. 0-0 ...
  static boost::regex reCastleKingSide(
                        SOL
                          + G(exprCastlingKingSide)
                          + grpModCheckOpt
                          + G(exprModCheck) + "?"
                        + EOL);
  
  // 1 group. 0-0-0 ...
  static boost::regex reCastleQueenSide(
                        SOL
                          + G(exprCastlingQueenSide)
                          + grpModCheckOpt
                        + EOL);
  
  // 3 groups. "bxc6e.p" "gxf3e.p" "b5xc6e.p" ...
  static boost::regex reEnPassant(
                        SOL
                          + grpDisambig
                          + G(exprModCapture)
                          + grpPos
                          + exprModEnPassant
                          + grpModCheckOpt
                        + EOL);
  
  // 3 groups. "c8=Q" "bxa1/Q" "d1Q" "axb8=N" "f8Q" ...
  static boost::regex rePawnPromotion1(
                        SOL
                          + grpDisambig + ".?" + grpPos
                          + exprModPromotion + "?"
                          + G(exprPiecePromo)
                          + grpModCheckOpt
                        + EOL);
  
  // 3 groups. "c7c8(Q)" "dxc8(R)" ...
  static boost::regex rePawnPromotion2(
                        SOL
                          + grpDisambig + ".?" + grpPos
                          + "\\("+G(exprPiecePromo)+"\\)"
                          + grpModCheckOpt
                        + EOL);
  
  // 2 groups. "a4" "exf5" "a2a4" ...
  static boost::regex rePawnMove(
                        SOL
                          + grpDisambig
                          + ".?"
                          + grpPos
                          + grpModCheckOpt
                        + EOL);
  
  // 3 groups. "Ba4" "Qxf5" "Ra2a4" ...
  static boost::regex reMajorMove(
                        SOL
                          + G(exprPieceMajor)
                          + grpDisambig + ".?" + grpPos
                          + grpModCheckOpt
                        + EOL);
  
  // 1 group. "x" ...
  static boost::regex reModCapture(".+" + G(exprModCapture) + ".+");
  
  // 1 group. "+" ...
  static boost::regex reModCheck("[^\\+]+" + G(exprModCheck) + EOL);
  /*! \} */
  
  /*!
   * \brief RegEx helper to parse position.
   *
   * \param strExpr   Move (sub)position.
   * \param [out] pos Position.
   */
  static void _position(const string &strExpr, ChessPos &pos)
  {
    pos.m_file = (ChessFile)strExpr[0];
    pos.m_rank = (ChessRank)strExpr[1];
  }
  
  /*!
   * \brief RegEx helper to parse disambiguous position.
   *
   * \param strExpr   Move (sub)position.
   * \param [out] pos Position.
   */
  static void _disambiguate(const string &strExpr, ChessPos &pos)
  {
    if( strExpr.size() == 2 )
    {
      _position(strExpr, pos);
    }
    else if( (strExpr[0] >= (char)ChessFileA) && 
             (strExpr[0] <= (char)ChessFileH) )
    {
      pos.m_file = (ChessFile)strExpr[0];
    }
    else if( (strExpr[0] >= (char)ChessRank1) && 
             (strExpr[0] <= (char)ChessRank8) )
    {
      pos.m_rank = (ChessRank)strExpr[0];
    }
  }

} // namespace chess_engine


//----------------------------------------------------------------------------
// Class impl
//----------------------------------------------------------------------------

/*!
 * Private implementation class.
 */
class impl
{
public:
  typedef std::string::const_iterator iterator_type;
  typedef chess_engine::an_grammar<iterator_type> an_grammar;
  typedef chess_engine::ChessANDecomp an_decomp;

  an_grammar  m_grammar;    ///< the grammar

  /*!
   * \brief Default constructor.
   */
  impl() { }

  /*!
   * \brief Destructor.
   */
  virtual ~impl() { };
};

using namespace chess_engine;


// --------------------------------------------------------------------------
// Struct ChessANDecomp
// --------------------------------------------------------------------------

/*!
 * \brief Default constructor.
 */
ChessANDecomp::ChessANDecomp()
{
  clear();
};

/*!
 * \brief Copy constructor.
 *
 * \param src   Source object.
 */
ChessANDecomp::ChessANDecomp(const ChessANDecomp &src)
{
  m_strAN           = src.m_strAN;
  m_ePlayer         = src.m_ePlayer;
  m_eAlgebra        = src.m_eAlgebra;
  m_ePieceMoved     = src.m_ePieceMoved;
  m_eColorMoved     = src.m_eColorMoved;
  m_posSrc          = src.m_posSrc;
  m_posDst          = src.m_posDst;
  m_bHasCaptured    = src.m_bHasCaptured;
  m_ePieceCaptured  = src.m_ePieceCaptured;
  m_ePiecePromoted  = src.m_ePiecePromoted;
  m_bIsEnPassant    = src.m_bIsEnPassant;
  m_eCastling       = src.m_eCastling;
  m_eCheck          = src.m_eCheck;
}

/*!
 * \brief Clear data.
 */
void ChessANDecomp::clear()
{
  m_strAN.clear();
  m_ePlayer         = NoColor;
  m_eAlgebra        = UnknownAN;
  m_ePieceMoved     = UndefPiece;
  m_eColorMoved     = UndefColor;
  m_posSrc.clear();
  m_posDst.clear();
  m_bHasCaptured    = false;
  m_ePieceCaptured  = NoPiece;
  m_ePiecePromoted  = NoPiece;
  m_bIsEnPassant    = false;
  m_eCastling       = NoCastling;
  m_eCheck          = NoCheckMod;
}

/*!
 * \brief The ChessANDecomp output stream operator.
 */
std::ostream &chess_engine::operator<<(std::ostream        &os,
                                       const ChessANDecomp &obj)
{
  os
    << "{" << endl
      << "  AN             = " << obj.m_strAN << endl
      << "  player         = "
        << nameOfColor(obj.m_ePlayer)
        << "(" << (char)obj.m_ePlayer << ")"
        << endl
      << "  algebra        = "
        << nameOfAlgebra(obj.m_eAlgebra)
        << "(" << obj.m_eAlgebra << ")"
        << endl
      << "  piece moved    = "
        << nameOfPiece(obj.m_ePieceMoved)
        << "(" << (char)obj.m_ePieceMoved << ")"
        << endl
      << "  color moved    = "
        << nameOfColor(obj.m_eColorMoved)
        << "(" << (char)obj.m_eColorMoved << ")"
        << endl
      << "  srcdst         = " << obj.m_posSrc << obj.m_posDst << endl
      << "  capture        = "
        << nameOfBool(obj.m_bHasCaptured)
        << "(" << obj.m_bHasCaptured << ")" << endl
      << "  piece captured = "
        << nameOfPiece(obj.m_ePieceCaptured)
        << "(" << (char)obj.m_ePieceCaptured << ")"
        << endl
      << "  piece promoted = "
        << nameOfPiece(obj.m_ePiecePromoted)
        << "(" << (char)obj.m_ePiecePromoted << ")"
        << endl
      << "  en_passant     = "
        << nameOfBool(obj.m_bIsEnPassant) << "(" << obj.m_bIsEnPassant << ")"
        << endl
      << "  castling       = "
        << nameOfCastling(obj.m_eCastling)
        << "(" << (char)obj.m_eCastling << ")"
        << endl
      << "  check          = "
        << nameOfCheckMod(obj.m_eCheck)
        << "(" << (char)obj.m_eCheck << ")"
        << endl
    << "}" << endl;
  ;

  return os;
}


//----------------------------------------------------------------------------
// Class ChessANParser
//----------------------------------------------------------------------------

/*!
 * \brief ChessANParser helper strings, functions, and macros.
 * \{
 */

const static string ErrorPreface("Parse error: ");
const static string TracePreface("Trace: ");

//
// Implementation access macros.
//
#define IMPL(p)         ((impl *)p)
#define IMPL_GRAMMAR(p) IMPL(p)->m_grammar

#undef AN_DBG      ///< define or undef (not tracing)

// Parse trace macro. (Or is it an ant race?).
#define AN_TRACE(_trace) \
do \
{ \
  if( m_bTrace ) cout << TracePreface << _trace << endl; \
} \
while(0)

// Parse error macro.
#define AN_ERROR(_err) \
do \
{ \
  stringstream ss; \
  ss << ErrorPreface << _err; \
  m_strError = ss.str(); \
} \
while(0)

const string nameOfParser(ChessANParser::Method eMethod)
{
  switch( eMethod )
  {
    case ChessANParser::MethodBNF:
      return "BNF Parser";
    case ChessANParser::MethodRegEx:
      return "RegEx Parser";
    default:
      return "Unknown Parser";
  }
}

/*! \} */


ChessANParser::ChessANParser()
{
  m_impl    = new impl;
  m_bTrace  = false;
  m_eMethod = MethodBNF;
}

ChessANParser::~ChessANParser()
{
  delete IMPL(m_impl);
}

int ChessANParser::parse(const string     &strAN,
                         const ChessColor ePlayer,
                         ChessANDecomp    &decomp)
{
  int   rc;   // return code

  AN_TRACE("Input: "
      << "AN = '" << strAN << "' "
      << "player = " << nameOfColor(ePlayer));
  AN_TRACE("Using: " << nameOfParser(m_eMethod));

  // clear decomposition
  decomp.clear();

  // clear errors
  clearErrors();

  // call parser
  switch( m_eMethod )
  {
    case MethodBNF:
      rc = runParserBNF(strAN, decomp);
      break;
    case MethodRegEx:
      rc = runParserRegEx(strAN, decomp);
      break;
    default:
      AN_ERROR("Bug: Unknown parser");
      rc = -CE_ECODE_GEN;
  }

  // do this after the parser which may clear data prior to parse
  decomp.m_strAN    = strAN;
  decomp.m_ePlayer  = ePlayer;

  AN_TRACE("Parsed: " << decomp);

  //
  // The parse succeeded. Now perform post-processing.
  //
  if( rc == CE_OK )
  {
    rc = postprocess(decomp);
  }

  if( rc == CE_OK )
  {
    AN_TRACE("Post-Processed: " << decomp);
    AN_TRACE("Ok");
  }
  else
  {
    AN_TRACE(m_strError);
  }

  return rc;
}

int ChessANParser::parse(const string     &strAN,
                         const ChessColor ePlayer,
                         ChessMove        &move)
{
  int   rc;

  ChessANDecomp decomp;

  if( (rc = parse(strAN, ePlayer, decomp)) == CE_OK )
  {
    toChessMove(decomp, move);
  }

  return rc;
}

int ChessANParser::runParserBNF(const string &strAN, ChessANDecomp &decomp)
{
  string::const_iterator iter = strAN.begin();
  string::const_iterator end  = strAN.end();
  
  // bnf parser
  bool bOk = qi::parse(iter, end, IMPL_GRAMMAR(m_impl), decomp);

  // unparsed trailing tokens is considered a failure
  if( iter != end )
  {
    bOk = false;
  }

  //
  // The parse failed. 
  //
  if( !bOk )
  {
    // error message from bnf parse handler
    string strError(IMPL_GRAMMAR(m_impl).m_ssError.str());

    // no error caught from parse handler, so create error message here
    if( strError.empty() )
    {
      string sstr(iter, end);

      strError = "Bad syntax at '" + sstr + "'";
    }

    AN_ERROR(strError);

    return -CE_ECODE_CHESS_PARSE;
  }

  return CE_OK;
}

int ChessANParser::runParserRegEx(const string &strAN, ChessANDecomp &decomp)
{
  int   rc;

  // try CAN first
  if( (rc = parseCANRegEx(strAN, decomp)) != CE_OK )
  {
    // otherwise try SAN
    rc = parseSANRegEx(strAN, decomp);
  }

  return rc;
}

int ChessANParser::parseCANRegEx(const string &strCAN, ChessANDecomp &decomp)
{
  boost::cmatch what;
  ChessAlgebra  an = CAN;
  string preface = nameOfParser(m_eMethod) + ": " + nameOfAlgebra(an) + ": ";

  // pawn promotion
  if( boost::regex_match(strCAN.c_str(), what, reCANPromotion1) ||
      boost::regex_match(strCAN.c_str(), what, reCANPromotion2) )
  {
    AN_TRACE(preface << NameOfMovePawnPromotion << " move matched.");
    decomp.m_ePieceMoved    = Pawn;
    decomp.m_ePiecePromoted = (ChessPiece)what[3].str()[0];
  }

  // basic coordinate move
  else if( boost::regex_match(strCAN.c_str(), what, reCAN) )
  {
    AN_TRACE(preface << NameOfMoveCoord << " move matched.");
  }

  // unknown or bad syntax
  else
  {
    AN_ERROR(preface << "No matches - unknown AN pattern.");
    return -CE_ECODE_CHESS_PARSE;
  }

  //
  // Good
  //
  decomp.m_eAlgebra = an;
  _position(what[1].str(), decomp.m_posSrc);
  _position(what[2].str(), decomp.m_posDst);

  return CE_OK;
}

int ChessANParser::parseSANRegEx(const string &strSAN, ChessANDecomp &decomp)
{
  boost::cmatch what;
  ChessAlgebra an = SAN;
  string  preface = nameOfParser(m_eMethod) + ": " + nameOfAlgebra(an) + ": ";

  // king side castle
  if( boost::regex_match(strSAN.c_str(), what, reCastleKingSide) )
  {
    AN_TRACE(preface << "Kingside " << NameOfMoveCastling << " move matched.");
    decomp.m_ePieceMoved = King;
    decomp.m_eCastling   = KingSide;
  }

  // queen side castle
  else if( boost::regex_match(strSAN.c_str(), what, reCastleQueenSide) )
  {
    AN_TRACE(preface << "Queenside " << NameOfMoveCastling << " move matched.");
    decomp.m_ePieceMoved = King;
    decomp.m_eCastling   = QueenSide;
  }

  // en passant
  else if( boost::regex_match(strSAN.c_str(), what, reEnPassant) )
  {
    AN_TRACE(preface << NameOfMoveEnPassant << " move matched.");
    decomp.m_ePieceMoved = Pawn;
    _disambiguate(what[1].str(), decomp.m_posSrc);
    _position(what[3].str(),     decomp.m_posDst);
    decomp.m_bHasCaptured = true;
    decomp.m_bIsEnPassant = true;
  }

  // pawn promotion
  else if( boost::regex_match(strSAN.c_str(), what, rePawnPromotion1) ||
           boost::regex_match(strSAN.c_str(), what, rePawnPromotion2) )
  {
    AN_TRACE(preface << NameOfMovePawnPromotion << " move matched.");
    decomp.m_ePieceMoved = Pawn;
    _disambiguate(what[1].str(), decomp.m_posSrc);
    _position(what[2].str(), decomp.m_posDst);
    decomp.m_ePiecePromoted = (ChessPiece)what[3].str()[0];
  }

  // pawn move
  else if( boost::regex_match(strSAN.c_str(), what, rePawnMove) )
  {
    AN_TRACE(preface << nameOfPiece(Pawn) << " move matched.");
    decomp.m_ePieceMoved = Pawn;
    _disambiguate(what[1].str(), decomp.m_posSrc);
    _position(what[2].str(), decomp.m_posDst);
  }

  // major piece move
  else if( boost::regex_match(strSAN.c_str(), what, reMajorMove) )
  {
    AN_TRACE(preface << NameOfMoveMajor << " move matched.");
    decomp.m_ePieceMoved = (ChessPiece)what[1].str()[0];
    _disambiguate(what[2].str(), decomp.m_posSrc);
    _position(what[3].str(), decomp.m_posDst);
  }

  else
  {
    AN_ERROR(preface << "No matches - unknown AN pattern.");
    return -CE_ECODE_CHESS_PARSE;
  }

  // capture
  if( boost::regex_match(strSAN.c_str(), what, reModCapture) )
  {
    AN_TRACE(preface << NameOfMoveCapture << " move matched.");
    decomp.m_bHasCaptured   = true;
    decomp.m_ePieceCaptured = decomp.m_bIsEnPassant? Pawn: UndefPiece;
  }

  // check or checkmate
  if( boost::regex_match(strSAN.c_str(), what, reModCheck) )
  {
    if( what[1].str() == "+" )
    {
      decomp.m_eCheck = ModCheck;
    }
    else if( what[1].str() == "++" )
    {
      decomp.m_eCheck = ModDoubleCheck;
    }
    else if( what[1].str() == "#" )
    {
      decomp.m_eCheck = ModCheckmate;
    }

    AN_TRACE(preface << nameOfCheckMod(decomp.m_eCheck)
        << " modifier matched.");
  }

  //
  // Good
  //
  decomp.m_eAlgebra = an;

  return CE_OK;
}

int ChessANParser::postprocess(ChessANDecomp &decomp)
{
  int rc = CE_OK;

  //
  // Post-process the parsed castling move.
  //
  if( decomp.m_eCastling != NoCastling )
  {
    rc = postprocCastling(decomp);
  }

  //
  // Post-process the parsed pawn en passant move.
  //
  else if( decomp.m_bIsEnPassant )
  {
    rc = postprocEnPassant(decomp);
  }

  //
  // Post-process the parsed pawn promotion move.
  //
  else if( decomp.m_ePiecePromoted != NoPiece )
  {
    rc = postprocPromotion(decomp);
  }

  //
  // Post-process the parsed pawn move.
  //
  else if( decomp.m_ePieceMoved == Pawn )
  {
    rc = postprocPawn(decomp);
  }

  //
  // Post-process the parsed major piece move.
  //
  else
  {
    rc = postprocMajorPiece(decomp);
  }

  return rc;
}

int ChessANParser::postprocCastling(ChessANDecomp &decomp)
{
  //
  // Determine source,destination file
  //
  switch( decomp.m_eCastling )
  {
    case KingSide:
      decomp.m_posSrc.m_file = ChessFileE;
      decomp.m_posDst.m_file = ChessFileG;
      break;
    case QueenSide:
      decomp.m_posSrc.m_file = ChessFileE;
      decomp.m_posDst.m_file = ChessFileC;
      break;
    default:
      AN_ERROR("Bug: Not a " << NameOfMoveCastling << " move.");
      return -CE_ECODE_CHESS_PARSE;
  }

  //
  // Determine color and source,destination rank (if possible)
  //
  switch( decomp.m_ePlayer )
  {
    case White:
      decomp.m_eColorMoved   = White;
      decomp.m_posSrc.m_rank = ChessRank1;
      decomp.m_posDst.m_rank = ChessRank1;
      break;
    case Black:
      decomp.m_eColorMoved   = Black;
      decomp.m_posSrc.m_rank = ChessRank8;
      decomp.m_posDst.m_rank = ChessRank8;
      break;
    case NoColor:
    case UndefColor:
    default:
      break;
  }

  return CE_OK;
}

int ChessANParser::postprocEnPassant(ChessANDecomp &decomp)
{
  ChessPos    src(decomp.m_posSrc); // source position
  ChessPos    dst(decomp.m_posDst); // destination position
  ChessColor  color;                // required piece color
  ChessRank   rank;                 // required source rank
  ChessFile   fileL, fileR;         // required source file is left or right

  //
  // From destination rank, determine piece color and source rank.
  //
  switch( dst.m_rank )
  {
    case ChessRank6:        // white
      color = White;
      rank  = ChessRank5;
      break;
    case ChessRank3:        // black
      color = Black;
      rank  = ChessRank4;
      break;
    default:                // error
      AN_ERROR(NameOfMoveEnPassant
        << " move "
        << decomp.m_posSrc << decomp.m_posDst
        << " specifies bad destination rank '"
        << (char)decomp.m_posDst.m_rank
        << "'.");
      return -CE_ECODE_CHESS_PARSE;
  }

  AN_TRACE(NameOfMoveEnPassant << "is for " << nameOfColor(color) << ".");

  // verify against player's color
  if( (decomp.m_ePlayer != NoColor) && (decomp.m_ePlayer != color) )
  {
    AN_ERROR(NameOfMoveEnPassant
      << " move "
      << decomp.m_posSrc << decomp.m_posDst
      << " is applicable for "
      << nameOfColor(color)
      << ", but the player is "
      << nameOfColor(decomp.m_ePlayer)
      << ".");
    return -CE_ECODE_CHESS_PARSE;
  }

  // if not specified set source rank
  if( src.m_rank == NoRank )
  {
    src.m_rank = rank;
  }

  // otherwise verify source rank
  else if( src.m_rank != rank )
  {
    AN_ERROR(NameOfMoveEnPassant
      << " move "
      << decomp.m_posSrc << decomp.m_posDst
      << " specifies bad source rank '"
      << (char)src.m_rank
      << "'.");
    return -CE_ECODE_CHESS_PARSE;
  }


  //
  // If not specified try to determine source file.
  //
  if( src.m_file == NoFile )
  {
    switch( dst.m_file )
    {
      case ChessFileA:  // capturing move must be from the right
       src.m_file = ChessFileB;
       break;
     case ChessFileH:   // capturing move must be from the left
       src.m_file = ChessFileG;
       break;
     default:           // not determinable
       break;
    }
  }

  //
  // Otherwise verify source file.
  //
  else
  {
    // source file is either to the left or right of destination file
    fileL = ChessBoard::shiftFile(dst.m_file, -1);
    fileR = ChessBoard::shiftFile(dst.m_file,  1);

    // verify en passant diagonal move
    if( (src.m_file != fileL) && (src.m_file != fileR)  )
    {
      AN_ERROR(NameOfMoveEnPassant
        << " move "
        << decomp.m_posSrc << decomp.m_posDst
        << " specifies bad source file '"
        << (char)src.m_file
        << "'.");
      return -CE_ECODE_CHESS_PARSE;
    }
  }
  
  //
  // Ok
  //
  decomp.m_eColorMoved  = color;
  decomp.m_posSrc       = src;
  decomp.m_posDst       = dst;

  return CE_OK;
}

int ChessANParser::postprocPromotion(ChessANDecomp &decomp)
{
  ChessPos    src(decomp.m_posSrc); // source position
  ChessPos    dst(decomp.m_posDst); // destination position
  ChessColor  color;                // required piece color
  ChessRank   rank;                 // required source rank
  ChessFile   fileL, fileR;         // source file to the left/right of destin.
  bool        cap = decomp.m_bHasCaptured;        // has [not] captured 
  ChessPiece  capPiece = decomp.m_ePieceCaptured; // captured piece, if any

  //
  // From destination rank, determine piece color and source rank.
  //
  switch( dst.m_rank )
  {
    case ChessRank8:      // white
      color = White;
      rank  = ChessRank7;
      break;
    case ChessRank1:      // black
      color = Black;
      rank  = ChessRank2;
      break;
    default:
      AN_ERROR(NameOfMovePawnPromotion
        << " move "
        << decomp.m_posSrc << decomp.m_posDst
        << " specifies bad destination rank '"
        << (char)dst.m_rank
        << "'.");
      return -CE_ECODE_CHESS_PARSE;
  }

  AN_TRACE(NameOfMovePawnPromotion << "is for " << nameOfColor(color) << ".");

  // verify against player's color
  if( (decomp.m_ePlayer != NoColor) && (decomp.m_ePlayer != color) )
  {
    AN_ERROR(NameOfMovePawnPromotion
      << " move "
      << decomp.m_posSrc << decomp.m_posDst
      << " is applicable for "
      << nameOfColor(color)
      << ", but the player is "
      << nameOfColor(decomp.m_ePlayer)
      << ".");
    return -CE_ECODE_CHESS_PARSE;
  }

  // if not specified set source rank
  if( src.m_rank == NoRank )
  {
    src.m_rank = rank;
  }

  // otherwise verify source rank
  else if( src.m_rank != rank )
  {
    AN_ERROR(NameOfMovePawnPromotion
      << " move "
      << decomp.m_posSrc << decomp.m_posDst
      << " specifies bad source rank '"
      << (char)src.m_rank
      << "'.");
    return -CE_ECODE_CHESS_PARSE;
  }

  //
  // If not specified, try to determine source file.
  //
  if( src.m_file == NoFile )
  {
    if( cap )
    {
      switch( dst.m_file )
      {
        case ChessFileA:  // capturing move must be from the right
          src.m_file = ChessFileB;
          break;
        case ChessFileH:  // capturing move must be from the left
          src.m_file = ChessFileG;
          break;
        default:          // not determinable
          break;
      }
    }
    else
    {
      src.m_file = dst.m_file;
    }
  }

  //
  // Otherwise verify source file.
  //
  else
  {
    // capture source file is either to the left or right of destination file
    fileL = ChessBoard::shiftFile(dst.m_file, -1);
    fileR = ChessBoard::shiftFile(dst.m_file,  1);

    // CAN does not have a capture notation. However, a capture can be inferred
    // if the pawn has moved diagonally.
    if( decomp.m_eAlgebra == CAN )
    {
      // diagonal move = capture move
      if( (src.m_file == fileL) || (src.m_file == fileR) )
      {
        cap       = true;
        capPiece  = UndefPiece;

        AN_TRACE(NameOfMovePawnPromotion
            << "in CAN specifies "
            << NameOfMoveCapture << " move.");
      }
      else
      {
        cap       = false;
        capPiece  = NoPiece;
      }
    }

    // verify capture move source file
    if( cap && (src.m_file != fileL) && (src.m_file != fileR) )
    {
      AN_ERROR(NameOfMovePawnPromotion
        << " with "
        << NameOfMoveCapture
        << " move "
        << decomp.m_posSrc << decomp.m_posDst
        << " specifies bad source file '"
        << (char)src.m_file
        << "'.");
      return -CE_ECODE_CHESS_PARSE;
    }

    // verify non-capture move source file
    else if( !cap && (src.m_file != dst.m_file) )
    {
      AN_ERROR(NameOfMovePawnPromotion 
        << " move "
        << decomp.m_posSrc << decomp.m_posDst
        << " specifies bad source file '"
        << (char)src.m_file
        << "'.");
      return -CE_ECODE_CHESS_PARSE;
    }
  }

  //
  // Ok
  //
  decomp.m_eColorMoved    = color;
  decomp.m_posSrc         = src;
  decomp.m_posDst         = dst;
  decomp.m_bHasCaptured   = cap;
  decomp.m_ePieceCaptured = capPiece;

  return CE_OK;
}

int ChessANParser::postprocPawn(ChessANDecomp &decomp)
{
  ChessPos      src(decomp.m_posSrc); // source position
  ChessPos      dst(decomp.m_posDst); // destination position
  ChessColor    color;                // required piece color
  ChessRank     rank;                 // required source rank
  int           rows, cols;           // number of rank/file spaces moved
  bool          cap = decomp.m_bHasCaptured;        // has [not] captured 
  ChessPiece    capPiece = decomp.m_ePieceCaptured; // captured piece, if any
  list_of_pos   positions;            // list of all source positions from dst
  list_of_pos   filtered;             // filtered on (partial) src 
  size_t        i;
  const string  pieceName(nameOfPiece(decomp.m_ePieceMoved));

  // assign piece color
  color = decomp.m_ePlayer != NoColor? decomp.m_ePlayer: decomp.m_eColorMoved;

  ChessBoard::findPawnSrcMoves(NoColor, dst, positions);

#ifdef AN_DBG
  cerr << "DBG: From '" << dst << "' there are " << positions.size()
    << " candidate source positions:" << endl;
  cerr << "  ";
  for(i = 0; i < positions.size(); ++i)
  {
    cerr << positions[i] << " ";
  }
  cerr << endl;
#endif

  // assign piece color
  color = decomp.m_ePlayer != NoColor? decomp.m_ePlayer: decomp.m_eColorMoved;

  //
  // Source rank unspecified. Might be determined if player's color is known.
  //
  if( src.m_rank == NoRank )
  {
    switch( color )
    {
      case White:
        // white pawn can move 1 or 2 spaces to rank 4, so not determinable
        if( dst.m_rank != ChessRank4 )
        {
          src.m_rank = ChessBoard::shiftRank(dst.m_rank, -1);
        }
        break;
      case Black:
        // black pawn can move 1 or 2 spaces to rank 5, so not determinable
        if( dst.m_rank != ChessRank5 )
        {
          src.m_rank = ChessBoard::shiftRank(dst.m_rank, 1);
        }
        break;
      default:
        break;
    }
  }

  //
  // Source rank specified. So pawn color can be determined and move 
  // partially verified.
  //
  else
  {
    rows = diffRanks(dst.m_rank, src.m_rank);

    switch( rows )
    {
      case 1:
      case 2:
        color = White;
        break;
      case -1:
      case -2:
        color = Black;
        break;
      default:
        AN_ERROR(pieceName
          << " move "
          << decomp.m_posSrc << decomp.m_posDst
          << " is illegal from source rank '"
          << (char)src.m_rank
          << "' to destination rank '"
          << (char)dst.m_rank
          << "'.");
        return -CE_ECODE_CHESS_PARSE;
    }

    AN_TRACE(pieceName << "is for " << nameOfColor(color) << ".");

    // verify against player's color
    if( (decomp.m_ePlayer != NoColor) && (decomp.m_ePlayer != color) )
    {
      AN_ERROR(pieceName
        << " move "
        << decomp.m_posSrc << decomp.m_posDst
        << " is applicable for "
        << nameOfColor(color)
        << ", but the player is "
        << nameOfColor(decomp.m_ePlayer)
        << ".");
      return -CE_ECODE_CHESS_PARSE;
    }
  }

  //
  // Source file unspecified. Can possibly be determined.
  //
  if( src.m_file == NoFile )
  {
    if( cap )
    {
      switch( dst.m_file )
      {
        case ChessFileA: // capturing move must be from the right
          src.m_file = ChessFileB;
          break;
        case ChessFileH: // capturing move must be from the left
          src.m_file = ChessFileG;
          break;
        default:    // not determinable
          break;
      }
    }
    else
    {
      src.m_file = dst.m_file;
    }
  }

  //
  // Source file specified. So unspecified capture can be determined and move 
  // partially verified.
  //
  else
  {
    cols = diffFiles(dst.m_file, src.m_file);

    switch( cols )
    {
      case 0:
        cap       = false;
        capPiece  = NoPiece;
        break;
      case 1:
      case -1:
        cap       = true;
        capPiece  = UndefPiece;
        break;
      default:
        AN_ERROR(pieceName
          << " move "
          << decomp.m_posSrc << decomp.m_posDst
          << " is illegal from source file '"
          << (char)src.m_file
          << "' to destination file '"
          << (char)dst.m_file
          << "'.");
        return -CE_ECODE_CHESS_PARSE;
    }

    if( !cap && decomp.m_bHasCaptured )
    {
      AN_ERROR(pieceName
          << " move "
          << decomp.m_posSrc << decomp.m_posDst
          << " is not a "
          << NameOfMoveCapture
          << " move.");
      return -CE_ECODE_CHESS_PARSE;
    }
  }

  ChessBoard::filterPositions(src, positions, filtered);

#ifdef AN_DBG
  cerr << "DBG: Filtered on '" << src << "' there are now " << filtered.size()
    << " candidate source positions:" << endl;
  cerr << "  ";
  for(i = 0; i < filtered.size(); ++i)
  {
    cerr << filtered[i] << " ";
  }
  cerr << endl;
#endif

  //
  // Sanity check
  //
  if( filtered.size() == 0 )
  {
    AN_ERROR(pieceName
        << " move "
        << decomp.m_posSrc << decomp.m_posDst
        << " is not legal.");
    return -CE_ECODE_CHESS_PARSE;
  }

  //
  // Ok
  //
  decomp.m_eColorMoved    = color;
  decomp.m_posSrc         = src;
  decomp.m_posDst         = dst;
  decomp.m_bHasCaptured   = cap;
  decomp.m_ePieceCaptured = capPiece;

  return CE_OK;
}

int ChessANParser::postprocMajorPiece(ChessANDecomp &decomp)
{
  ChessPos      src(decomp.m_posSrc); // source position
  ChessPos      dst(decomp.m_posDst); // destination position
  ChessColor    color;                // required piece color
  list_of_pos   positions;            // list of all source positions from dst
  list_of_pos   filtered;             // filtered on (partial) src 
  size_t        i;
  const string  pieceName(nameOfPiece(decomp.m_ePieceMoved));

  // assign piece color
  color = decomp.m_ePlayer != NoColor? decomp.m_ePlayer: decomp.m_eColorMoved;


  ChessBoard::findMajorPieceMoves(decomp.m_ePieceMoved, dst, positions);
  ChessBoard::filterPositions(src, positions, filtered);

#ifdef AN_DBG
  cerr << "DBG: From '" << dst << "' there are " << positions.size()
    << " candiate source positions:" << endl;
  cerr << "  ";
  for(i = 0; i < positions.size(); ++i)
  {
    cerr << positions[i] << " ";
  }
  cerr << endl;

  cerr << "DBG: Filtered on '" << src << "' there are now " << filtered.size()
    << " candiate source positions:" << endl;
  cerr << "  ";
  for(i = 0; i < filtered.size(); ++i)
  {
    cerr << filtered[i] << " ";
  }
  cerr << endl;
#endif

  //
  // Source and destination positions fully specified, simply verify.
  //
  if( src.isSpecified() )
  {
    // "No move" move
    if( src == dst )
    {
      AN_ERROR(pieceName
          << " move "
          << decomp.m_posSrc << decomp.m_posDst
          << " goes no where.");
      return -CE_ECODE_CHESS_PARSE;
    }

    // multiple positions, but there should only be 1
    else if( filtered.size() != 1 )
    {
      AN_ERROR(pieceName
          << " move "
          << decomp.m_posSrc << decomp.m_posDst
          << " is not legal.");
      return -CE_ECODE_CHESS_PARSE;
    }
  }

  //
  // Even though source is not fully specified, there exist only one source
  // position possible.
  //
  else if( filtered.size() == 1 )
  {
    src = filtered[0];
  }

  //
  // Ok
  //
  decomp.m_eColorMoved  = color;
  decomp.m_posSrc       = src;
  decomp.m_posDst       = dst;

  return CE_OK;
}

void ChessANParser::toChessMove(const ChessANDecomp &decomp, ChessMove &move)
{
  move.m_strAN          = decomp.m_strAN;
  move.m_ePieceMoved    = decomp.m_ePieceMoved;
  move.m_posSrc         = decomp.m_posSrc;
  move.m_posDst         = decomp.m_posDst;
  move.m_ePieceCaptured = decomp.m_ePieceCaptured;
  move.m_ePiecePromoted = decomp.m_ePiecePromoted;
  move.m_bIsEnPassant   = decomp.m_bIsEnPassant;
  move.m_eCastling      = decomp.m_eCastling;
  move.m_eCheck         = decomp.m_eCheck;
}

ChessANParser::Method
                  ChessANParser::setParserMethod(ChessANParser::Method eMethod)
{
  switch( eMethod )
  {
    case MethodBNF:
    case MethodRegEx:
      m_eMethod = eMethod;
      break;
    default:
      break;
  }
  return m_eMethod;
}

void ChessANParser::clearErrors()
{
  IMPL_GRAMMAR(m_impl).m_ssError.str("");
  m_strError.clear();
}
