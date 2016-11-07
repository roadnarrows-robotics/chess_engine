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

#include <boost/config/warning_disable.hpp>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_fusion.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/phoenix_object.hpp>

#include <boost/fusion/include/adapt_struct.hpp>

#include <iostream>
#include <string>
#include <map>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceUtils.h"


//------------------------------------------------------------------------------
// Chess Algebraic Notation Parser
//------------------------------------------------------------------------------

namespace chess_engine
{

/* RDK
  struct chess_pos
  {
    char  m_file;
    char  m_rank;

    chess_pos()
    {
      m_file = NoFile;
      m_rank = NoRank;
    }

    chess_pos(const chess_pos &src)
    {
      m_file = src.m_file;
      m_rank = src.m_rank;
    }

    void clear()
    {
      m_file = NoFile;
      m_rank = NoRank;
    }
  };
*/

  // --------------------------------------------------------------------------
  // an_move structure
  // --------------------------------------------------------------------------

  /*!
   * \brief Algebraic Notation parsed resultant object class.
   */
  struct an_move
  {
    std::string   m_strAN;          ///< associated AN string describing move
    ChessAlgebra  m_algebra;        ///< chess algebra notation
    ChessPiece    m_piece;          ///< moved piece
    ChessPos      m_src;            ///< source chess position
    ChessPos      m_dst;            ///< destination chess position
    bool          m_capture;        ///< [not] a capture move
    bool          m_en_passant;     ///< [not] en passant move
    ChessCastling m_castling;       ///< castling move, if any
    ChessPiece    m_promotion;      ///< promoted piece, if any
    ChessCheckMod m_check;          ///< check or mate, if any

    /*!
     * \brief Default constructor.
     */
    an_move()
    {
      clear();
    };

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    an_move(const an_move &src)
    {
      m_strAN       = src.m_strAN;
      m_algebra     = src.m_algebra;
      m_piece       = src.m_piece;
      m_src         = src.m_src;
      m_dst         = src.m_dst;
      m_capture     = src.m_capture;
      m_en_passant  = src.m_en_passant;
      m_castling    = src.m_castling;
      m_promotion   = src.m_promotion;
      m_check       = src.m_check;
    }

    /*!
     * \brief Clear data.
     */
    void clear()
    {
      m_strAN.clear();
      m_algebra     = UnknownAN;
      m_piece       = NoPiece;
      m_src.clear();
      m_dst.clear();
      m_capture     = false;
      m_en_passant  = false;
      m_castling    = NoCastling;
      m_promotion   = NoPiece;
      m_check       = NoCheckMod;
    }

    /*!
     * \brief Associtate the AN that populated the move fields.
     *
     * \param strAN   AN string.
     */
    void associateAN(const std::string &strAN)
    {
      m_strAN = strAN;
    }

    //
    // Friends
    //
    friend std::ostream &operator<<(std::ostream &os, const an_move &move);
  };

  /*!
   * \brief The an_move output stream operator.
   */
  std::ostream &operator<<(std::ostream &os, const an_move &move)
  {
    os
      << "{" << std::endl
        << "  AN         = " << move.m_strAN << std::endl
        << "  algebra    = "
          << nameOfAlgebra(move.m_algebra)
          << "(" << move.m_algebra << ")"
          << std::endl
        << "  piece      = "
          << nameOfPiece(move.m_piece)
          << "(" << (char)move.m_piece << ")"
          << std::endl
        << "  srcdst     = "
          << (char)move.m_src.m_file << (char)move.m_src.m_rank
          << (char)move.m_dst.m_file << (char)move.m_dst.m_rank
          << std::endl
        << "  capture    = "
          << nameOfBool(move.m_capture)
          << "(" << move.m_capture << ")" << std::endl
        << "  en_passant = "
          << nameOfBool(move.m_en_passant) << "(" << move.m_en_passant << ")"
          << std::endl
        << "  castling   = "
          << nameOfCastling(move.m_castling)
          << "(" << (char)move.m_castling << ")"
          << std::endl
        << "  promotion  = "
          << nameOfPiece(move.m_promotion)
          << "(" << (char)move.m_promotion << ")"
          << std::endl
        << "  check      = "
          << nameOfCheckMod(move.m_check)
          << "(" << (char)move.m_check << ")"
          << std::endl
      << "}" << std::endl
    ;

    return os;
  }

} // namespace chess_engine

//
// Adapt ChessPos structure for parser. Keep in global scope.
//
BOOST_FUSION_ADAPT_STRUCT(
    chess_engine::ChessPos,
    (chess_engine::ChessFile, m_file)   // 0
    (chess_engine::ChessFile, m_rank)   // 1
)

//
// Adapted ChessPos member access helpers.
//
#define POS_FILE(p)     at_c<0>(p)  ///< position file
#define POS_RANK(p)     at_c<1>(p)  ///< position rank

//
// Adapt an_move structure for parser. Keep in global scope.
//
BOOST_FUSION_ADAPT_STRUCT(
    chess_engine::an_move,
    (chess_engine::ChessAlgebra, m_algebra)   // 0
    (chess_engine::ChessPiece, m_piece)       // 1
    (chess_engine::ChessPos, m_src)           // 2
    (chess_engine::ChessPos, m_dst)           // 3
    (bool, m_capture)                         // 4
    (bool, m_en_passant)                      // 5
    (chess_engine::ChessPiece, m_promotion)   // 6
    (chess_engine::ChessCheckMod, m_check)    // 7
    (chess_engine::ChessCastling, m_castling) // 8
)

//
// Adapted an_move member access helpers.
//
#define MOVE_ALGEBRA(m)     at_c<0>(m)  ///< move algebra
#define MOVE_PIECE(m)       at_c<1>(m)  ///< move piece
#define MOVE_SRC_POS(m)     at_c<2>(m)  ///< move source position
#define MOVE_DST_POS(m)     at_c<3>(m)  ///< move destination position
#define MOVE_CAPTURE(m)     at_c<4>(m)  ///< move capture
#define MOVE_EN_PASSANT(m)  at_c<5>(m)  ///< move en passant
#define MOVE_PROMOTION(m)   at_c<6>(m)  ///< move promotion piece
#define MOVE_CHECK(m)       at_c<7>(m)  ///< move check modifier
#define MOVE_CASTLING(m)    at_c<8>(m)  ///< move castling type

//
// Rule helpers.
//
#define RULE_NAME(rule) rule.name(#rule)

namespace chess_engine
{
  namespace qi = boost::spirit::qi;
  namespace ascii = boost::spirit::ascii;

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
  

  //----------------------------------------------------------------------------
  // Grammar Template
  //----------------------------------------------------------------------------
  template <typename Iterator> struct an_grammar :
      qi::grammar<Iterator, an_move()>
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
        // Grammer rules.
        // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
        
        //
        // Algebraic Notation
        //
        an =
          eps >>
          (
              can [_val = _1, MOVE_ALGEBRA(_val) = CAN]
            | san [_val = _1, MOVE_ALGEBRA(_val) = SAN]
          )
        ;

        RULE_NAME(an);

        //
        // Coordinate Algebraic Notation
        //
        can =
            can_promotion  [_val = _1]
          |
                pos [MOVE_SRC_POS(_val) = _1]
            >>  pos [MOVE_DST_POS(_val) = _1]
        ;

        RULE_NAME(can);

        //
        // CAN pawn promotion. Examples: "(Q) "=N".
        //
        can_promotion = 
                promotion_src_pos [MOVE_SRC_POS(_val) = _1]
            >>  promotion_dst_pos [MOVE_DST_POS(_val) = _1]
            >>  promotion         [MOVE_PROMOTION(_val) = _1,
                                   MOVE_PIECE(_val) = Pawn]
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
          >   -(check_or_mate) [MOVE_CHECK(_val) = _1]
        ;

        RULE_NAME(san);

        //
        // SAN castling move. Examples: "0-0" "0-0-0" "O-O".
        //
        san_castling_move =
          castling [MOVE_CASTLING(_val) = _1, MOVE_PIECE(_val) = King]
        ;

        RULE_NAME(san_castling_move);

        //
        // SAN en passant move. Examples: "dxc6e.p" "hxg3e.p".
        //
        san_en_passant_move =
              disambig_pawn       [MOVE_SRC_POS(_val) = _1]
          >>  capture_mod         [MOVE_CAPTURE(_val) = true]
          >>  en_passant_dst_pos  [MOVE_DST_POS(_val) = _1]
          >>  en_passant_mod      [MOVE_EN_PASSANT(_val) = true,
                                   MOVE_PIECE(_val) = Pawn]
        ;

        RULE_NAME(san_en_passant_move);

        //
        // SAN pawn promotion move. Examples: "b8=Q" "b1/Q" "h8N" "gxf8(N)".
        //
        san_promotion_move =
                promotion_dst_pos   [MOVE_DST_POS(_val) = _1]
            >>  promotion           [MOVE_PROMOTION(_val) = _1, 
                                     MOVE_PIECE(_val) = Pawn]
          |
                disambig_pawn [MOVE_SRC_POS(_val) = _1]
            >>  capture_mod   [MOVE_CAPTURE(_val) = true]
            >>  promotion_dst_pos   [MOVE_DST_POS(_val) = _1]
            >>  promotion           [MOVE_PROMOTION(_val) = _1, 
                                     MOVE_PIECE(_val) = Pawn]
        ;

        RULE_NAME(san_promotion_move);

        //
        // SAN capture move.
        //
        san_capture_move = 
            san_pawn_capturing   [_val = _1, MOVE_PIECE(_val) = Pawn]
          | san_major_capturing  [_val = _1]
        ;

        RULE_NAME(san_capture_move);

        //
        // SAN pawn capturing move. Examples: "exd7" "axb4".
        //
        san_pawn_capturing =
              disambig_pawn [MOVE_SRC_POS(_val) = _1]
          >>  capture_mod   [MOVE_CAPTURE(_val) = true]
          >   pos           [MOVE_DST_POS(_val) = _1,
                             MOVE_PIECE(_val) = Pawn]
        ;

        RULE_NAME(san_pawn_capturing);

        //
        // SAN major piece capturing move. Examples: "Bxe5" "Ngxf3".
        //
        san_major_capturing =
              major_piece [MOVE_PIECE(_val) = _1]
          >>  -(disambig) [MOVE_SRC_POS(_val) = _1]
          >>  capture_mod [MOVE_CAPTURE(_val) = true]
          >   pos         [MOVE_DST_POS(_val) = _1]
        ;

        RULE_NAME(san_major_capturing);

        //
        // SAN basic move (i.e. no capture, not special move).
        //
        san_basic_move = 
            san_pawn_move   [_val = _1, MOVE_PIECE(_val) = Pawn]
          | san_major_move  [_val = _1]
        ;

        RULE_NAME(san_basic_move);

        //
        // SAN pawn basic move. Examples: "c3" "e5" "de3".
        //
        san_pawn_move =
            pos               [MOVE_DST_POS(_val) = _1]
          |
                file_or_rank  [MOVE_SRC_POS(_val) = _1]
            >>  pos           [MOVE_DST_POS(_val) = _1,
                               MOVE_PIECE(_val) = Pawn]
        ;
        
        RULE_NAME(san_pawn_move);

        //
        // SAN major piece basic move. Examples: "Nc3" "Kd2" "Rdd5" "N5f3".
        //
        san_major_move =
                major_piece [MOVE_PIECE(_val) = _1]
            >>  pos         [MOVE_DST_POS(_val) = _1]
          |
                major_piece [MOVE_PIECE(_val) = _1]
            >>  disambig    [MOVE_SRC_POS(_val) = _1]
            >>  pos         [MOVE_DST_POS(_val) = _1]
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
        file_only_pos = file  [POS_FILE(_val) = _1];

        RULE_NAME(file_only_pos);

        //
        // Rank only position.
        //
        rank_only_pos = rank  [POS_RANK(_val) = _1];

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

        RULE_NAME(disambig);

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
    qi::rule<Iterator, an_move()> an;

    // CAN
    qi::rule<Iterator, an_move()> can;
    qi::rule<Iterator, an_move()> can_promotion;

    // SAN
    qi::rule<Iterator, an_move()> san;
    qi::rule<Iterator, an_move()> san_castling_move;
    qi::rule<Iterator, an_move()> san_en_passant_move;
    qi::rule<Iterator, an_move()> san_promotion_move;
    qi::rule<Iterator, an_move()> san_capture_move;
    qi::rule<Iterator, an_move()> san_pawn_capturing;
    qi::rule<Iterator, an_move()> san_major_capturing;
    qi::rule<Iterator, an_move()> san_basic_move;
    qi::rule<Iterator, an_move()> san_pawn_move;
    qi::rule<Iterator, an_move()> san_major_move;


    // piece
    qi::rule<Iterator, ChessPiece()>  promotion;

    // position
    qi::rule<Iterator, chess_pos()> file_only_pos;
    qi::rule<Iterator, chess_pos()> rank_only_pos;
    qi::rule<Iterator, chess_pos()> file_or_rank;
    qi::rule<Iterator, chess_pos()> pos;
    qi::rule<Iterator, chess_pos()> disambig;
    qi::rule<Iterator, chess_pos()> disambig_pawn;
    qi::rule<Iterator, chess_pos()> en_passant_src_pos;
    qi::rule<Iterator, chess_pos()> en_passant_dst_pos;
    qi::rule<Iterator, chess_pos()> promotion_src_pos;
    qi::rule<Iterator, chess_pos()> promotion_dst_pos;

    // modifier
    qi::rule<Iterator, ChessCheckMod()> check_or_mate;

    std::stringstream m_ssError;
  };

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

  an_grammar    m_grammar;    ///< the grammar
  an_move       m_an_move;    ///< parsed output

  /*!
   * \brief Default constructor.
   */
  impl() { }

  /*!
   * \brief Destructor.
   */
  virtual ~impl() { };
};

//
// Implementation access macros.
//
#define IMPL(p)         ((impl *)p)
#define IMPL_GRAMMAR(p) IMPL(p)->m_grammar
#define IMPL_MOVE(p)    IMPL(p)->m_an_move


//----------------------------------------------------------------------------
// Class ChessANparser
//----------------------------------------------------------------------------

/*! parse error preface string */
const static std::string ErrorPreface("Parse error: ");

chess_engine::ChessANParser()
{
  m_impl    = new impl;
  m_bTrace  = false;
}

~chess_engine::ChessANParser()
{
  delete IMPL(m_impl);
}

bool chess_engine::ChessANParser::parse(const std::string &strAN)
{
  std::string::const_iterator iter  = strAN.begin();
  std::string::const_iterator end   = strAN.end();
  
  bool r;   // parse result

  if( m_bTrace )
  {
    std::cout << "Input: " << strAN << std::endl << std::endl;
  }

  // clear errors
  clearErrors();

  r = qi::parse(iter, end, IMPL_GRAMMAR(m_impl), IMPL_MOVE(m_impl));

  // do this after parse which clears move data prior to parse
  IMPL_MOVE(m_impl).associateAN(strAN);

  // unparsed trailing tokens
  if( iter != end )
  {
    r = false;
  }

  if( m_bTrace )
  {
    std::cout << "parsed move: " << IMPL_MOVE(m_impl) << std::endl;
  }

  if( r )
  {
    r = postprocess();

    if( r && m_bTrace )
    {
      std::cout << "post-processed move: " << IMPL_MOVE(m_impl) << std::endl;
    }
  }
  else
  {
    std::string strError(IMPL_GRAMMAR(m_impl).m_ssError.str());

    if( strError.empty() )
    {
      std::string sstr(iter, end);

      strError = "Bad syntax, stopped at \"" + sstr + "\"";
    }

    m_strError = ErrorPreface + strError;
  }

  if( !r )
  {
    if( m_bTrace )
    {
      std::cout << m_strError << std::endl;
    }
  }

  return r;
}

bool chess_engine::ChessANParser::postprocess()
{
  if( IMPL_MOVE(m_impl).m_castling != NoCastling )
  {
    return postprocCastling();
  }

  else if( IMPL_MOVE(m_impl).m_en_passant )
  {
    return postprocEnPassant();
  }

  else if( IMPL_MOVE(m_impl).m_promotion != NoPiece )
  {
    return postprocPromotion();
  }

  // can add other move here, especially pawn moves
 
  else
  {
    return true;
  }
}

//
// Castling.
//
// Without piece color, only source and destinaion files are known.
//
bool chess_engine::ChessANParser::postprocCastling()
{
  chess_engine::an_move &m = IMPL_MOVE(m_impl);

  //
  // Kingside castling.
  //
  if( m.m_castling == KingSide )
  {
    m.m_src.m_file = ChessFileE;
    m.m_dst.m_file = ChessFileG;
  }

  //
  // Queenside castling.
  //
  else if( m.m_castling == QueenSide )
  {
    m.m_src.m_file = ChessFileE;
    m.m_dst.m_file = ChessFileC;
  }

  return true;
}

//
// En Passant.
//
// Without game state, only the source rank can be determined.
//
bool chess_engine::ChessANParser::postprocEnPassant()
{
  chess_engine::an_move &m = IMPL_MOVE(m_impl);

  ChessRank         rank;
  std::stringstream ssError;
  bool              verified = false;

  //
  // Determine source rank.
  //
  switch( m.m_dst.m_rank )
  {
    case ChessRank6:        // white
      rank = ChessRank5;
      break;
    case ChessRank3:        // black
      rank = ChessRank4;
      break;
    default:
      ssError << ErrorPreface
        << "En Passant bad destination rank \""
        << (char)m.m_dst.m_rank
        << "\"";
      m_strError = ssError.str();
      return false;
  }

  // set source rank
  if( m.m_src.m_rank == NoRank )
  {
    m.m_src.m_rank = rank;
  }

  // verify source rank
  else if( m.m_src.m_rank != rank )
  {
    ssError << ErrorPreface
      << "En Passant bad source rank \""
      << (char)m.m_src.m_rank
      << "\"";
    m_strError = ssError.str();
    return false;
  }

  //
  // Verify source file.
  //
  if( m.m_src.m_file == NoFile )
  {
    verified = true;  // nothing to verify
  }

  // en passant move to the right
  else if( m.m_src.m_file == shiftFile(m.m_dst.m_file, -1) )
  {
    verified = true;
  }

  // en passant move to the left
  else if( m.m_src.m_file == shiftFile(m.m_dst.m_file, 1) )
  {
    verified = true;
  }

  if( !verified )
  {
    ssError << ErrorPreface
      << "En Passant bad source file \""
      << (char)m.m_src.m_file
      << "\"";
    m_strError = ssError.str();
  }

  return verified;
}

//
// Pawn promotion.
//
// Without game state the source can only be partially determined.
//
bool chess_engine::ChessANParser::postprocPromotion()
{
  chess_engine::an_move &m = IMPL_MOVE(m_impl);

  ChessRank         rank;
  std::stringstream ssError;
  bool              verified = false;

  //
  // Determine source rank.
  //
  switch( m.m_dst.m_rank )
  {
    case ChessRank8:      // white
      rank = ChessRank7;
      break;
    case ChessRank1:
      rank = ChessRank2;  // black
      break;
    default:
      ssError << ErrorPreface
        << "Promotion bad destination rank \""
        << (char)m.m_dst.m_rank
        << "\"";
      m_strError = ssError.str();
      return false;
  }

  // set source rank
  if( m.m_src.m_rank == NoRank )
  {
    m.m_src.m_rank = rank;
  }

  // verify source rank
  else if( m.m_src.m_rank != rank )
  {
    ssError << ErrorPreface
      << "Promotion bad source rank \""
      << (char)m.m_src.m_rank
      << "\"";
    m_strError = ssError.str();
    return false;
  }

  //
  // CAN algebra move. Note that CAN drives capture state.
  //
  if( m.m_algebra == CAN )
  {
    // no capture move
    if( m.m_src.m_file == (ChessFile)m.m_dst.m_file )
    {
      verified = true;
    }

    // capture move to the right
    else if( m.m_src.m_file == shiftFile(m.m_dst.m_file, -1) )
    {
      m.m_capture = true;
      verified    = true;
    }

    // capture move to the left
    else if( m.m_src.m_file == shiftFile(m.m_dst.m_file, 1) )
    {
      m.m_capture = true;
      verified    = true;
    }
  }

  //
  // SAN algebra move with capture.
  //
  else if( m.m_capture )
  {
    // no source specified
    if( m.m_src.m_file == NoFile )
    {
      verified = true; // nothing to to verify
    }

    // capture move to the right
    else if( m.m_src.m_file == shiftFile(m.m_dst.m_file, -1) )
    {
      verified = true;
    }

    // capture move to the left
    else if( m.m_src.m_file == shiftFile(m.m_dst.m_file, 1) )
    {
      verified = true;
    }
  }

  //
  // SAN algebra move without capture.
  //
  else
  {
    // set source file
    if( m.m_src.m_file == NoFile )
    {
      m.m_src.m_file  = m.m_dst.m_file;
      verified        = true;
    }

    // pawn moves in same file
    else if( m.m_src.m_file == m.m_dst.m_file )
    {
      verified = true;
    }
  }

  if( !verified )
  {
    ssError << ErrorPreface
      << "Promotion bad source file \""
      << (char)m.m_src.m_file
      << "\"";
    m_strError = ssError.str();
  }

  return verified;
}

void chess_engine::ChessANParser::clearErrors()
{
  IMPL_GRAMMAR(m_impl).m_ssError.str("");
  m_strError.clear();
}


////////////////////////////////////////////////////////////////////////////
//  Main program
////////////////////////////////////////////////////////////////////////////
int main()
{
  std::cout << "/////////////////////////////////////////////////////////\n\n";
  std::cout << "\t\tAlgebraic Notation parser\n\n";
  std::cout << "/////////////////////////////////////////////////////////\n\n";

  std::cout << "Enter CAN or SAN\n";
  std::cout << "Type q to quit\n\n";

  bool  trace = true;

  chess_engine::ChessANParser  parser;

  parser.setTracing(trace);

  std::string str;

  while( true )
  {
    std::cout << "an> ";

    if( !getline(std::cin, str) )
    {
      break;
    }
    else if (str.empty() || str[0] == 'q' )
    {
      break;
    }

    if( parser.parse(str) )
    {
      //std::cout << "good" << std::endl;
    }
    else if( !trace )
    {
      std::cout << parser.getErrorStr() << std::endl;
    }
  }

  std::cout << "\nbye\n";

  return 0;
}
