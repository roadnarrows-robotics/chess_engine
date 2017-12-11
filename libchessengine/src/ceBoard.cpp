////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceBoard.cpp
//
/*! \file
 *
 * \brief The chess board state implementation.
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

#include <stdio.h>
#include <ctype.h>

#include <string>
#include <vector>
#include <map>

#include "boost/assign.hpp"

#include "rnr/color.h"
#include "rnr/appkit/LogStream.h"

#include "chess_engine/ceTypes.h"
#include "chess_engine/ceError.h"
#include "chess_engine/ceUtils.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceBoard.h"

using namespace std;
using namespace boost::assign;
using namespace chess_engine;


// -----------------------------------------------------------------------------
// Private
// -----------------------------------------------------------------------------

namespace chess_engine
{
  static const string ColorWhiteSq(ANSI_COLOR(ANSI_SGR_TEXT_NORMAL,
                                              ANSI_SGR_FG_COLOR_BLACK,
                                              ANSI_SGR_BG_COLOR_CYAN));
  static const string ColorBlackSq(ANSI_COLOR(ANSI_SGR_TEXT_NORMAL,
                                              ANSI_SGR_FG_COLOR_BLACK,
                                              ANSI_SGR_BG_COLOR_WHITE));
  static const string strReset(ANSI_COLOR_RESET);

  static ChessSquare  OffBoardSquare;  ///< sacrificial off-board empty square

  /*!
   * \brief Convenient structure to hold two integers.
   */
  struct twoints
  {
    twoints()
    {
      x = 0;
      y = 0;
    }

    twoints(int a, int b)
    {
      x = a;
      y = b;
    }

    int x;
    int y;
  };  // struct twoints

  /*!
   * \brief Chess board direction deltas.
   */
  static const map<ChessBoardDir, twoints> DirDeltas = map_list_of
    (DirNone,       twoints(0, 0))
    (DirUp,         twoints(0, 1))
    (DirUpperLeft,  twoints(-1, 1))
    (DirLeft,       twoints(-1, 0))
    (DirLowerLeft,  twoints(-1, -1))
    (DirDown,       twoints(0, -1))
    (DirLowerRight, twoints(1, -1))
    (DirRight,      twoints(1, 0))
    (DirUpperRight, twoints(1, 1))
  ;

  /*!
   * \brief King allowed movement directions.
   */
  ChessBoardDir KingDirs[] =
  {
    DirUp,   DirUpperLeft,  DirLeft,  DirLowerLeft,
    DirDown, DirLowerRight, DirRight, DirUpperRight,
    DirNone
  };

  /*!
   * \brief Queen allowed movement directions.
   */
  ChessBoardDir *QueenDirs = KingDirs;

  /*!
   * \brief Bishop allowed movement directions.
   */
  ChessBoardDir BishopDirs[] =
  {
    DirUpperLeft,
    DirLowerLeft,
    DirLowerRight,
    DirUpperRight,
    DirNone
  };

  vector<twoints> KnightMoveDeltas = list_of
    (twoints(-1,  2)) (twoints( 1,  2))
    (twoints( 2,  1)) (twoints( 2, -1))
    (twoints( 1, -2)) (twoints(-1, -2))
    (twoints(-2, -1)) (twoints(-2,  1))
  ;

  /*!
   * \brief Rook allowed movement directions.
   */
  ChessBoardDir RookDirs[] =
  {
    DirUp, DirLeft, DirDown, DirRight, DirNone
  };

  /*!
   * \brief White pawn allowed movement directions.
   */
  ChessBoardDir WhitePawnDirs[] =
  {
    DirUp, DirUpperLeft, DirUpperRight, DirNone
  };

  /*!
   * \brief Black pawn allowed movement directions.
   */
  ChessBoardDir BlackPawnDirs[] =
  {
    DirDown, DirLowerLeft, DirLowerRight, DirNone
  };

  /*!
   * \brief White pawn source movement directions.
   */
  ChessBoardDir *WhitePawnSrcDirs = BlackPawnDirs;

  /*!
   * \brief Black pawn source movement directions.
   */
  ChessBoardDir *BlackPawnSrcDirs = WhitePawnDirs;

} // namespace chess_engine


// -----------------------------------------------------------------------------
// Class ChessBoard
// -----------------------------------------------------------------------------

ChessBoard::ChessBoard()
{
  initBoard();

  m_bGraphic = false;
}

void ChessBoard::clearBoard()
{
  for(int col=0; col<NumOfFiles; ++col)
  {
    for(int row=0; row<NumOfRanks; ++row)
    {
      m_board[row][col].removePiece();
    }
  }
}

void ChessBoard::setupBoard()
{
  clearBoard();

  setupBoard(White);
  setupBoard(Black);
}

void ChessBoard::setPiece(const ChessPos    &pos,
                          const ChessColor   eColor,
                          const ChessPiece   ePiece,
                          const std::string &strId)
{
  at(pos).setPiece(eColor, ePiece, strId);
}

void ChessBoard::movePiece(const ChessPos &posSrc, const ChessPos &posDst)
{
  at(posSrc).movePiece(at(posDst));
}

void ChessBoard::removePiece(const ChessPos &pos)
{
  at(pos).removePiece();
}

const ChessSquare &ChessBoard::at(const int file, const int rank) const
{
  return at(ChessPos(file, rank));
}

ChessSquare &ChessBoard::at(const int file, const int rank)
{
  return at(ChessPos(file, rank));
}

const ChessSquare &ChessBoard::at(const ChessPos &pos) const
{
  int   row, col;

  if( toRowCol(pos, row, col) == CE_OK )
  {
    return m_board[row][col];
  }
  else
  {
    return NoSquare;
  }
}

ChessSquare &ChessBoard::at(const ChessPos &pos)
{
  int   row, col;

  if( toRowCol(pos, row, col) == CE_OK )
  {
    return m_board[row][col];
  }
  else
  {
    OffBoardSquare = NoSquare;
    return OffBoardSquare;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Board Positioning Member Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int ChessBoard::toRow(int rank)
{
  //fprintf(stderr, "DBG: toRow(rank=%c) = %d\n",
  //    rank, NumOfRanks - (rank - (int)ChessRank1) - 1);
  return NumOfRanks - (rank - (int)ChessRank1) - 1;
}

int ChessBoard::toCol(int file)
{
  return file - (int)ChessFileA;
}

int ChessBoard::toRowCol(const ChessPos &pos, int &row, int &col)
{
  row = toRow(pos.m_rank);
  col = toCol(pos.m_file);

  if( (row < 0) || (row >= NumOfRanks) || (col < 0) || (col >= NumOfFiles) )
  {
    return -CE_ECODE_CHESS_FATAL;
  }
  else
  {
    return CE_OK;
  }
}

ChessFile ChessBoard::toFile(int col)
{
  if( (col >= 0) && (col < NumOfFiles) )
  {
    return (ChessFile)((int)ChessFileA + col);
  }
  else
  {
    return NoFile;
  }
}

ChessRank ChessBoard::toRank(int row)
{
  if( (row >= 0) && (row < NumOfRanks) )
  {
    //fprintf(stderr, "DBG: toRank(row=%d) = %c\n",
    //    row, (ChessRank)((int)ChessRank1 + NumOfRanks - row - 1));
    return (ChessRank)((int)ChessRank1 + NumOfRanks - row - 1);
  }
  else
  {
    return NoRank;
  }
}

ChessFile ChessBoard::shiftFile(int file, int offset)
{
  int shift = file + offset;

  return (shift >= ChessFileA) && (shift <= ChessFileH)?
                                                (ChessFile)shift: NoFile;
}

ChessRank ChessBoard::shiftRank(int rank, int offset)
{
  int shift = rank + offset;

  return (shift >= ChessRank1) && (shift <= ChessRank8)?
                                                (ChessRank)shift: NoRank;
}

bool ChessBoard::isOnChessBoard(const ChessPos &pos)
{
  int   row, col;

  return toRowCol(pos, row, col) == CE_OK? true: false;
}

bool ChessBoard::isOnChessBoard(const ChessFile file, const ChessRank rank)
{
  return isOnChessBoard(ChessPos(file, rank));
}

ChessColor ChessBoard::getSquareColor(int file, int rank)
{
  return ChessSquare::colorOfSquare(file, rank);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Find Board Positions Member Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

bool ChessBoard::findMoveSrcPos(ChessMove &move)
{
  list_of_pos   positions;
  ChessPos     &src = move.m_posSrc;
  size_t        i;

  //cerr << "DBG: " << __func__ << " move_in " << move << endl;

  switch( move.m_ePieceMoved )
  {
    case King:
      return findKingMoveSrcPos(move);
    case Queen:
      return findQueenMoveSrcPos(move);
    case Bishop:
      return findBishopMoveSrcPos(move);
    case Knight:
      return findKnightMoveSrcPos(move);
    case Rook:
      return findRookMoveSrcPos(move);
    case Pawn:
      return findPawnMoveSrcPos(move);
    default:
      return false;
  }
}

bool ChessBoard::findKingMoveSrcPos(ChessMove &move)
{
  list_of_pos positions;    // all possible source positions
  list_of_pos candidates;   // working candidates with piece
  size_t      i;            // working index

  //
  // Move type.
  //
  switch( move.m_eCastling )
  {
    case NoCastling:
      rangeOfKing(move.m_posDst, positions);
      break;
    case KingSide:
    case QueenSide:
      if( move.m_ePlayer == White )
      {
        positions.push_back(CastlingPosWhiteKSrc);
      }
      else if( move.m_ePlayer == Black )
      {
        positions.push_back(CastlingPosBlackKSrc);
      }
      break;
    default:
      break;
  }

  //
  // Find all kings.
  //
  for(i = 0; i < positions.size(); ++i)
  {
    // board square
    ChessSquare &sq = at(positions[i]);
    
    if( (sq.getPieceColor() == move.m_ePlayer) &&
        (sq.getPieceType()  == move.m_ePieceMoved) )
    {
      candidates.push_back(positions[i]);
    }
  }

  return testAndSetMoveSrc(move, candidates);
}

bool ChessBoard::findQueenMoveSrcPos(ChessMove &move)
{
  list_of_pos positions;      // source positions in one direction
  list_of_pos candidates;     // working candidates with piece
  size_t      i, j;           // working indices

  //
  // For each possible direction of movement, get the list of the board source
  // positions.
  //
  for(i = 0; QueenDirs[i] != DirNone; ++i)
  {
    positions.clear();

    findPositions(move.m_posDst, QueenDirs[i], EndOfBoard, positions);

    //
    // Then only add if the piece is found.
    //
    for(j = 0; j < positions.size(); ++j)
    {
      // board square
      ChessSquare &sq = at(positions[j]);
    
      if( !sq.isEmpty() )
      {
        // found a source candidate
        if( (sq.getPieceColor() == move.m_ePlayer) &&
            (sq.getPieceType()  == move.m_ePieceMoved) )
        {
          candidates.push_back(positions[j]);
        }

        // move is now blocked in this direction
        break;
      }
    }
  }

  return testAndSetMoveSrc(move, candidates);
}

bool ChessBoard::findBishopMoveSrcPos(ChessMove &move)
{
  list_of_pos positions;      // source positions in one direction
  list_of_pos candidates;     // working candidates with piece
  size_t      i, j;           // working indices

  //
  // For each possible direction of movement, get the list of the board source
  // positions.
  //
  for(i = 0; BishopDirs[i] != DirNone; ++i)
  {
    positions.clear();

    findPositions(move.m_posDst, BishopDirs[i], EndOfBoard, positions);

    //
    // Then only add if the piece is found.
    //
    for(j = 0; j < positions.size(); ++j)
    {
      // board square
      ChessSquare &sq = at(positions[j]);
    
      if( !sq.isEmpty() )
      {
        // found a source candidate
        if( (sq.getPieceColor() == move.m_ePlayer) &&
            (sq.getPieceType()  == move.m_ePieceMoved) )
        {
          candidates.push_back(positions[j]);
        }

        // move is now blocked in this direction
        break;
      }
    }
  }

  return testAndSetMoveSrc(move, candidates);
}

bool ChessBoard::findKnightMoveSrcPos(ChessMove &move)
{
  list_of_pos positions;    // all possible source positions
  list_of_pos candidates;   // working candidates
  size_t      i;            // working index

  // find all of possible source positions given the destination position
  rangeOfKnight(move.m_posDst, positions);

  // find the moved piece source position
  for(i = 0; i < positions.size(); ++i)
  {
    // board square
    ChessSquare &sq = at(positions[i]);
    
    if( (sq.getPieceColor() == move.m_ePlayer) &&
        (sq.getPieceType()  == move.m_ePieceMoved) )
    {
      candidates.push_back(positions[i]);
    }
  }

  return testAndSetMoveSrc(move, candidates);
}

bool ChessBoard::findRookMoveSrcPos(ChessMove &move)
{
  list_of_pos positions;      // source positions in one direction
  list_of_pos candidates;     // working candidates with piece
  size_t      i, j;           // working indices

  //
  // For each possible direction of movement, get the list of the board source
  // positions.
  //
  for(i = 0; RookDirs[i] != DirNone; ++i)
  {
    positions.clear();

    findPositions(move.m_posDst, RookDirs[i], EndOfBoard, positions);

    //
    // Then only add if the piece is found.
    //
    for(j = 0; j < positions.size(); ++j)
    {
      // board square
      ChessSquare &sq = at(positions[j]);
    
      if( !sq.isEmpty() )
      {
        // found a source candidate
        if( (sq.getPieceColor() == move.m_ePlayer) &&
            (sq.getPieceType()  == move.m_ePieceMoved) )
        {
          candidates.push_back(positions[j]);
        }

        // move is now blocked in this direction
        break;
      }
    }
  }

  return testAndSetMoveSrc(move, candidates);
}

bool ChessBoard::findPawnMoveSrcPos(ChessMove &move)
{
  list_of_pos positions;    // source positions in one direction
  list_of_pos candidates;   // working candidates with piece
  size_t      i;            // working index

  // find all of candidate source positions given the destination position
  rangeOfPawnSrc(move.m_ePlayer, move.m_posDst, positions);

  // not a capture move
  if( move.m_ePieceCaptured == NoPiece )
  {
    // find the moved piece source position
    for(i = 0; i < positions.size(); ++i)
    {
      // board square
      ChessSquare &sq = at(positions[i]);
    
      // found pawn in column 
      if( (sq.getPos().m_file == move.m_posDst.m_file) && !sq.isEmpty() &&
          (sq.getPieceColor() == move.m_ePlayer) &&
          (sq.getPieceType()  == move.m_ePieceMoved) )
      {
        candidates.push_back(positions[i]);
        break; // move is now blocked in this direction
      }
    }
  }

  // a capture move
  else
  {
    // find the moved piece source position
    for(i = 0; i < positions.size(); ++i)
    {
      // board square
      ChessSquare &sq = at(positions[i]);
    
      // found pawn along diagonal 
      if( (sq.getPos().m_file != move.m_posDst.m_file) && !sq.isEmpty() &&
          (sq.getPieceColor() == move.m_ePlayer) &&
          (sq.getPieceType()  == move.m_ePieceMoved) )
      {
        candidates.push_back(positions[i]);
      }
    }
  }

  return testAndSetMoveSrc(move, candidates);
}

bool ChessBoard::testAndSetMoveSrc(ChessMove &move, list_of_pos &candidates)
{
  list_of_pos filtered;

  //cerr << "DBG: " << __func__ << " candidates:";
  //for(size_t i = 0; i < candidates.size(); ++i)
  //{
  //  cerr << " " << candidates[i];
  //}
  //cerr << endl;

  // filter based on (partial) source that may be specified in move
  filterPositions(move.m_posSrc, candidates, filtered);
  
  // no source position found
  if( filtered.size() == 0 )
  {
    LOGERROR_STREAM("Could not find the move's source position." << endl  
        << "  Moved piece: " << nameOfPiece(move.m_ePieceMoved) << endl
        << "  Destination: " << move.m_posDst);
    return false;
  }

  // multiple source positions found
  else if( filtered.size() > 1 )
  {
    LOGERROR_STREAM("Ambiguous move's multiple source positions." << endl  
        << "  Moved piece: " << nameOfPiece(move.m_ePieceMoved) << endl
        << "  Destination: " << move.m_posDst << endl
        << "  Sources:     ");
    for(size_t i = 0; i < filtered.size(); ++i)
    {
      LOGERROR_STREAM("    " << filtered[i]);
    }
    return false;
  }

  // all good
  else
  {
    move.m_posSrc = filtered[0];
    //cerr << "DBG: " << __func__ << " move_out " << move << endl;
    return true;
  }
}

size_t ChessBoard::findAvailMovesInGame(const ChessPiece ePiece,
                                        const ChessColor eColor,
                                        const ChessPos   &pos,
                                        list_of_pos      &positions)
{
  switch( ePiece )
  {
    case King:
      return findAvailKingMovesInGame(eColor, pos, positions);
    case Queen:
      return findAvailQueenMovesInGame(eColor, pos, positions);
    case Bishop:
      return findAvailBishopMovesInGame(eColor, pos, positions);
    case Knight:
      return findAvailKnightMovesInGame(eColor, pos, positions);
    case Rook:
      return findAvailRookMovesInGame(eColor, pos, positions);
    case Pawn:
      return findAvailPawnMovesInGame(eColor, pos, positions);
    default:
      return 0;
  }
}

size_t ChessBoard::findAvailKingMovesInGame(const ChessColor eColor,
                                            const ChessPos   &pos,
                                            list_of_pos      &positions)
{
  size_t      n = positions.size(); // current number of positions in list
  list_of_pos candidates;           // working candidates
  size_t      i;                    // working index

  // find all of king's candidate board moves
  rangeOfKing(pos, candidates);

  //
  // Then only add if availble, given the current board state. 
  //
  for(i = 0; i < candidates.size(); ++i)
  {
    // board square
    ChessSquare &sq = at(candidates[i]);
    
    // square is empty
    if( sq.isEmpty() )
    {
      positions.push_back(candidates[i]);
    }

    // square is occupied by an opponent's piece
    else if( sq.getPieceColor() != eColor )
    {
      positions.push_back(candidates[i]);
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::findAvailQueenMovesInGame(const ChessColor eColor,
                                             const ChessPos   &pos,
                                             list_of_pos      &positions)
{
  size_t      n = positions.size(); // current number of positions in list
  list_of_pos candidates;           // working candidates
  size_t      i, j;                 // working indices

  //
  // For each possible direction of movement, get the list of the board
  // candidate positions.
  //
  for(i = 0; QueenDirs[i] != DirNone; ++i)
  {
    candidates.clear();

    findPositions(pos, QueenDirs[i], EndOfBoard, candidates);

    //
    // Then only add if availble, given the current board state. 
    //
    for(j = 0; j < candidates.size(); ++j)
    {
      // board square
      ChessSquare &sq = at(candidates[j]);
    
      // square is empty
      if( sq.isEmpty() )
      {
        positions.push_back(candidates[j]);
      }

      // square is occupied by this player's piece
      else if( sq.getPieceColor() == eColor )
      {
        break;
      }

      // square is occupied by an opponent's piece
      else
      {
        positions.push_back(candidates[j]);
        break;
      }
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::findAvailBishopMovesInGame(const ChessColor eColor,
                                              const ChessPos   &pos,
                                              list_of_pos      &positions)
{
  size_t      n = positions.size(); // current number of positions in list
  list_of_pos candidates;           // working candidates
  size_t      i, j;                 // working indices

  //
  // For each possible direction of movement, get the list of the board
  // candidate positions.
  //
  for(i = 0; BishopDirs[i] != DirNone; ++i)
  {
    candidates.clear();

    findPositions(pos, BishopDirs[i], EndOfBoard, candidates);

    //
    // Then only add if availble, given the current board state. 
    //
    for(j = 0; j < candidates.size(); ++j)
    {
      // board square
      ChessSquare &sq = at(candidates[j]);
    
      // square is empty
      if( sq.isEmpty() )
      {
        positions.push_back(candidates[j]);
      }

      // square is occupied by this player's piece
      else if( sq.getPieceColor() == eColor )
      {
        break;
      }

      // square is occupied by an opponent's piece
      else
      {
        positions.push_back(candidates[j]);
        break;
      }
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::findAvailKnightMovesInGame(const ChessColor eColor,
                                              const ChessPos   &pos,
                                              list_of_pos      &positions)
{
  size_t      n = positions.size(); // current number of positions in list
  list_of_pos candidates;           // working candidates
  size_t      i;                    // working index

  // Find all of knight's candidate board moves.
  rangeOfKnight(pos, candidates);

  //
  // Then only add if availble, given the current board state. 
  //
  for(i = 0; i < candidates.size(); ++i)
  {
    // board square
    ChessSquare &sq = at(candidates[i]);
    
    // square is empty
    if( sq.isEmpty() )
    {
      positions.push_back(candidates[i]);
    }

    // square is occupied by an opponent's piece
    else if( sq.getPieceColor() != eColor )
    {
      positions.push_back(candidates[i]);
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::findAvailRookMovesInGame(const ChessColor eColor,
                                            const ChessPos   &pos,
                                            list_of_pos      &positions)
{
  size_t      n = positions.size(); // current number of positions in list
  list_of_pos candidates;           // working candidates
  size_t      i, j;                 // working indices

  //
  // For each possible direction of movement, get the list of the board
  // candidate positions.
  //
  for(i = 0; RookDirs[i] != DirNone; ++i)
  {
    candidates.clear();

    findPositions(pos, RookDirs[i], EndOfBoard, candidates);

    //
    // Then only add if availble, given the current board state. 
    //
    for(j = 0; j < candidates.size(); ++j)
    {
      // board square
      ChessSquare &sq = at(candidates[j]);
    
      // square is empty
      if( sq.isEmpty() )
      {
        positions.push_back(candidates[j]);
      }

      // square is occupied by this player's piece
      else if( sq.getPieceColor() == eColor )
      {
        break;
      }

      // square is occupied by an opponent's piece
      else
      {
        positions.push_back(candidates[j]);
        break;
      }
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::findAvailPawnMovesInGame(const ChessColor eColor,
                                            const ChessPos   &pos,
                                            list_of_pos      &positions)
{
  size_t      n = positions.size(); // current number of positions in list
  list_of_pos candidates;           // working candidates
  size_t      i;                    // working index
  bool        isBlocked = false;    // pawn is [not] blocked

  // Find all of king's candidate board moves.
  rangeOfPawnDst(eColor, pos, candidates);

  //
  // Then only add if availble, given the current board state. 
  //
  for(i = 0; i < candidates.size(); ++i)
  {
    // board square
    ChessSquare &sq = at(candidates[i]);
    
    // diagonal capture move
    if( sq.getPos().m_file != pos.m_file )
    {
      // square is occupied by an opponent's piece - capture move available
      if( sq.getPieceColor() != eColor )
      {
        positions.push_back(candidates[i]);
      }
    }

    // non-capture move
    else
    {
      // square is empty and forward is not blocked
      if( sq.isEmpty() && !isBlocked )
      {
        positions.push_back(candidates[i]);
      }
      else
      {
        isBlocked = true;
      }
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::rangeOfMajorPiece(const ChessPiece ePiece,
                                     const ChessPos   &pos,
                                     list_of_pos      &positions)
{
  switch( ePiece )
  {
    case King:
      return rangeOfKing(pos, positions);
    case Queen:
      return rangeOfQueen(pos, positions);
    case Bishop:
      return rangeOfBishop(pos, positions);
    case Knight:
      return rangeOfKnight(pos, positions);
    case Rook:
      return rangeOfRook(pos, positions);
    case Pawn:
    default:
      return 0;
  }
}

size_t ChessBoard::rangeOfKing(const ChessPos &pos, list_of_pos &positions)
{
  size_t        n = positions.size();

  for(size_t i = 0; KingDirs[i] != DirNone; ++i)
  {
    findPositions(pos, KingDirs[i], 1, positions);
  }

  return positions.size() - n;
}

size_t ChessBoard::rangeOfQueen(const ChessPos &pos, list_of_pos &positions)
{
  size_t        n = positions.size();

  // a queen is the unholy marriage between a rook and his bishop
  for(size_t i = 0; QueenDirs[i] != DirNone; ++i)
  {
    findPositions(pos, QueenDirs[i], EndOfBoard, positions);
  }

  return positions.size() - n;
}

size_t ChessBoard::rangeOfBishop(const ChessPos &pos, list_of_pos &positions)
{
  size_t  n = positions.size();

  for(size_t i = 0; BishopDirs[i] != DirNone; ++i)
  {
    findPositions(pos, BishopDirs[i], EndOfBoard, positions);
  }

  return positions.size() - n;
}

size_t ChessBoard::rangeOfKnight(const ChessPos &pos, list_of_pos &positions)
{
  size_t  n = positions.size();

  ChessPos  p;

  for(size_t i = 0; i < KnightMoveDeltas.size(); ++i)
  {
    const twoints &delta = KnightMoveDeltas[i];

    p.m_file = shiftFile(pos.m_file, delta.x);
    p.m_rank = shiftRank(pos.m_rank, delta.y);

    if( isOnChessBoard(p) )
    {
      positions.push_back(p);
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::rangeOfRook(const ChessPos &pos, list_of_pos &positions)
{
  size_t  n = positions.size();

  for(size_t i = 0; RookDirs[i] != DirNone; ++i)
  {
    findPositions(pos, RookDirs[i], EndOfBoard, positions);
  }

  return positions.size() - n;
}

size_t ChessBoard::rangeOfPawnSrc(const ChessColor eColor,
                                  const ChessPos   &pos,
                                  list_of_pos      &positions)
{
  size_t  n = positions.size();
  size_t  max;

  // white pawn or any pawn
  if( (eColor == White) || (eColor == NoColor) )
  {
    for(size_t i = 0; WhitePawnSrcDirs[i] != DirNone; ++i)
    {
      if( WhitePawnSrcDirs[i] == DirDown )
      {
        max = pos.m_rank == PawnRankWhite2SqDst? 2: 1;
      }
      else
      {
        max = 1;
      }

      findPositions(pos, WhitePawnSrcDirs[i], max, positions);
    }
  }

  // black pawn or any pawn
  if( (eColor == Black) || (eColor == NoColor) )
  {
    for(size_t i = 0; BlackPawnSrcDirs[i] != DirNone; ++i)
    {
      if( BlackPawnSrcDirs[i] == DirUp )
      {
        max = pos.m_rank == PawnRankBlack2SqDst? 2: 1;
      }
      else
      {
        max = 1;
      }

      findPositions(pos, BlackPawnSrcDirs[i], max, positions);
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::rangeOfPawnDst(const ChessColor eColor,
                                  const ChessPos   &pos,
                                  list_of_pos      &positions)
{
  size_t  n = positions.size();
  size_t  max;

  // white pawn or any pawn
  if( (eColor == White) || (eColor == NoColor) )
  if( eColor == White )
  {
    for(size_t i = 0; WhitePawnDirs[i] != DirNone; ++i)
    {
      if( WhitePawnDirs[i] == DirUp )
      {
        max = pos.m_rank == PawnRankWhite2SqSrc? 2: 1;
      }
      else
      {
        max = 1;
      }

      findPositions(pos, WhitePawnDirs[i], max, positions);
    }
  }

  // black pawn or any pawn
  if( (eColor == Black) || (eColor == NoColor) )
  {
    for(size_t i = 0; BlackPawnDirs[i] != DirNone; ++i)
    {
      if( BlackPawnDirs[i] == DirDown )
      {
        max = pos.m_rank == PawnRankBlack2SqSrc? 2: 1;
      }
      else
      {
        max = 1;
      }

      findPositions(pos, BlackPawnDirs[i], max, positions);
    }
  }

  return positions.size() - n;
}

size_t ChessBoard::findPositions(const ChessPos      &pos,
                                 const ChessBoardDir dir,
                                 const size_t        max,
                                 list_of_pos         &positions)
{
  // no or unknown direction
  if( (dir <= DirNone) || (dir >= DirNumOf) )
  {
    return 0;
  }

  twoints delta = DirDeltas.at(dir);  // direction defined as two deltas
  size_t  n     = 0;

  // starting candidate position
  ChessPos  p(shiftFile(pos.m_file, delta.x), shiftRank(pos.m_rank, delta.y));

  //
  // Add candidates while position is on the board and less than maximum 
  // number of positions.
  //
  while( isOnChessBoard(p) && (n < max) )
  {
    positions.push_back(p);

    p.m_file = shiftFile(p.m_file, delta.x);
    p.m_rank = shiftRank(p.m_rank, delta.y);

    ++n;
  }

  return n;
}

size_t ChessBoard::filterPositions(const ChessPos    &filter,
                                   const list_of_pos &positions,
                                   list_of_pos       &filtered)
{
  size_t  n = filtered.size();

  for(size_t i = 0; i < positions.size(); ++i)
  {
    if( ((filter.m_file == NoFile) || (filter.m_file == positions[i].m_file)) &&
        ((filter.m_rank == NoRank) || (filter.m_rank == positions[i].m_rank)) )
    {
      filtered.push_back(positions[i]);
    }
  }

  return filtered.size() - n;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Protected
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void ChessBoard::initBoard()
{
  int       row, col;
  ChessPos  pos;

  for(row = 0; row < NumOfRanks; ++row)
  {
    for(col = 0; col < NumOfFiles; ++col)
    {
      pos.m_file = toFile(col);
      pos.m_rank = toRank(row);
      m_board[row][col].setPos(pos);
    }
  }
}

void ChessBoard::setupBoard(ChessColor eColor)
{
  ChessPiece  ePiece;
  string      strId;
  ChessFile   file;
  ChessRank   rank;
  int         row, col;
  
  // -----
  // Back line
  // -----
  rank = eColor == White? ChessRank1: ChessRank8;
  row  = toRow(rank);

  //
  // Major pieces from left to right.
  //
  for(file = ChessFileA; file != NoFile; file = nextFile(file))
  {
    col = toCol(file);

    switch( file )
    {
      // rooks
      case ChessFileA:
      case ChessFileH:
        ePiece = Rook;
        break;

      // knights
      case ChessFileB:
      case ChessFileG:
        ePiece = Knight;
        break;

      // bishops
      case ChessFileC:
      case ChessFileF:
        ePiece = Bishop;
        break;

      // queen
      case ChessFileD:
        ePiece = Queen;
        break;

      // king
      case ChessFileE:
        ePiece = King;
        break;
    }

    strId = ChessFqPiece::makePieceId(file, rank, eColor, ePiece);

    m_board[row][col].setPiece(eColor, ePiece, strId);
  }

  // -----
  // Pawns
  // -----
  rank    = eColor == White? ChessRank2: ChessRank7;
  row     = toRow(rank);
  ePiece  = Pawn;

  //
  // Left to right files.
  //
  for(file = ChessFileA; file != NoFile; file = nextFile(file))
  {
    col   = toCol(file);
    strId = ChessFqPiece::makePieceId(file, rank, eColor, ePiece);

    m_board[row][col].setPiece(eColor, ePiece, strId);
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

namespace chess_engine
{
  ostream &operator<<(ostream &os, const ChessBoard &board)
  {
    if( board.m_bGraphic )
    {
      return ographic(os, board);
    }
    else
    {
      return oascii(os, board);
    }
  }
  
  ostream &ographic(ostream &os, const ChessBoard &board)
  {
    int         row, col;     // board matrix row,column
    ChessFile   file;         // chess board file
    ChessRank   rank;         // chess board rank
    ChessColor  eSquareColor; // color of square (white or black)
    ChessPiece  ePieceType;   // type of piece
    ChessColor  ePieceColor;  // color of piece (white or black)
    string      strFigurine;  // piece figurine unicode character
  
    for(row = 0; row < NumOfRanks; ++row)
    {
      rank = board.toRank(row);
  
#ifdef LARGE_BOARD
      os << "  ";
      for(col = 0; col < NumOfFiles; ++col)
      {
        file = board.toFile(col);

        eSquareColor = board.at(file, rank).getColor();

        os << (eSquareColor == White? ColorWhiteSq: ColorBlackSq);
        os << "   ";
        os << strReset;
      }
      os << endl;
#endif // LARGE_BOARD

      os << (char)rank << ' ';
  
      for(col = 0; col < NumOfFiles; ++col)
      {
        file = board.toFile(col);
  
        const ChessSquare &sq = board.at(file, rank);
  
        eSquareColor  = sq.getColor();
        ePieceType    = sq.getPieceType();
        ePieceColor   = sq.getPieceColor();
        strFigurine   = figurineOfPiece(ePieceColor, ePieceType);
  
        os << (eSquareColor == White? ColorWhiteSq: ColorBlackSq);
#ifdef LARGE_BOARD
        os << " ";
#endif // LARGE_BOARD
        os << strFigurine << " ";
        os << strReset;
      }
  
      os << endl;
    }
  
    os << "  ";
  
    for(col = 0; col < NumOfFiles; ++col)
    {
      file = board.toFile(col);
#ifdef LARGE_BOARD
      os << " ";
#endif // LARGE_BOARD
      os << (char)file << " ";
    }
  
    os << endl;
  
    return os;
  }
  
  ostream &oascii(ostream &os, const ChessBoard &board)
  {
    int         row, col;       // board matrix row,column
    ChessFile   file;           // chess board file
    ChessRank   rank;           // chess board rank
    ChessColor  eSquareColor;   // color of square (white or black)
    ChessPiece  ePieceType;     // type of piece
    ChessColor  ePieceColor;    // color of piece (white or black)
    string      strPiece;       // piece one-character name
  
    for(row = 0; row < NumOfRanks; ++row)
    {
      rank = board.toRank(row);
  
      // row border
      os << "  ";
      for(col = 0; col < NumOfFiles; ++col)
      {
        os << "- - ";
      }
      os << "-" << endl;

      os << (char)rank << ' ';

      for(col = 0; col < NumOfFiles; ++col)
      {
        file = board.toFile(col);
  
        const ChessSquare &sq = board.at(file, rank);
  
        eSquareColor  = sq.getColor();
        ePieceType    = sq.getPieceType();
        ePieceColor   = sq.getPieceColor();
  
        if( ePieceType == NoPiece )
        {
          strPiece    = " ";
          ePieceColor = eSquareColor;
        }
        else if( ePieceColor == White )
        {
          strPiece = (char)ePieceType;
        }
        else
        {
          strPiece = (char)tolower(ePieceType);
        }
  
        os << ":";
        os << " " << strPiece << " ";
      }
  
      os << ":";
      os << endl;
    }
  
    // bottom row border
    os << "  ";
    for(col = 0; col < NumOfFiles; ++col)
    {
      os << "- - ";
    }
    os << "-" << endl;
  
    // file labels
    os << "  ";
    for(col = 0; col < NumOfFiles; ++col)
    {
      file = board.toFile(col);
      os << "  " << (char)file << " ";
    }
    os << endl;

    return os;
  }
} // namespace chess_engine
