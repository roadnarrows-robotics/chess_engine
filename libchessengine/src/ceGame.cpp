////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Library:   libchessengine
//
// File:      ceGame.cpp
//
/*! \file
 *
 * \brief The chess game state implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2016  RoadNarrows
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

#include "rnr/color.h"

#include "chess_engine/ceChess.h"
#include "chess_engine/ceMove.h"
#include "chess_engine/ceGame.h"

using namespace std;
using namespace chess_engine;


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Chess board square class.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ChessSquare::ChessSquare()
{
  m_pos         = NoPos;
  m_eColor      = colorOfSquare(m_pos);
  m_ePieceColor = NoColor;
  m_ePieceType  = NoPiece;
}

void ChessSquare::ChessSquare(const ChessPos    &pos,
                              const ChessColor  ePieceColor,
                              const ChessPiece  ePieceType,
                              const std::string &strPieceId)
{
  m_pos         = pos;
  m_eColor      = colorOfSquare(pos);
  m_ePieceColor = ePieceColor;
  m_ePieceType  = ePieceType;
  m_strPieceId  = strPiecedId;
}

ChessSquare ChessSquare::operator==(const ChessSquare &rhs)
{
  m_pos         = rhs.m_pos;
  m_eColor      = colorOfSquare(m_pos);
  m_ePieceColor = rhs.m_ePieceColor;
  m_ePieceType  = rhs.m_ePieceType;
  m_strPieceId  = rhs.m_strPiecedId;

  return *this;
}

void ChessSquare::setPos(const ChessPos &pos)
{
  m_pos = pos;

  m_eColor = colorOfSquare(m_pos);
}

ChessPos ChessSquare::getPos();
{
  return m_pos;
}

ChessColor ChessSquare::getColor();
{
  return m_eColor;
}

void ChessSquare::setPiece(const ChessColor   ePieceColor,
                           const ChessPiece   ePieceType,
                           const std::string &strPieceId)
{
  m_ePieceColor = ePieceColor;
  m_ePieceType  = ePieceType;
  m_strPieceId  = strPiecedId;
}

void ChessSquare::getPiece(ChessColor  &ePieceColor,
                           ChessPiece  &ePieceType,
                           std::string &strPieceId)
{
  eColor      = m_ePieceColor;
  eType       = m_ePieceType;
  strPieceId  = m_strPiecedId;
}

void ChessSquare::copyPiece(ChessSquare &dst)
{
  dst.m_ePieceColor = m_ePieceColor;
  dst.m_ePieceType  = m_ePieceType;
  dst.m_strPieceId  = m_strPiecedId;
}

void ChessSquare::movePiece(ChessSquare &dst)
{
  copyPiece(dst);
  removePiece();
}

void ChessSquare::removePiece()
{
  m_ePieceColor = NoColor;
  m_ePieceType  = NoPiece;
  m_strPieceId.clear();
}

bool ChessSquare::isEmpty()
{
  return m_ePieceType == NoPiece;
}

bool ChessSquare::isNotOnBoard()
{
  return (m_pos.m_file == NoFile) || (m_pos.m_rank == NoRank);
}

ChessColor ChessSquare::getPieceColor()
{
  return m_ePieceColor;
}

ChessPiece ChessSquare::getPieceType()
{
  return m_ePieceType;
}

string ChessSquare::getPieceId()
{
  return m_strPieceId;
}

ChessColor ChessSquare::colorOfSquare(const ChessPos &pos)
{
  return ChessSquare::colorOfSquare(pos.m_file, pos.m_rank);
}

ChessColor ChessSquare::colorOfSquare(int file, int rank)
{
  if( (file == NoFile) || (rank == NoRank) )
  {
    return NoColor;
  }
  else
  {
    return ((file - ChessFileA) + (rank - ChessRank1)) % 2 == 0? Black: White;
  }
}


static ChessNoSquare = ChessSquare();   ///< "no square" square


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Chess game class.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

Game::Game()
{
  initBoard();

  m_bGui = false;
}

void Game::setupGame()
{
  setupBoard();

  // RDK clear history, boneyards. Reset state
}

int Game::qualify(Move &move)
{
  int   rc;

  //
  // Game state sanity checks.
  //
  if( !m_bIsPlaying )
  {
    move.m_result = NoGame;
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Cursory check of move's current result
  //
  switch( move.m_result )
  {
    case BadMove:
      return -CE_ECODE_CHESS_MOVE;
    case OutOfTurn:
      return -CE_ECODE_CHESS_OUT_OF_TURN;
    case NoGame:
      m_bIsPlaying  = false;
      m_endReason   = move.m_result;
      return -CE_ECODE_CHESS_SYNC;
    case GameFatal:
      m_bIsPlaying  = false;
      m_endReason   = move.m_result;
      return -CE_ECODE_CHESS_FATAL;
    case Checkmate:
    case Draw:
    case Resign:
      m_bIsPlaying  = false;
      m_endReason   = move.m_result;
      m_winner      = move.m_winner;
      // there is still a move to process
      break;
    case Ok:
    case NoResult:
    default:
      break;
  }

  ChessSquare &sqSrc = getBoardSquare(move.m_posFrom);
  ChessSquare &sqDst = getBoardSquare(move.m_posTo);

  //
  // Source square is not on the board.
  //
  if( sqSrc.isNotOnBoard() )
  {
    printf("ROSLOG: Error: bad 'from' square = %c%c.\n",
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_FATAL;
  }
  
  //
  // Destination square is not on the board.
  //
  else if( sqDst.isNotOnBoard() )
  {
    printf("ROSLOG: Error: bad 'to' square = %c%c.\n",
        move.m_posTo.m_file, move.m_posTo.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_FATAL;
  }
  
  //
  // Set moved piece, if unset
  //
  if( move.m_piece == NoPiece )
  {
    move.m_piece = sqSrc.getPieceType();
  }

  //
  // Cannot move a non-existent piece.
  //
  if( move.m_piece == NoPiece )
  {
    printf("ROSLOG: Error: no piece found on 'from' square %c%c.\n",
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Disagreement about what piece to move - game probably out of sync.
  //
  else if( move.m_piece != sqSrc.getPieceType() )
  {
    printf("ROSLOG: Error: piece %c != %c found on 'from' square %c%c.\n",
        move.m_piece, sqSrc.getPieceType(),
        move.m_posFrom.m_file, move.m_posFrom.m_rank);
    m_bIsPlaying  = false;
    move.m_result = GameFatal;
    return -CE_ECODE_CHESS_SYNC;
  }

  //
  // Capture
  //
  if( sqDst.getPieceType() != NoPiece )
  {
    rc = qualifyCapture(move);
  }

  //
  // En Passant (pawn's destination must be empty)
  //
  else if( move.m_piece == Pawn )
  {
    rc = qualifyEnPassant(move);
  }

  //
  // Castling (king's destination must be empty)
  //
  else if( move.m_castle != NoCastle )
  {
    rc = qualifyCastling(move);
  }

  //
  // "Normal" move.
  //
  else
  {
    rc = CE_OK;
  }

  return rc;
}

int Game::qualifyCapture(Move &move)
{
  move.m_captured = getBoardSquare(move.m_posTo).getPieceType();
  return CE_OK;
}

int Game::qualifyEnPassant(Move &move)
{
  move.m_en_passant = false;

  if( move.m_piece != Pawn )
  {
    return CE_OK;
  }

  ChessSquare &sqDst = getBoardSquare(move.m_posTo);

  switch( move.m_player )
  {
    case White:
      if( (move.m_posFrom.m_rank == ChessRank5) &&
          (move.m_posTo.m_rank   == ChessRank6) &&
          (move.m_posFrom.m_file != move.m_posTo.m_file) &&
          (sqDst.getPieceType() == NoPiece) )
      {
        move.m_en_passant = true;
      }
      break;
    case Black:
      if( (move.m_posFrom.m_rank == ChessRank4) &&
          (move.m_posTo.m_rank   == ChessRank3) &&
          (move.m_posFrom.m_file != move.m_posTo.m_file) &&
          (sqDst.getPieceType() == NoPiece) )
      {
        move.m_en_passant = true;
      }
      break;
    default:
      return -CE_ECODE_CHESS_MOVE;
  }

  if( !move.m_en_passant )
  {
    return CE_OK;
  }

  move.m_posAuxFrom.m_file = move.m_posTo.m_file;
  move.m_posAuxFrom.m_rank = move.m_posFrom.m_rank;

  ChessSquare &sqAux = getBoardSquare(move.m_posAuxFrom);

  //
  // Indeed, en passant.
  //
  if( sqAux.getPieceType() == Pawn )
  {
    move.m_captured = Pawn;
  }

  //
  // Something is amiss.
  //
  else
  {
    printf("ROSLOG:\n");
    move.m_en_passant = false;
    move.m_posAuxFrom = NoPos;
    return -CE_ECODE_CHESS_MOVE;
  }

  return CE_OK;
}

int Game::qualifyCastling(Move &move)
{
  ChessRank rank = move.m_player == White? ChessRank1: ChessRank8;

  if( move.m_piece != King )
  {
    move.m_castle = NoCastle;
  }
  else if( (move.m_posFrom.m_rank != rank) || (move.m_posTo.m_rank != file) )
  {
    move.m_castle = NoCastle;
  }
  else if( move.m_posFrom.m_file != ChessFileE )
  {
    move.m_castle = NoCastle;
  }
  else if( move.m_posTo.m_file == ChessFileG )
  {
    move.m_castle = KingSide;
  }
  else if( move.m_posTo.m_file == ChessFileC )
  {
    move.m_castle = QueenSide;
  }
  else
  {
    move.m_castle = NoCastle;
  }
 
  switch( move.m_castle )
  {
    // king side rook
    case KingSide:
      move.m_posAuxFrom.m_file  = ChessFileH;
      move.m_posAuxFrom.m_rank  = rank
      move.m_posAuxTo.m_file    = ChessFileF
      move.m_posAuxTo.m_rank    = rank
      break;
    // queen side rook
    case QueenSide:
      move.m_posAuxFrom.m_file  = ChessFileA;
      move.m_posAuxFrom.m_rank  = rank
      move.m_posAuxTo.m_file    = ChessFileD
      move.m_posAuxTo.m_rank    = rank
      break;
    default:
      return CE_OK;
  }

  ChessSquare &sqAux = getBoardSquare(move.m_posAuxFrom);

  if( sqAux.getPieceType() != ROOK )
  {
    printf("ROSLOG:\n");
    return -CE_ECODE_CHESS_MOVE;
  }

  return CE_OK;
}

void Game::execMove(Move &move)
{
  boneyard
  movePiece(pSrc, pDst);

  recordHistory(move);

}

void Game::stopPlaying(ChessResult reason, ChessColor winner)
{
  m_bIsPlaying  = false;
  m_endReason   = reason;
  m_winner      = winner;
}

int Game::getNumOfMoves()
{
  return ((int)m_history.size() + 1) / 2;
}

int Game::getNumOfPlies()
{
  return (int)m_history.size();
}

ChessSquare &Game::getBoardSquare(const int file, const int rank)
{
  ChessPos  pos;

  pos.m_file = (ChessFile)file;
  pos.m_rank = (ChessRank)rank;

  return getBoardSquare(pos);
}

ChessSquare &Game::getBoardSquare(const ChessPos &pos)
{
  if( toRowCol(pos, row, col) == CE_OK )
  {
    return m_board[row][col];
  }
  else
  {
    return ChessNoSquare;
  }
}

std::vector<ChessPiece> &Game::getBoneYard(ChessColor color)
{
  return color == White? m_boneYardWhite: m_boneYardBlack;
}

int Game::toRow(int rank)
{
  return NumOfRanks - (rank - (int)ChessRank1) - 1;
}

int Game::toCol(int file)
{
  return file - (int)ChessFileA;
}

int Game::toRowCol(const ChessPos &pos, int &row, int &col)
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

ChessFile Game::toFile(int col)
{
  return (ChessFile)((int)ChessFileA + col);
}

ChessRank Game::toRank(int row)
{
  return (ChessRank)((int)ChessRank1 + NumOfRanks - row - 1);
}

ChessFile Game::nextFile(int file)
{
  int col = toCol(file) + 1;

  return col < NumOfFiles: toFile(col): NoFile;
}

ChessRank Game::nextRank(int rank)
{
  int row = toRow(rank) + 1;

  return row < NumOfRanks: toRank(row): NoRank;
}

ChessColor Game::getSquareColor(int file, int rank)
{
  return ChessSquare::colorOfSquare(file, rank);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Protected
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void Game::initBoard()
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

  setupBoard();
}

void Game::clearBoard()
{
  for(int col=0; col<NumOfFiles; ++col)
  {
    for(int row=0; row<NumOfRanks; ++row)
    {
      m_board[row][col].removePiece();
    }
  }
}

void Game::setupBoard()
{
  clearBoard();

  setupBoard(White);
  setupBoard(Black);
}

void Game::setupBoard(ChessColor eColor)
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
  // Pieces from left to right.
  //
  for(file = ChessFileA; file = nextFile(file); file != NoFile)
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

    strId = makePieceId(file, rank, eColor, ePiece);

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
  for(file = ChessFileA; file = nextFile(file); file != NoFile)
  {
    col   = toCol(file);
    strId = makePieceId(file, rank, eColor, ePiece);

    m_board[row][col].set(eColor, ePiece, strId);
  }
}

string Game::makePieceId(int file, int rank,
                         ChessColor eColor, ChessPiece ePiece)
{
  string  strId;
  string  strMod;
  string  strSep("-");

  // pawns
  if( (rank == ChessRank2) && (file != ChessRank7) )
  {
    strMod.assign(1, (char)file);
  }

  // the muscle pieces
  else if( (rank == ChessRank1) && (file != ChessRank8) )
  {
    if( file <= ChessFileC )
    {
      strMod = nameOfPiece(Queen);
    }
    else if( file >= ChessFileF )
    {
      strMod = nameOfPiece(King);
    }
  }

  // empty id
  else
  {
    return strId;
  }

  // color-piece
  if( strMod.empty() )
  {
    strId = nameOfColor(eColor) + strSep + nameOfPiece(ePiece);
  }
  // color-modifier-piece
  else
  {
    strId = nameOfColor(eColor) + strSep + strMod + 
                  strSep + nameOfPiece(ePiece);
  }

  return strId;
}






BoardElem *Game::elem(const ChessPos &pos)
{
  int row, col;

  if( toRowCol(pos, row, col) == CE_OK )
  {
    return &m_board[row][col];
  }
  return NULL;
}

void Game::movePiece(const ChessPos &posFrom, const ChessPos &posTo)
{
  movePiece(elem(posFrom), elem(posTo));
}

void Game::movePiece(BoardElem *pSrc, BoardElem *pDst)
{
  *pDst = *pSrc;
  *pSrc = EmptyElem;
}

void Game::recordHistory(Move &move)
{
  m_history.push_back(move);
}

void Game::moveToBoneYard(BoardElem *pDeadPiece)
{
  if( pDeadPiece->m_piece != NoPiece )
  {
    if( pDeadPiece->m_color == White )
    {
      m_boneYardWhite.push_back(pDeadPiece->m_piece);
    }
    else if( pDeadPiece->m_color == Black )
    {
      m_boneYardBlack.push_back(pDeadPiece->m_piece);
    }
  }

  *pDeadPiece = EmptyElem;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
//  Friends
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &chess_engine::operator<<(ostream &os, const Game &game)
{
  int         file, rank;       // chess board file,rank
  int         row, col;         // board matrix row,column
  ChessPiece  piece;            // chess piece
  string      strPiece;         // piece one-character name
  ChessColor  colorPiece;       // color of piece (white or black)
  ChessColor  colorSquare;      // color of square (white or black)
  streamsize  w = os.width();   // save default width
  string      strWW;            // white piece on white square
  string      strBW;            // black piece on white square
  string      strWB;            // white piece on black square
  string      strBB;            // black piece on black square
  string      strReset;         // reset colors

  if( game.m_bGui )
  {
    // RDK: fix color.h
    strWW = ANSI_COLOR_PRE "0;30;47m";
    strBW = ANSI_COLOR_PRE "0;30;47m";
    strWB = ANSI_COLOR_PRE "0;30;46m";
    strBB = ANSI_COLOR_PRE "0;30;46m";

    strReset = ANSI_COLOR_RESET;
  }

  for(rank = ChessRank8; rank >= ChessRank1; --rank)
  {
    if( !game.m_bGui )
    {
      os.width(NumOfFiles * 4 + 1);
      os.fill('.');
      os << '.';
      os.width(w);
      os << endl;
    }

    for(file = ChessFileA; file <= ChessFileH; ++file)
    {
      row = game.toRow(rank);
      col = game.toCol(file);

      colorSquare = game.getSquareColor(file, rank);
      piece       = game.m_board[row][col].m_piece;
      colorPiece  = game.m_board[row][col].m_color;

      if( game.m_bGui )
      {
        if( piece == NoPiece )
        {
          //strPiece = "\U00002588";
          strPiece = " ";
          colorPiece  = colorSquare;
        }
        else
        {
          strPiece = figurineOfPiece(colorPiece, piece);
        }
      }
      else
      {
        if( piece == NoPiece )
        {
          strPiece    = " ";
          colorPiece  = colorSquare;
        }
        else if( colorPiece == White )
        {
          strPiece = (char)piece;
        }
        else
        {
          strPiece = (char)tolower(piece);
        }
      }

      if( game.m_bGui )
      {
        if( colorSquare == White )
        {
          if( colorPiece == White )
          {
            os << strWW;
          }
          else
          {
            os << strBW;
          }
        }
        else
        {
          if( colorPiece == White )
          {
            os << strWB;
          }
          else
          {
            os << strBB;
          }
        }

        os << strPiece << " ";
        os << strReset;
      }
      else
      {
        os << "|";
        os << " " << strPiece << " ";
      }
    }

    if( !game.m_bGui )
    {
      os << "|";
    }

    os << endl;
  }

  if( !game.m_bGui )
  {
    os.width(NumOfFiles * 4 + 1);
    os.fill('-');
    os << '-' << endl;
    os.width(w);
  }

  os << endl;

  return os;
}
