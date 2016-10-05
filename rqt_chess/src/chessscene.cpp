////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Node:      rqt_chess
//
// File:      chessscene.cpp
//
/*! \file
 *
 * \brief Chess scene widget implementation.
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
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include <map>

#include <QGraphicsSceneMouseEvent>
#include <QGraphicsItem>

#include "rqt_chess/chessscene.h"


using namespace std;
using namespace chess_engine;

namespace rqt_chess
{
  static const int ChessSqSize    = 49; ///< chess board square size (pixels)
  static const int ChessBoarder   = 1;  ///< chess board boarder (pixels)
  static const int ChessBoardSize = ChessSqSize * NumOfFiles - 2 * ChessBoarder;
                                        ///< chess board size (pixels);
}

using namespace rqt_chess;

ChessScene::ChessScene() :
  m_pieceSrc(EmptyElem), m_pieceDst(EmptyElem)
{
  QPixmap               img;
  QGraphicsPixmapItem  *pItem;

  // chess board
  img = QPixmap(":/images/chessboard2.png");
  pItem = addPixmap(img);
  pItem->setPos(0, 0);
  pItem->setZValue(-1);

  // chess pieces
  loadPieces();
}

void ChessScene::newGame()
{
  fprintf(stderr, "newGame()\n");
  setupBoard();
}

void ChessScene::mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
}

void ChessScene::mouseMoveEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
}

void ChessScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *mouseEvent)
{
  if( mouseEvent->button() == Qt::LeftButton )
  {
    QPointF pt = mouseEvent->scenePos(); //.toPoint();

    fprintf(stderr, "mousePressEvent(): %lf,%lf --> ", pt.x(), pt.y());

    if( (pt.x() >= 0.0) && (pt.x() <= 390.0) &&
        (pt.y() >= 0.0) && (pt.y() <= 390.0) )
    {
      int row = (int)(pt.y() / (double)ChessSqSize);
      int col = (int)(pt.x() / (double)ChessSqSize);

      ChessFile file = m_game.toFile(col);
      ChessRank rank = m_game.toRank(row);
      
      fprintf(stderr, "sq=%c%c\n", (char)file, (char)rank);
    }
    else
    {
      fprintf(stderr, "no sq\n");
    }
  }
}

void ChessScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event)
{
}

void ChessScene::loadPieces()
{
  int           file, rank;
  BoardElem    *pElem;

  string        strId;
  string        strFileName;

  QPixmap               img;
  QGraphicsPixmapItem  *pItem;

  int           x, y;

  m_game.setupBoard();

  for(file = ChessFileA; file <= ChessFileH; ++file)
  {
    for(rank = ChessRank1; rank <= ChessRank8; ++rank)
    {
      pElem = m_game.getBoardElem((ChessFile)file, (ChessRank)rank);

      if( (pElem == NULL) ||
          (pElem->m_color == NoColor) ||
          (pElem->m_piece == NoPiece) )
      {
        continue;
      }

      strFileName = ":/images/" +
                      nameOfColor(pElem->m_color) +
                      nameOfPiece(pElem->m_piece) +
                      ".png";

      strId = toPieceId((ChessFile)file, (ChessRank)rank,
                            pElem->m_color, pElem->m_piece);

      img = QPixmap(strFileName.c_str());

      pItem = addPixmap(img);
      pItem->setZValue(0);

      x = m_game.toCol(file) * ChessSqSize + ChessBoarder;
      y = m_game.toRow(rank) * ChessSqSize + ChessBoarder;

      pItem->setPos(x, y);

      m_pixmapPiece[strId] = pItem;
    }
  }
}

void ChessScene::setupBoard()
{
  int           file, rank;
  string        strId;
  BoardElem    *pElem;

  QGraphicsPixmapItem  *pItem;

  int           x, y;

  m_game.setupBoard();

  for(file = ChessFileA; file <= ChessFileH; ++file)
  {
    for(rank = ChessRank1; rank <= ChessRank8; ++rank)
    {
      pElem = m_game.getBoardElem((ChessFile)file, (ChessRank)rank);

      if( (pElem == NULL) ||
          (pElem->m_color == NoColor) ||
          (pElem->m_piece == NoPiece) )
      {
        continue;
      }

      strId = toPieceId((ChessFile)file, (ChessRank)rank,
                            pElem->m_color, pElem->m_piece);

      pItem = m_pixmapPiece[strId];

      x = m_game.toCol(file) * ChessSqSize + ChessBoarder;
      y = m_game.toRow(rank) * ChessSqSize + ChessBoarder;

      pItem->setPos(x, y);
    }
  }
}

string ChessScene::toPieceId(ChessFile file,   ChessRank rank,
                             ChessColor color, ChessPiece piece)
{
  char    buf[64];

  sprintf(buf, "%c%c-%s-%s",
      file, rank, nameOfColor(color).c_str(), nameOfPiece(piece).c_str());

  return string(buf);
}
