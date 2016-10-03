////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics ROS Chess Engine Package
//
// Link:      https://github.com/roadnarrows-robotics/chess_engine
//
// Node:      rqt_chess
//
// File:      chessscene.h
//
/*! \file
 *
 * \brief Chess scene widget interface.
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

#ifndef _CHESS_SCENE_H
#define _CHESS_SCENE_H

#include <map>

#include <QObject>
#include <QGraphicsScene>

#include "chess_engine/ceChess.h"
#include "chess_engine/ceGame.h"

typedef std::map<std::string, QGraphicsPixmapItem*> PiecePixMap;

class ChessScene : public QGraphicsScene
{
  Q_OBJECT

public:
  explicit ChessScene();

  void setupGame();

signals:

public slots:

protected:
  chess_engine::Game    m_game;
  PiecePixMap           m_pixmapPiece;

  void loadPieces();

  static std::string toPieceId(chess_engine::ChessFile file,
                               chess_engine::ChessRank rank,
                               chess_engine::ChessColor color,
                               chess_engine::ChessPiece piece);
};

#endif // _CHESS_SCENE_H
