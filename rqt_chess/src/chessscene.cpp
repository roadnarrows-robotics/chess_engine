#include <QGraphicsItem>

#include "rqt_chess/chessscene.h"

ChessScene::ChessScene()
{
  //QPixmap img = QPixmap("/prj/ros/indigo/src/chess_engine.wiki/images/chessboard2.png");
  QPixmap img = QPixmap(":/images/chessboard2.png");

  //setBackgroundImage(img);

  QGraphicsPixmapItem *bgItem = new QGraphicsPixmapItem();
  
  bgItem = addPixmap(img);
  bgItem->setPos(0, 0);
  bgItem->setZValue(-1);

  QPixmap img2 = QPixmap(":/images/blackknight.png");
  QGraphicsPixmapItem *p = new QGraphicsPixmapItem();
  p = addPixmap(img2);
  p->setPos(192+5, 48+5);
  p->setZValue(0);

  QPixmap img3 = QPixmap(":/images/whitequeen.png");
  QGraphicsPixmapItem *p2 = new QGraphicsPixmapItem();
  p2 = addPixmap(img3);
  p2->setPos(48*6+5, 48*7+5);
  p2->setZValue(0);
}
