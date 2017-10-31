#-------------------------------------------------
#
# Project created by QtCreator 2016-09-23T13:31:57
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = rqt_chess
TEMPLATE = app

DEFINES += QT_DEPRECATED_WARNINGS

SOURCES += \
  src/main.cpp \
  src/rqtchess.cpp \
  src/chessscene.cpp

#  src/qnode.cpp

HEADERS += \
  include/rqt_chess/rqtchess.h \
  include/rqt_chess/chessscene.h

#  include/qnode.h

FORMS += ui/rqtchess.ui

RESOURCES += resources/images.qrc

INCLUDEPATH += include ../include /opt/ros/kinetic/include

LIBS += \
  -L/prj/ros/kinetic/devel/lib \
  -L../../../devel/lib \
  -lchessengine
  -lroscpp
  -lroslib
  -lrosconsole
  -lboost_regex
  -lboost_system
