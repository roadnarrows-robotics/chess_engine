#-------------------------------------------------
#
# Project created by QtCreator 2016-09-23T13:31:57
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = rqt_chess
TEMPLATE = app

SOURCES += \
  src/main.cpp \
  src/mainwindow.cpp \
  src/chessscene.cpp

#  src/qnode.cpp

HEADERS += \
  include/rqt_chess/mainwindow.h \
  include/rqt_chess/chessscene.h

#  include/qnode.h

INCLUDEPATH += include ../include

FORMS    += ui/mainwindow.ui

RESOURCES += resources/images.qrc

