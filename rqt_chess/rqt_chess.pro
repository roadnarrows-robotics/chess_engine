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
  src/chessboard.cpp

#  src/qnode.cpp

HEADERS += \
  include/mainwindow.h \
  include/chessboard.h

#  include/qnode.h

INCLUDEPATH += include

FORMS    += ui/mainwindow.ui

RESOURCES += resources/images.qrc

