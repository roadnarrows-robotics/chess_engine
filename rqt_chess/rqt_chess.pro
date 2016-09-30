#-------------------------------------------------
#
# Project created by QtCreator 2016-09-23T13:31:57
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = rqt_chess
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    chessboard.cpp

HEADERS  += mainwindow.h \
    chessboard.h

FORMS    += mainwindow.ui

RESOURCES += resources/images.qrc

