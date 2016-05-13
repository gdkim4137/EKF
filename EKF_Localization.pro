#-------------------------------------------------
#
# Project created by QtCreator 2016-03-21T03:33:09
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = EKF_Localization
TEMPLATE = app

CONFIG += c++11 \   # for using cn++11
          console   # for debugging


SOURCES += main.cpp\
        dialog.cpp \
    background_service.cpp

HEADERS  += dialog.h \
    background_service.h \
    Serial.h \
    Sensor.h \
    YR9010.h \
    Encoder.h \
    myahrs_plus.hpp \
    EKF_algorithm.h

FORMS    += dialog.ui
