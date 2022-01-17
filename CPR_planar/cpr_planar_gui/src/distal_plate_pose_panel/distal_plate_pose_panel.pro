#-------------------------------------------------
#
# Project created by QtCreator 2021-05-24T22:41:39
#
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = distal_plate_pose_panel
TEMPLATE = lib
CONFIG += staticlib

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        distal_plate_pose_panel.cpp

HEADERS += \
        distal_plate_pose_panel.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../controllers/release/ -lcontrollers
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../controllers/debug/ -lcontrollers
else:unix: LIBS += -L$$OUT_PWD/../controllers/ -lcontrollers

INCLUDEPATH += $$PWD/../controllers
DEPENDPATH += $$PWD/../controllers

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../controllers/release/libcontrollers.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../controllers/debug/libcontrollers.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../controllers/release/controllers.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../controllers/debug/controllers.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../controllers/libcontrollers.a
