#-------------------------------------------------
#
# Project created by QtCreator 2021-05-25T11:20:54
#
#-------------------------------------------------

QT       += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = dof_selection_window
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
        dof_selection_window.cpp \
        dof_controller.cpp \
        basic_motion.cpp



HEADERS += \
        dof_selection_window.h \
        dof_controller.h \
        basic_motion.h \
        ui_dof_selection_window.h


FORMS += \
    dof_selection_window.ui
unix {
    target.path = /usr/lib
    INSTALLS += target
}


