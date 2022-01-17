#-------------------------------------------------
#
# Project created by QtCreator 2021-05-24T11:19:04
#
#-------------------------------------------------

QT       += core gui

LIBS += -lyaml-cpp

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = parameters_vector_manager
TEMPLATE = lib
CONFIG += staticlib
UI_DIR = ui_autogen

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        parameters_vector_loader.cpp \
        parameters_vector_manager.cpp \
        parameters_vector_saving.cpp

HEADERS += \
    parameters_vector_loader.h \
        parameters_vector_manager.h \
    parameters_vector_saving.h \
        ui_autogen/ui_parameters_vector_manager.h \
        ui_autogen/ui_parameters_vector_naming.h \

FORMS += \
        ui/parameters_vector_manager.ui \
        ui/parameters_vector_naming.ui

unix {
    target.path = /usr/lib
    INSTALLS += target
}
