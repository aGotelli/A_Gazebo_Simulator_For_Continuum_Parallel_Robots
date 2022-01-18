#-------------------------------------------------
#
# Project created by QtCreator 2021-05-22T12:54:20
#
#-------------------------------------------------

QT       += core gui



greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = main_window
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++14

SOURCES += \
        main.cpp \
        main_window.cpp \
        menubar.cpp

HEADERS += \
        main_window.h \
        menubar.h

FORMS += \
        main_window.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target




win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../distal_plate_pose_panel/release/ -ldistal_plate_pose_panel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../distal_plate_pose_panel/debug/ -ldistal_plate_pose_panel
else:unix: LIBS += -L$$OUT_PWD/../distal_plate_pose_panel/ -ldistal_plate_pose_panel

INCLUDEPATH += $$PWD/../distal_plate_pose_panel
DEPENDPATH += $$PWD/../distal_plate_pose_panel

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_pose_panel/release/libdistal_plate_pose_panel.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_pose_panel/debug/libdistal_plate_pose_panel.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_pose_panel/release/distal_plate_pose_panel.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_pose_panel/debug/distal_plate_pose_panel.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_pose_panel/libdistal_plate_pose_panel.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../distal_plate_wrench_panel/release/ -ldistal_plate_wrench_panel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../distal_plate_wrench_panel/debug/ -ldistal_plate_wrench_panel
else:unix: LIBS += -L$$OUT_PWD/../distal_plate_wrench_panel/ -ldistal_plate_wrench_panel

INCLUDEPATH += $$PWD/../distal_plate_wrench_panel
DEPENDPATH += $$PWD/../distal_plate_wrench_panel

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_wrench_panel/release/libdistal_plate_wrench_panel.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_wrench_panel/debug/libdistal_plate_wrench_panel.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_wrench_panel/release/distal_plate_wrench_panel.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_wrench_panel/debug/distal_plate_wrench_panel.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../distal_plate_wrench_panel/libdistal_plate_wrench_panel.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../joints_control_panel/release/ -ljoints_control_panel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../joints_control_panel/debug/ -ljoints_control_panel
else:unix: LIBS += -L$$OUT_PWD/../joints_control_panel/ -ljoints_control_panel

INCLUDEPATH += $$PWD/../joints_control_panel
DEPENDPATH += $$PWD/../joints_control_panel

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../joints_control_panel/release/libjoints_control_panel.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../joints_control_panel/debug/libjoints_control_panel.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../joints_control_panel/release/joints_control_panel.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../joints_control_panel/debug/joints_control_panel.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../joints_control_panel/libjoints_control_panel.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../solver_options_panel/release/ -lsolver_options_panel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../solver_options_panel/debug/ -lsolver_options_panel
else:unix: LIBS += -L$$OUT_PWD/../solver_options_panel/ -lsolver_options_panel

INCLUDEPATH += $$PWD/../solver_options_panel
DEPENDPATH += $$PWD/../solver_options_panel

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../solver_options_panel/release/libsolver_options_panel.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../solver_options_panel/debug/libsolver_options_panel.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../solver_options_panel/release/solver_options_panel.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../solver_options_panel/debug/solver_options_panel.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../solver_options_panel/libsolver_options_panel.a

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

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../dof_selection_window/release/ -ldof_selection_window
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../dof_selection_window/debug/ -ldof_selection_window
else:unix: LIBS += -L$$OUT_PWD/../dof_selection_window/ -ldof_selection_window

INCLUDEPATH += $$PWD/../dof_selection_window
DEPENDPATH += $$PWD/../dof_selection_window

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../dof_selection_window/release/libdof_selection_window.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../dof_selection_window/debug/libdof_selection_window.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../dof_selection_window/release/dof_selection_window.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../dof_selection_window/debug/dof_selection_window.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../dof_selection_window/libdof_selection_window.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../parameters_vector_manager/release/ -lparameters_vector_manager
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../parameters_vector_manager/debug/ -lparameters_vector_manager
else:unix: LIBS += -L$$OUT_PWD/../parameters_vector_manager/ -lparameters_vector_manager

INCLUDEPATH += $$PWD/../parameters_vector_manager
DEPENDPATH += $$PWD/../parameters_vector_manager

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../parameters_vector_manager/release/libparameters_vector_manager.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../parameters_vector_manager/debug/libparameters_vector_manager.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../parameters_vector_manager/release/parameters_vector_manager.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$OUT_PWD/../parameters_vector_manager/debug/parameters_vector_manager.lib
else:unix: PRE_TARGETDEPS += $$OUT_PWD/../parameters_vector_manager/libparameters_vector_manager.a



LIBS += -lyaml-cpp




