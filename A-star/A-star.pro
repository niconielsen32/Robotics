TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        astar.cpp \
        main.cpp \
        pathplanning.cpp \
        travel.cpp


CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv4

unix: PKGCONFIG += fuzzylite

HEADERS += \
    astar.h \
    globalFunctions.h \
    pathplanning.h \
    tests.h \
    travel.h
