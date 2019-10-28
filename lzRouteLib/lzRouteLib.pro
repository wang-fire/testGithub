#-------------------------------------------------
#
# Project created by QtCreator 2018-12-05T14:24:15
#
#-------------------------------------------------

QT       += core

TARGET = lzRouteLib
TEMPLATE = lib

DEFINES += LZROUTELIB_LIBRARY

SOURCES += lzroutelib.cpp \
    RouteSearch/agvForkPath2StandardPath.cpp \
    RouteSearch/AStarSearch.cpp \
    RouteSearch/generateAgvForkPath.cpp \
    RouteSearch/geometryCalculate.cpp \
    RouteSearch/getRoute.cpp \
    RouteSearch/loadMap.cpp \
    RouteSearch/moveScript.cpp

HEADERS += lzroutelib.h\
        lzroutelib_global.h \
    RouteSearch/agvForkParam.h \
    RouteSearch/agvForkPath2StandardPath.h \
    RouteSearch/AStarSearch.h \
    RouteSearch/common.h \
    RouteSearch/generateAgvForkPath.h \
    RouteSearch/geometryCalculate.h \
    RouteSearch/getRoute.h \
    RouteSearch/loadMap.h \
    RouteSearch/moveScript.h \
    RouteSearch/RetType.h \
    RouteSearch/typeDefines.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
