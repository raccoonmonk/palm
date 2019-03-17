TEMPLATE = lib
TARGET = pal
CONFIG += c++11
CONFIG += shared
CONFIG -= qt
LIBS += -lgeos_c
include(pal11.pri)

HEADERS += \
    src/costcalculator.h \
    src/feature.h \
    src/geomfunction.h \
    src/hashtable.hpp \
    src/internalexception.h \
    src/labelposition.h \
    src/layer.h \
    src/linkedlist.hpp \
    src/pal.h \
    src/palexception.h \
    src/palgeometry.h \
    src/palstat.h \
    src/pointset.h \
    src/priorityqueue.h \
    src/problem.h \
    src/rtree.hpp \
    src/util.h \
    src/feature_id.h

SOURCES += \
    src/costcalculator.cpp \
    src/feature.cpp \
    src/geomfunction.cpp \
    src/labelposition.cpp \
    src/layer.cpp \
    src/pal.cpp \
    src/palstat.cpp \
    src/pointset.cpp \
    src/priorityqueue.cpp \
    src/problem.cpp \
    src/util.cpp
