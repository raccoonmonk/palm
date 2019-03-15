TEMPLATE = lib
TARGET = pal
CONFIG += c++11
CONFIG += shared
CONFIG -= qt
LIBS += -lgeos_c

HEADERS += \
    src/pal/costcalculator.h \
    src/pal/feature.h \
    src/pal/geomfunction.h \
    src/pal/hashtable.hpp \
    src/pal/internalexception.h \
    src/pal/labelposition.h \
    src/pal/layer.h \
    src/pal/linkedlist.hpp \
    src/pal/pal.h \
    src/pal/palexception.h \
    src/pal/palgeometry.h \
    src/pal/palstat.h \
    src/pal/pointset.h \
    src/pal/priorityqueue.h \
    src/pal/problem.h \
    src/pal/rtree.hpp \
    src/pal/util.h \
    src/pal/feature_id.h

SOURCES += \
    src/pal/costcalculator.cpp \
    src/pal/feature.cpp \
    src/pal/geomfunction.cpp \
    src/pal/labelposition.cpp \
    src/pal/layer.cpp \
    src/pal/pal.cpp \
    src/pal/palstat.cpp \
    src/pal/pointset.cpp \
    src/pal/priorityqueue.cpp \
    src/pal/problem.cpp \
    src/pal/util.cpp
