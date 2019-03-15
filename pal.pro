
TEMPLATE = lib
TARGET = pal
unix:CONFIG += c++11
win32:CONFIG += -std=gnu++0x
CONFIG += dll
LIBS += -lgeos_c
!android {
LIBS += -lpthread
}

DEFINES +=_HAVE_PTHREAD_
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
    src/pal/simplemutex.h \
    src/pal/util.h

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
