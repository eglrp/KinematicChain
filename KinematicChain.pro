TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    viewer.cpp \
    tracer.cpp

HEADERS += \
    frame.h \
    kinematicchain.h \
    viewer.h \
    tracer.h

#eigen3
unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += eigen3

#opencv
unix: PKGCONFIG += opencv

#pangolin
LIBS += -lpangolin -lGL -lglut
