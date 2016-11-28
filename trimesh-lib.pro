QT       -= gui

CONFIG += c++14

TARGET = trimesh-lib
TEMPLATE = lib

QMAKE_CXXFLAGS+= -fopenmp
QMAKE_LFLAGS +=  -fopenmp

DEFINES += TRIMESHLIB_LIBRARY

SOURCES += \
    Mesh/TriMesh.cpp

HEADERS +=\
    Mesh/TriMesh.h \
    TriMeshDefs.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

LIBS += -lOpenMeshCore -lOpenMeshTools
