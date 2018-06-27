TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

TARGET = ObstacleDetectorLoopmain
#TARGET = data_parse.so
DESTDIR = /home/wzq/QtProjects/Obstacle/bin/
OBJECTS_DIR = ./

SOURCES += \
    framework/alv_data.cpp \
    kernel/detect_positive_obstacle.cpp \
    kernel/detect_negative_obstacle.cpp \
    framework/ObstacleDetector.cpp \
    framework/ObstacleDetectorLoopmain.cpp

INCLUDEPATH += \
    ../../../message \
    include \
    /usr/include\
    /usr/local/include\

LIBS += \
    -L/usr/local/lib \
    -lrcs \
    ../../../lib/libMSG.a \
    #-L/home/wzq/QtProjects/Obstacle/lib/libMSG.a  \
    -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_ml \

HEADERS += \
    include/alv_data.h \
    include/program.h \
    include/Lidar16DataStruct.h \
    include/Lidar32DataStruct.h \
    include/detect_positive_obstacle.h \
    include/detect_negative_obstacle.h \
    include/ObstacleDetector.h \
    include/lidar4Datastruct.h
