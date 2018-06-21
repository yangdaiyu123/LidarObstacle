#ifndef LIDAR4DATASTRUCT_H
#define LIDAR4DATASTRUCT_H
#include "Lidar4_Datan.h"

static const int LIDAR4_BEAM_NUM = 4;
static const int LIDAR4_BEAM_POINTSIZE = 500;
static const int LIDAR4_MAX_POINT_SIZE = 1000;

struct ORI_DATA{  //store origine data read from txt
    int layer;
    int echo;
    double angle;  //angle but not radian
    int distance;  //cetermeter
};

struct position_data{   //store position data in x y z coordinate system
    int layer;
    double posi_x;  //x y z in cetermeter
    double posi_y;
    double posi_z;
};

struct carlibration{
    int layer;
    double x;
    double y;
    double z;
};
struct LIDAR4_DATA{
    ORI_DATA origine_point[1600];
    int length;

    position_data position_point[1600];

    carlibration carli_point[1600];

};

#endif // LIDAR4DATASTRUCT_H
