/* Created  :   Linhui
 * Date     :   2016-08-05
 * Usage    :   Some data structure that is needed to parse VLP16 lidar data
*/
#ifndef VLP16DATASTRUCT_H
#define VLP16DATASTRUCT_H

#include <sys/types.h>
#include "Lidar16_Datan.h"
#include "program.h"
/* raw lidar data parser:
 *  each packet consists of 1026 bytes, and
 *  each frame is made up with some 75 packets
*/
// VLP16 data struct definition
static const int VLP16_NUM_LASERS = 32;                   // The number of lasers per shot
static const int VLP16_NUM_SHOTS = 12;                    // The number of shots per packet
static const u_int16_t VLP16_SHOT_START_FLAG = 0xffee;    // The byte indicating a shot begins
static const int VLP16_BEAM_NUM = 16;
static const int VLP16_BEAM_POINTSIZE = VLP16_PACKET_NUM*VLP16_NUM_SHOTS*2 * FRAME_FUSION_NUM; // maximum 2400
static const int VLP16_VALID_RADIUS_L = 1000/2;           // in 2 mm, 最近有效距离为1m
static const int VLP16_VALID_RADIUS_H = 55000/2;          // in 2 mm, 最远有效距离为55m
// 雷达正放时，前面是Y轴，右侧为X轴，上方为Z轴，angleH为Y顺时针从0到360度
static const unsigned int VLP16_INVALID_ANGLE_LL = 3000;
static const unsigned int VLP16_INVALID_ANGLE_LH = 15000;
static const unsigned int VLP16_INVALID_ANGLE_RL = 21000;       // 左右两侧雷达无效数据的角度相反
static const unsigned int VLP16_INVALID_ANGLE_RH = 33000;


// Velodyne datastructures
// one laser, 3 Bytes
typedef struct vlp_laser {
  u_int16_t distance;                   // 0-65536*2mm
  unsigned char intensity;              // 0-255, the greater, the brighter
} __attribute__((packed)) vlp_laser_t;

// one shot, 100 Bytes
typedef struct vlp_shot {
  u_int16_t lower_upper;
  u_int16_t rotational_angle;
  vlp_laser_t lasers[VLP16_NUM_LASERS];
} __attribute__((packed)) vlp_shot_t;

// one packet 1206 Bytes
typedef struct vlp_packet {
    vlp_shot_t shots[VLP16_NUM_SHOTS];
    u_int8_t GPS_time_stamp[4];
    u_int8_t Factory[2];
}  __attribute__((packed)) vlp_packet_t;


#endif // VLP16DATASTRUCT_H
