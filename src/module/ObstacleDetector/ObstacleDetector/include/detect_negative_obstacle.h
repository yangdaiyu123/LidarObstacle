/* Created  :   Linhui
 * Date     :   2016-08-05
 * Usage    :   Declaration of negative detector
*/

#ifndef DETECT_NEGATIVE_OBSTACLE_H
#define DETECT_NEGATIVE_OBSTACLE_H

#include "alv_data.h"
#include <vector>
#include <list>
#include <mutex>

static const int FLAT_THS = 50;
static const int LESS_FLAT_THS = 150;
static const int SHARP_THS = 350;
static const int LESS_SHARP_THS = 250;
static const int WIN_SIZE = 5;                      // big window size
static const int MIN_WIN_SIZE = 2;                  // small window size
static const float NEGATIVE_GAP_MAX_SIZE = 4000;    // 最大凹坑宽度，in mm, 超过此大小不会检测
static const float NEGATIVE_GAP_MIN_SIZE = 400;     // 最小凹坑宽度，in mm, 超过此大小不会检测
//static const float NEGATIVE_GAP_MIN_SIZE_MOAT = 200;  // 壕沟比较小，因此特别对壕沟设定一个凹障碍宽度下限值
static const float NEGATIVE_NEIGHBOR_MIN_DIFF = 400;    // 负障碍后沿点的neighbor距离应该远大于附近几个点，此为该值的下限
static const float NEGATIVE_NEIGHBOR_MIN_DIS = 200;     // 负障碍后沿点的后续一个点的neighbor_dis通常很小，此为其上限
static const int NEGDET_MAX_RANGE = 1500;           // 最远检测负障碍范围, in cm
static const int NEGDET_MIN_RANGE = 350;            // min检测负障碍范围, in cm
static std::mutex m_lock;   // 线程锁，防止多个线程同时访问 NEGATIVE_DETECTOR::p_instance

struct NEGATIVE_PAIR        // point pair
{
    NEGATIVE_PAIR(alv_Point3f p1, alv_Point3f p2, float x, float y, int idx);
    alv_Point3f front;
    alv_Point3f back;
    float center_x;
    float center_y;
    int avg_distance;
    int xy_length;
    int id;

    // 自定义数据结构使用set需要定义 < 重载，因为set是用红黑树实现的
    bool operator < (const NEGATIVE_PAIR& other) const
    {
        if(avg_distance != other.avg_distance)
            return avg_distance < other.avg_distance;
        else
            return  id < other.id;
    }
};

struct NEGATIVE_CLUSTER
{
    NEGATIVE_CLUSTER();
    float variance();  // 计算x~y方差
    void length_variance();    // 计算聚类里线段长度的方差
    void cal_average();
    void pick_out(const NEGATIVE_PAIR& val);
    void put_in(const NEGATIVE_PAIR& val);

    NEGATIVE_CLUSTER& operator = (const NEGATIVE_CLUSTER& other)
    {
        data.clear();
        data.insert(other.data.begin(), other.data.end());
        avg_x = other.avg_x;
        avg_y = other.avg_y;
        return *this;
    }

    float avg_x;    // average x axis of negative pairs
    float avg_y;    // average y axis of negative pairs
    float xy_length_variance; // 线段xy长度之方差
    std::set<NEGATIVE_PAIR> data; // a same group pairs
};

// meanshift basic data element
struct MSData
{
    float data[3];
};

struct MSCenter
{
    MSCenter(int x_val, int y_val, int length_val, int i):x(x_val), y(y_val), length(length_val), ind(i){}
    bool operator == (const MSCenter &other)
    {
        return (x == other.x) && (y == other.y) && (length == other.length);
    }

    int x;
    int y;
    int length;
    int ind;
};

//struct MSCenter
//{
//    MSCenter(int x_val, int y_val, int length_val):x(x_val), y(y_val), length(length_val){}
//    // 自定义数据结构使用set需要定义 < 重载，因为set是用红黑树实现的
//    bool operator < (const MSCenter& other) const
//    {
//        if(x != other.x)
//            return x < other.x;
//        else if(y != other.y)
//            return y < other.y;
//        else
//            return length < other.length;
//    }

//    int x;
//    int y;
//    int length;
//};

// mean shift processer
class MEANSHIFT
{
public:
    std::vector<MSData> dataset;    // data set

    MEANSHIFT(double kernel_bw = 4.0, double epsilon = 0.00001) :
        kernel_bandwidth(kernel_bw),
        EPSILON(epsilon){};
    void SetInputData(const std::vector<MSData> &input);    // setup input dataset
    std::vector<MSData> applyMeanShift();
private:
    double kernel_bandwidth;
    double EPSILON;

    MSData shiftCenter(MSData prv_center); // caculate the shifted center
    double gaussian_kernel(double distance);
    double euclidean_distance(const MSData &data1, const MSData &data2);
};


/* 负障碍检测类，使用了单例模式（singleton），内含一个静态指针，通过get_instance()获得唯一的实例；
 * 关于析构：类内含有一个GarbageRobot类的对象，其析构函数专门用于delete p_instance；该对象在程序结束
 * 时会被系统释放，从而调用其析构函数，从而delete p_instance.
*/
class NEGATIVE_DETECTOR
{
public:

    static NEGATIVE_DETECTOR *p_instance;
    static NEGATIVE_DETECTOR *get_instance();   // singleton mode
    void find_negative_pairs(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE);
    void detect(ALV_DATA *alv_data);
    void reset();
    void clustering(int depth_ths, float delta_val);  // 聚类
    void clustering_MeshShift();    // MeanShift 聚类
    void recursive(NEGATIVE_CLUSTER cluster);
    void show_cluster_result(ALV_DATA *alv_data);
    void remove_outlier(const ALV_DATA *alv_data, const unsigned int min_pair_num, const int min_backgrid_ptNum, const float variance_ths);
    void labelling(ALV_DATA *alv_data);

protected:
    NEGATIVE_DETECTOR();
    ~NEGATIVE_DETECTOR();

    class GarbageRobot
    {
    public:
        ~GarbageRobot()
        {
            delete NEGATIVE_DETECTOR::p_instance;
        }
    };

    static GarbageRobot grobot;     // used to delete p_instance automatically when whole program ends
    std::vector<alv_Point3f> candidate;
    std::vector<alv_Point3f> ground;
    std::vector<alv_Point3f> corner;
    std::vector<NEGATIVE_PAIR> negative_pairs;
    std::vector<NEGATIVE_CLUSTER> clusters;

    int recursive_depth;
    int depth_threshold;
    float delta;
};

#endif // DETECT_NEGATIVE_OBSTACLE_H
