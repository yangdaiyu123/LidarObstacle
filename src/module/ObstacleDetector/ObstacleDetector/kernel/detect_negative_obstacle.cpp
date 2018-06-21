/* Created  :   Linhui
 * Date     :   2016-08-05
 * Usage    :   definition of negative obstacle detector class
*/
#include <iostream>
#include <strstream>
#include <vector>
#include <queue>
#include <opencv/cv.h>
#include <exception>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include "detect_negative_obstacle.h"
using namespace std;
using namespace cv;

NEGATIVE_PAIR::NEGATIVE_PAIR(alv_Point3f front_p, alv_Point3f back_p, float x, float y, int idx):
    front(front_p), back(back_p), center_x(x), center_y(y), id(idx)
{
    avg_distance = (front_p.distance + back_p.distance)/2;
    xy_length = sqrt((front.x-back.x)*(front.x-back.x)+(front.y-back.y)*(front.y-back.y));
}

NEGATIVE_CLUSTER::NEGATIVE_CLUSTER():avg_x(0.0), avg_y(0.0),xy_length_variance(0.0){}

void NEGATIVE_CLUSTER::cal_average()
{
    assert(!data.empty());
    set<NEGATIVE_PAIR>::iterator it;
    for(it = data.begin(); it != data.end(); it++)
    {
        avg_x += it->center_x;
        avg_y += it->center_y;
    }
    avg_x /= static_cast<float>(data.size());
    avg_y /= static_cast<float>(data.size());
}

float NEGATIVE_CLUSTER::variance()
{
    assert(!data.empty());
    set<NEGATIVE_PAIR>::iterator it;
    float result = 0.0;
    for(it = data.begin(); it != data.end(); it++)
        result = result + pow(it->center_x-avg_x, 2) + pow(it->center_y-avg_y, 2);
    result /= data.size();
    return result;
}

void NEGATIVE_CLUSTER::length_variance()
{
    if(data.empty() || data.size() == 1)
        xy_length_variance = 0.0;
    float avg_length = 0.0;
    set<NEGATIVE_PAIR>::iterator it;
    for(it = data.begin(); it != data.end(); it++)
        avg_length += it->xy_length;

    avg_length /= static_cast<float>(data.size());

    for(it = data.begin(); it != data.end(); it++)
        xy_length_variance += static_cast<float>(pow(it->xy_length-avg_length,2));
    xy_length_variance = sqrt(xy_length_variance)/data.size();
}

void NEGATIVE_CLUSTER::pick_out(const NEGATIVE_PAIR& val)
{
    int N = data.size();
    data.erase(val);
    // update avg_x & avg_y
    avg_x = N>1?(N*avg_x - val.center_x)/static_cast<float>(N-1):0.0;
    avg_y = N>1?(N*avg_y - val.center_y)/static_cast<float>(N-1):0.0;
}

void NEGATIVE_CLUSTER::put_in(const NEGATIVE_PAIR& val)
{
    int N = data.size();
    data.insert(val);
    // update avg_x & avg_y
    avg_x = (N*avg_x + val.center_x)/static_cast<float>(N+1);
    avg_y = (N*avg_y + val.center_y)/static_cast<float>(N+1);
}

NEGATIVE_DETECTOR* NEGATIVE_DETECTOR::p_instance = nullptr;

NEGATIVE_DETECTOR::NEGATIVE_DETECTOR():recursive_depth(0), depth_threshold(0),delta(0.0){}

NEGATIVE_DETECTOR::~NEGATIVE_DETECTOR(){}

bool CenterCMP(const MSCenter &a, const MSCenter &b)
{
    if(a.x != b.x)
        return a.x < b.x;
    else if(a.y != b.y)
        return a.y < b.y;
    else
        return a.length < b.length;
}

//  聚类--method 1: MeanShift
void NEGATIVE_DETECTOR::clustering_MeshShift()
{
    if(negative_pairs.empty())
        return;

    // generate dataset
    vector<MSData> input;
    MSData point;
    for(int i=0; i<negative_pairs.size(); i++)
    {
        point.data[0] = negative_pairs[i].center_x / 20.0;
        point.data[1] = negative_pairs[i].center_y / 20.0;
        point.data[2] = negative_pairs[i].xy_length / 20.0;
        input.push_back(point);
    }

    // mean shift processing, find group center for each line pair
    MEANSHIFT meanshift(2.0, 0.0001);
    meanshift.SetInputData(input);
    vector<MSData> centers = meanshift.applyMeanShift();

    // clustering
    vector<MSCenter> vCenters;
    for(int i=0; i<centers.size(); i++)
    {
        vCenters.push_back(MSCenter(int(10.0*centers[i].data[0]),
                                    int(10.0*centers[i].data[1]),
                                    int(10.0*centers[i].data[2]),
                                    i));
    }
    sort(vCenters.begin(), vCenters.end(), CenterCMP);

    NEGATIVE_CLUSTER cluster;
    cluster.data.insert(negative_pairs[vCenters[0].ind]);
    for(int i=1; i<vCenters.size(); i++)
    {
        if(vCenters[i] == vCenters[i-1])    // in the same cluster
        {
            cluster.data.insert(negative_pairs[vCenters[i].ind]);
        }
        else
        {
            clusters.push_back(cluster);
            cluster.data.clear();
            cluster.data.insert(negative_pairs[vCenters[i].ind]);
        }
    }
    clusters.push_back(cluster);
}

// 聚类--method 2
void NEGATIVE_DETECTOR::clustering(int depth_ths, float delta_val)
{
    depth_threshold = depth_ths;
    delta = delta_val;
    if(negative_pairs.empty())
        return;

    NEGATIVE_CLUSTER cluster;
    int N = negative_pairs.size();
    for(int i=0; i<N; i++)
        cluster.data.insert(negative_pairs[i]);
    cluster.cal_average();

    recursive(cluster);
}

float cal_E(NEGATIVE_CLUSTER cluster1, NEGATIVE_CLUSTER cluster2)
{
    int N1 = cluster1.data.size();
    int N2 = cluster2.data.size();
    if(N1 == 0 || N2 == 0)
        return 0.0;
    return N1*N2*sqrt(pow(cluster1.avg_x-cluster2.avg_x,2)+pow(cluster1.avg_y-cluster2.avg_y,2))/static_cast<float>(N1+N2);
}

// 从一个集合分离一个样本到另一个集合
float split(NEGATIVE_CLUSTER& cluster, NEGATIVE_CLUSTER& new_cluster)
{
    if(cluster.data.empty())
        return 0.0;
    std::set<NEGATIVE_PAIR>::iterator it, max;
    float E = 0.0, max_E = 0.0;
    set<NEGATIVE_PAIR> tmp_set;
    tmp_set.insert(cluster.data.begin(), cluster.data.end());
    // pick out one single sample from cluster to new_cluster to make the greatest E
    for(it = tmp_set.begin(); it != tmp_set.end(); it++)
    {
        cluster.pick_out(*it);
        new_cluster.put_in(*it);
        E = cal_E(cluster, new_cluster);
        if(it == tmp_set.begin() || E > max_E)
        {
            max_E = E;
            max = it;
        }
        new_cluster.pick_out(*it);
        cluster.put_in(*it);
    }
    // update cluster & new_cluster
    cluster.pick_out(*max);
    new_cluster.put_in(*max);
    return max_E;
}

void NEGATIVE_DETECTOR::recursive(NEGATIVE_CLUSTER cluster)
{
    recursive_depth++;
    if(recursive_depth > depth_threshold)
        return;
    if(cluster.data.size() <= 1 || cluster.variance() <= delta)
    {
        cluster.length_variance();
        clusters.push_back(cluster);
        return;
    }

    NEGATIVE_CLUSTER new_cluster;   // new cluster
    NEGATIVE_CLUSTER tmp_cluster, tmp_new_cluster;

    float E_prev = split(cluster, new_cluster);
    tmp_cluster = cluster;
    tmp_new_cluster = new_cluster;
    float E_cur = split(cluster, new_cluster);
    while(E_cur > E_prev)
    {
        tmp_cluster = cluster;
        tmp_new_cluster = new_cluster;
        E_prev = E_cur;
        E_cur = split(cluster, new_cluster);
    }

    recursive(tmp_cluster);
    recursive_depth--;
    recursive(tmp_new_cluster);
    recursive_depth--;

    recursive_depth--;
    return;
}

NEGATIVE_DETECTOR* NEGATIVE_DETECTOR::get_instance()
{
    if(p_instance == nullptr)
    {
        m_lock.lock();  // prevent multi-thread ambiguous
        if(p_instance == nullptr)
            p_instance = new NEGATIVE_DETECTOR;
        m_lock.unlock();
    }
    return p_instance;
}


void NEGATIVE_DETECTOR::reset()
{
    candidate.clear();
    ground.clear();
    corner.clear();
    negative_pairs.clear();
    clusters.clear();
    recursive_depth = 0;
}


void NEGATIVE_DETECTOR::find_negative_pairs(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE)
{
    const int map_range_front = alv_data->para_table.map_range_front;
    const int map_range_rear = alv_data->para_table.map_range_rear;
    const int map_range_left = alv_data->para_table.map_range_left;
    const int map_range_right = alv_data->para_table.map_range_right;


    alv_Point3f *prev = NULL;
    int negative_pair_cnt = 0;
    for(int beam = 0; beam<BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<BEAM_POINTSIZE; cnt++)
        {
            alv_Point3f *pt = reinterpret_cast<alv_Point3f*>(pointcloud) + beam*BEAM_POINTSIZE + cnt;
            if(pt->x <= -map_range_left || pt->x >= map_range_right || pt->y <= -map_range_rear || pt->y >= map_range_front)    // beyond map range
                continue;
            if(static_cast<int>(sqrt((pt->x)*(pt->x)+(pt->y)*(pt->y))) > NEGDET_MAX_RANGE
                    || static_cast<int>(sqrt((pt->x)*(pt->x)+(pt->y)*(pt->y))) < NEGDET_MIN_RANGE)   // farer than 15 m
            {
                pt->neighbor_dis = INVALID_VAL;
                continue;
            }


            /* part I.calculate neighbor distance*/
            if(cnt < WIN_SIZE || cnt >= BEAM_POINTSIZE-WIN_SIZE)
            {
                pt->flatness = INVALID_VAL;
                pt->attribute = PT_UNDEFINE;
                pt->sub_flatness = INVALID_VAL;
                pt->sub_attribute = PT_UNDEFINE;
                pt->neighbor_dis = INVALID_VAL;
                pt->prev = nullptr;
                continue;
            }

            if(pt->valid && (pt-1)->valid)
            {
                pt->neighbor_dis = static_cast<int>(10.0*point_distance(*pt, *(pt-1))); // in mm
            }
            else
            {
                pt->neighbor_dis = INVALID_VAL;
            }
//            if(!prev && pt->valid)
//            {
//                prev = pt;
//                pt->flatness = INVALID_VAL;
//                pt->attribute = PT_UNDEFINE;
//                pt->sub_flatness = INVALID_VAL;
//                pt->sub_attribute = PT_UNDEFINE;
//                pt->neighbor_dis = INVALID_VAL;
//                pt->prev = nullptr;
//                continue;
//            }
//            if(prev && pt->valid)
//            {
//                pt->neighbor_dis = static_cast<int>(10.0*point_distance(*pt, *prev)); // in mm
//                pt->prev = prev;
//                prev = pt;
//            }
//            else
//            {
//                pt->neighbor_dis = INVALID_VAL;
//                pt->prev = nullptr;
//            }


            /* part II. calculate local flatness*/
            // step 1.check neighbor valid
            vector<int> front_invalid;
            vector<int> back_invalid;
            bool sub_valid_flag = true;
            for(int i=-WIN_SIZE; i<0; i++)
            {
                if(!(pt+i)->valid)
                {
                    back_invalid.push_back(i);
                    if(sub_valid_flag && i >= -MIN_WIN_SIZE)
                        sub_valid_flag = false;
                }
            }
            for(int i=1; i<=WIN_SIZE; i++)
            {
                if(!(pt+i)->valid)
                {
                    front_invalid.push_back(i);
                    if(sub_valid_flag && i <= MIN_WIN_SIZE)
                        sub_valid_flag = false;
                }
            }

            // step 2-1.calculate local flatness
            set<int> invalid_set;
            for(size_t idx = 0; idx < back_invalid.size(); idx++)
            {
                invalid_set.insert(back_invalid[idx]);
                invalid_set.insert(-back_invalid[idx]);
            }
            for(size_t idx = 0; idx < front_invalid.size(); idx++)
            {
                invalid_set.insert(front_invalid[idx]);
                invalid_set.insert(-front_invalid[idx]);
            }

            if(pt->neighbor_dis<=NEGATIVE_GAP_MAX_SIZE && (back_invalid.size()+front_invalid.size())<=2)
            {
                float dx = 0.0, dy = 0.0, dz = 0.0;
                for(int i=-WIN_SIZE; i<=WIN_SIZE; i++)
                {
                    if(i == 0)
                        continue;

                    if(invalid_set.find(i) != invalid_set.end())
                        continue;
                    float length = point_distance(*pt, *(pt+i));
                    dx += (pt->x-(pt+i)->x)/length;
                    dy += (pt->y-(pt+i)->y)/length;
                    dz += (pt->z-(pt+i)->z)/length;
                }
                pt->flatness = static_cast<int>(
                                    100*sqrt(pow(static_cast<double>(dx),2)
                                    + pow(static_cast<double>(dy),2)
                                    + pow(static_cast<double>(dz),2)));
                if(pt->flatness <= FLAT_THS)
                    pt->attribute = PT_FLAT;
                else if(pt->flatness <= LESS_FLAT_THS)
                    pt->attribute = PT_LESS_FLAT;
                else if(pt->flatness > LESS_SHARP_THS)
                    pt->attribute = PT_LESS_SHARP;
                else if(pt->flatness > SHARP_THS)
                    pt->attribute = PT_SHARP;
                else
                    pt->attribute = PT_UNDEFINE;
            }
            else
            {
                pt->flatness = INVALID_VAL;
                pt->attribute = PT_UNDEFINE;
            }

            // step 2-2.caculate sub local flatness
            if(sub_valid_flag)
            {
                float ddx = 0.0, ddy = 0.0, ddz = 0.0;
                for(int i=-MIN_WIN_SIZE; i<=MIN_WIN_SIZE; i++)
                {
                    if(i==0)
                        continue;
                    float length = point_distance(*pt, *(pt+i));
                    ddx += (pt->x-(pt+i)->x)/length;
                    ddy += (pt->y-(pt+i)->y)/length;
                    ddz += (pt->z-(pt+i)->z)/length;
                }
                pt->sub_flatness = static_cast<int>(
                                    100*sqrt(pow(static_cast<double>(ddx),2)
                                    + pow(static_cast<double>(ddy),2)
                                    + pow(static_cast<double>(ddz),2)));
                if(pt->sub_flatness <= FLAT_THS)
                    pt->sub_attribute = PT_FLAT;
                else if(pt->sub_flatness <= LESS_FLAT_THS)
                    pt->sub_attribute = PT_LESS_FLAT;
                else if(pt->sub_flatness > LESS_SHARP_THS)
                    pt->sub_attribute = PT_LESS_SHARP;
                else if(pt->sub_flatness > SHARP_THS)
                    pt->sub_attribute = PT_SHARP;
                else
                    pt->sub_attribute = PT_UNDEFINE;
            }
            else
            {
                pt->sub_flatness = INVALID_VAL;
                pt->sub_attribute = PT_UNDEFINE;
            }
        }


        /* part III.generate negative obstacle candidate*/
        for(int cnt = 2*WIN_SIZE+1; cnt<BEAM_POINTSIZE-2*WIN_SIZE-1; cnt++)
        {
            const alv_Point3f *pt = reinterpret_cast<const alv_Point3f*>(pointcloud) + beam*BEAM_POINTSIZE + cnt;

            // 过滤点1.凹障碍后沿没有落在检测范围内则抛弃
            if(static_cast<int>(sqrt((pt->x)*(pt->x)+(pt->y)*(pt->y))) > NEGDET_MAX_RANGE
                    || static_cast<int>(sqrt((pt->x)*(pt->x)+(pt->y)*(pt->y))) < NEGDET_MIN_RANGE)   // farer than 10 m
                continue;

            // 过滤点2.neighbor_dis无效，或者大于凹障碍尺寸上下限则抛弃
            if(pt->neighbor_dis == INVALID_VAL || (pt+1)->neighbor_dis == INVALID_VAL || pt->neighbor_dis > NEGATIVE_GAP_MAX_SIZE || pt->neighbor_dis < NEGATIVE_GAP_MIN_SIZE)
                continue;


            // step 1. find distance leaps
//-------------------------------------------------------------------------------------------------------//
//          // 原版本要求凹障碍后沿点的neighbor_dis比前后WIN_SIZE个点都要大很多，这不利于检测宽度较窄的壕沟
//            bool is_disleap = true;
//            for(int i=-WIN_SIZE; i<=WIN_SIZE; i++)
//            {
//                if(i == 0 || !(pt+i)->neighbor_dis == INVALID_VAL)
//                    continue;
//                if((pt->neighbor_dis-(pt+i)->neighbor_dis) < NEGATIVE_NEIGHBOR_MIN_DIFF)
//                {
//                    is_disleap = false;
//                    break;
//                }
//            }
//            if(is_disleap)
//-------------------------------------------------------------------------------------------------------//

            // 过滤点3.凹障碍后沿点的neighbor_dis应该比前一个点大一些，比后一个点大很多，后一个点的neighbor_dis通常很小
            if(pt->neighbor_dis > (pt-1)->neighbor_dis
                    && pt->neighbor_dis-(pt+1)->neighbor_dis > NEGATIVE_NEIGHBOR_MIN_DIFF
                    && (pt+1)->neighbor_dis < NEGATIVE_NEIGHBOR_MIN_DIS)
            {
                // step 2. fit line
                vector<cv::Point3f> neighbor_points0;
                int prevCnt = 0;
                for(int i=-1; i>=-2*WIN_SIZE; i--)
                {
                    if((pt+i)->attribute == PT_FLAT || (pt+i)->attribute == PT_LESS_FLAT)
                    {
                        neighbor_points0.push_back((const_cast<alv_Point3f*>(pt+i))->toCvPoint3f());
                        prevCnt++;
                        if(prevCnt > WIN_SIZE)
                            break;
                    }
                }

                if(neighbor_points0.size() >= WIN_SIZE-2)
                {
                    Vec6f line0;
                    fitLine(neighbor_points0, line0, CV_DIST_L2, 0, 0.5, 0.5);

                    // step 3.pick out edge points which are under the line in z axis
                    // (line0[0], line0[1], line0[2])表示空间直线的方向向量， （line0[3],line0[4],line0[5]）表示该直线上的一个点
                    float estimate_z0 = (line0[2]/line0[0]*(pt->x-line0[3])+line0[5] + line0[2]/line0[1]*(pt->y-line0[4])+line0[5])/2.0;
                    // 过滤点4.凹障碍后沿点位于其前方地面点拟合直线的下方
//                    float vec_norm = sqrt(line0[0]*line0[0]+line0[1]*line0[1]+line0[2]*line0[2]);
                    if((pt-1)->valid && estimate_z0 > pt->z/* && line0[2]/vec_norm < 0.2*/)
                    {
                        negative_pairs.push_back(NEGATIVE_PAIR(*(pt-1), *pt, ((pt-1)->x+pt->x)/2.0, ((pt-1)->y+pt->y)/2.0, ++negative_pair_cnt));
                    }
//                    if(estimate_z0 > pt->z && pt->prev)
//                    {
//                        if((pt->prev)->distance < pt->distance)
//                            negative_pairs.push_back(NEGATIVE_PAIR(*(pt->prev), *pt, ((pt->prev)->x+pt->x)/2.0, ((pt->prev)->y+pt->y)/2.0, ++negative_pair_cnt));
//                        else
//                            negative_pairs.push_back(NEGATIVE_PAIR(*pt, *(pt->prev), ((pt->prev)->x+pt->x)/2.0, ((pt->prev)->y+pt->y)/2.0, ++negative_pair_cnt));
//                    }
                }
            }
        }
    }
}


void NEGATIVE_DETECTOR::show_cluster_result(ALV_DATA *alv_data)
{
    // show clustering result
    cv::Mat cloud = cv::Mat::zeros(600, 600, CV_8UC3);
    int row, col;

    alv_Point3f *lidar16_pointcloud_L = reinterpret_cast<alv_Point3f *>(alv_data->lidar16_pointcloud_L);
    alv_Point3f *lidar16_pointcloud_R = reinterpret_cast<alv_Point3f *>(alv_data->lidar16_pointcloud_R);

    // show point cloud grid map
    for(size_t beam=0; beam<VLP16_BEAM_NUM; beam++)
    {
        for(size_t cnt = 0; cnt < VLP16_BEAM_POINTSIZE; cnt++)
        {
            const alv_Point3f &pt_L = *(lidar16_pointcloud_L+beam*VLP16_BEAM_POINTSIZE+cnt);
            if(pt_L.valid)
            {
                col = (pt_L.x+1500)/5;
                row = (pt_L.y+1000)/5;
                if(row>=0 && row <600 && col>=0 && col<600)
                    cloud.at<Vec3b>(row, col) = Vec3b(100,100,100);

            }
            const alv_Point3f &pt_R = *(lidar16_pointcloud_R+beam*VLP16_BEAM_POINTSIZE+cnt);
            if(pt_R.valid)
            {
                col = (pt_R.x+1500)/5;
                row = (pt_R.y+1000)/5;
                if(row>=0 && row <600 && col>=0 && col<600)
                    cloud.at<Vec3b>(row, col) = Vec3b(100,100,100);
            }
        }
    }

    // show line-pairs

    Scalar color_table[10] = {
        Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),
        Scalar(0,255,255),Scalar(255,0,255),Scalar(255,255,0),
        Scalar(255,100,200),Scalar(100,255,200),Scalar(100,200,255),Scalar(255,255,255)
    };
    for(size_t i = 0; i<clusters.size(); i++)
    {
        for(set<NEGATIVE_PAIR>::iterator it = clusters[i].data.begin(); it != clusters[i].data.end(); it++)
        {
            int col1 = (it->front.x+1500)/5;
            int row1 = (it->front.y+1000)/5;
            int col2 = (it->back.x+1500)/5;
            int row2 = (it->back.y+1000)/5;

            if(row1>=0 && row1 <600 && col1>=0 && col1<600 && row2>=0 && row2 <600 && col2>=0 && col2<600)
            {
                line(cloud, Point(col1, row1), Point(col2, row2), color_table[i],3);
//                circle(cloud, Point(col1, row1), 3, Scalar(255,0,255), -1);   // BGR
//                circle(cloud, Point(col2, row2), 3, Scalar(0,255,0), -1);
            }
        }
    }

    flip(cloud, cloud, 0);

    // show cluster xy_length_variance
    for(size_t i = 0; i<clusters.size(); i++)
    {
        stringstream ss;
        string text;
//        ss<<ios::fixed<<setw(4)<<clusters[i].xy_length_variance;
        ss<<i;
        ss>>text;
        int centerCol = (clusters[i].avg_x+1500+40)/5;
        int centerRow = 599 - (clusters[i].avg_y+1000+40)/5;
        putText(cloud, text, Point(centerCol, centerRow), 0, 1.0, Scalar(255,255,255));
    }
    imshow("clustering", cloud);
    waitKey(1);
}

void NEGATIVE_DETECTOR::remove_outlier(const ALV_DATA *alv_data, const unsigned int min_pair_num, const int min_backgrid_ptNum, const float variance_ths)
{
    const PARA_TABLE *para = &(alv_data->para_table);
    GRID **grids = alv_data->grid_map.grids;

    for(vector<NEGATIVE_CLUSTER>::iterator it = clusters.begin(); it!=clusters.end();)
    {
        int invalid_pair_num = 0;
        for(set<NEGATIVE_PAIR>::const_iterator it2 = it->data.begin();it2!=it->data.end(); it2++)
        {
            int col = (it2->back.x + para->map_range_left)/para->grid_size;
            int row = (it2->back.y + para->map_range_rear)/para->grid_size;
            if(row >= 0 && row < para->grid_rows && col >= 0 && col < para->grid_cols)
            {
                // 凹障碍线段后沿对应栅格点云少于min_backgrid_ptNum个则视为无效线段
                if(grids[row][col].ptNum < min_backgrid_ptNum)
                    invalid_pair_num++;
            }
        }
        // 凹障碍聚类内有效线段数量少于min_pair_num则抛弃
        if(it->data.size()-invalid_pair_num < min_pair_num || it->xy_length_variance > variance_ths)
            it = clusters.erase(it);
        else
        {
            it++;
        }
    }
    cout<<", negative obstacle num: "<<clusters.size()<<" ";
}


bool PIXEL_CMP(const PIXEL& p1, const PIXEL& p2)
{
    return p1.col < p2.col;
}


bool PIXEL_RCMP(const PIXEL& p1, const PIXEL& p2)
{
    return p1.col >= p2.col;
}

void NEGATIVE_DETECTOR::labelling(ALV_DATA *alv_data)
{
    const PARA_TABLE *para = &(alv_data->para_table);
    cv::Mat negative_mask = cv::Mat::zeros(para->grid_rows, para->grid_cols, CV_8UC3);
    const cv::Scalar mask_val = cv::Scalar(255,255,255);
    GRID **grids = alv_data->grid_map.grids;

    for(size_t i=0; i<clusters.size(); i++)
    {
        vector<PIXEL> negative_front_pixel;
        vector<PIXEL> negative_back_pixel;

        bool invalid = false;
        for(set<NEGATIVE_PAIR>::iterator it = clusters[i].data.begin(); it!=clusters[i].data.end(); it++)
        {
            int col = (it->front.x + para->map_range_left)/para->grid_size;
            int row = (it->front.y + para->map_range_rear)/para->grid_size;
            if(row<0 || row>=para->grid_rows || col<0 || col>=para->grid_cols)
            {
                invalid = true;
                break;
            }
            negative_front_pixel.push_back(PIXEL(row,col));
            col = (it->back.x + para->map_range_left)/para->grid_size;
            row = (it->back.y + para->map_range_rear)/para->grid_size;
            if(row<0 || row>=para->grid_rows || col<0 || col>=para->grid_cols)
            {
                invalid = true;
                break;
            }
            negative_back_pixel.push_back(PIXEL(row,col));
        }
        if(invalid)
            continue;

        sort(negative_front_pixel.begin(), negative_front_pixel.end(), PIXEL_CMP);
        sort(negative_back_pixel.begin(), negative_back_pixel.end(), PIXEL_RCMP);
        int pixelN = negative_front_pixel.size()+negative_back_pixel.size();

        cv::Point **contour = new cv::Point*[1];
        contour[0] = new cv::Point[pixelN];
        int id = 0;
        for(size_t t = 0; t < negative_front_pixel.size(); t++)
            contour[0][id++] = cv::Point(negative_front_pixel[t].col, negative_front_pixel[t].row);
        for(size_t t = 0; t < negative_back_pixel.size(); t++)
            contour[0][id++] = cv::Point(negative_back_pixel[t].col, negative_back_pixel[t].row);

        cv::polylines(negative_mask, const_cast<const Point**>(contour), &id, 1, true, mask_val);
        cv::fillPoly(negative_mask, const_cast<const Point**>(contour), &id, 1, mask_val);

        delete[] contour[0];
        delete[] contour;
    }

    for(int row=0; row<para->grid_rows; row++)
    {
        for(int col=0; col<para->grid_cols; col++)
        {
            if(negative_mask.at<Vec3b>(row,col)[0] == 255)
                grids[row][col].attribute = GRID_NEG_OBS;
        }
    }
//    cv::flip(negative_mask, negative_mask, 0);
//    imshow("negative mask", negative_mask);
//    waitKey(1);
}


void MEANSHIFT::SetInputData(const std::vector<MSData> &input)
{
    for(int i=0; i<input.size(); i++)
        dataset.push_back(input[i]);
}

std::vector<MSData> MEANSHIFT::applyMeanShift()
{
    std::vector<bool> stop_moving;
    stop_moving.resize(dataset.size());
    for(int i=0; i<stop_moving.size(); i++)
        stop_moving[i] = false;

    std::vector<MSData> shifted_points = dataset;

    double max_shift_distance = 0.0;
    do
    {
        max_shift_distance = 0.0;
        for(int i=0; i<shifted_points.size(); i++)
        {
            if(!stop_moving[i])
            {
                MSData point_new = shiftCenter(shifted_points[i]);
                double shift_distance = euclidean_distance(point_new, shifted_points[i]);
                if(shift_distance > max_shift_distance)
                    max_shift_distance = shift_distance;

                if(shift_distance <= EPSILON)
                    stop_moving[i] = true;
                shifted_points[i] = point_new;
            }
        }
    }while(max_shift_distance > EPSILON);

    return shifted_points;
}

MSData MEANSHIFT::shiftCenter(MSData prv_center)
{
    MSData shiftVector;
    shiftVector.data[0] = shiftVector.data[1] = shiftVector.data[2] = 0.0;
    double total_weight = 0.0;
    for(int i=0; i<dataset.size(); i++)
    {
        MSData tmp = dataset[i];
        double distance = euclidean_distance(prv_center, tmp);
        double weight = gaussian_kernel(distance);
        for(int j=0; j<3; j++)
        {
            shiftVector.data[j] += tmp.data[j] * weight;
        }
        total_weight += weight;
    }
    for(int i=0; i<3; i++)
    {
        shiftVector.data[i] /= total_weight;
    }
    return shiftVector;
}

double MEANSHIFT::gaussian_kernel(double distance)
{
    return (1.0/(sqrt(2*M_PI)*kernel_bandwidth))*
            exp(-0.5*(distance*distance)/(kernel_bandwidth*kernel_bandwidth)
           );
}

double MEANSHIFT::euclidean_distance(const MSData &data1, const MSData &data2)
{
    double sum = 0.0;
    for(int i=0; i<3; i++)
    {
        sum += (data1.data[i]-data2.data[i]) * (data1.data[i]-data2.data[i]);
    }
    return sqrt(sum);
}

void NEGATIVE_DETECTOR::detect(ALV_DATA *alv_data)
{
    reset();
    find_negative_pairs(alv_data, reinterpret_cast<const alv_Point3f**>(alv_data->lidar16_pointcloud_L), VLP16_BEAM_NUM, VLP16_BEAM_POINTSIZE);
    find_negative_pairs(alv_data, (const alv_Point3f**)alv_data->lidar16_pointcloud_R, VLP16_BEAM_NUM, VLP16_BEAM_POINTSIZE);
    clustering(5, 3000.0);
//    clustering_MeshShift();
    show_cluster_result(alv_data);
    remove_outlier(alv_data, 3, 2, 10000.0);
    labelling(alv_data);
}
