/* Created  :   Linhui
 * Date     :   2016-08-05
 * Usage    :
*/
#include "detect_positive_obstacle.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>

using namespace std;
using namespace cv;

POSITIVE_DETECTOR * POSITIVE_DETECTOR::p_instance = nullptr;
static std::mutex n_lock;   // 线程锁，防止多个线程同时访问 POSITIVE_DETECTOR::p_instance

POSITIVE_DETECTOR * POSITIVE_DETECTOR::get_instance()
{
    if(p_instance == nullptr)
    {
        n_lock.lock();  // prevent multi-thread ambiguous
        if(p_instance == nullptr)
            p_instance = new POSITIVE_DETECTOR;
        n_lock.unlock();
    }
    return p_instance;
}

POSITIVE_DETECTOR::POSITIVE_DETECTOR(){}
POSITIVE_DETECTOR::~POSITIVE_DETECTOR(){}


void POSITIVE_DETECTOR::mark_car_area(ALV_DATA *alv_data)
{
    const PARA_TABLE &para = alv_data->para_table;// alv_data->para_table包含了指针申请的动态内存，所以要么用引用的形式，要么自定义=重载，否则形参析构时会将实参的动态内存释放掉！！！
    GRID **grids = alv_data->grid_map.grids;
    // mark car area
    for(int row = para.grid_center_row - (para.car_length/2)/para.grid_size;
            row <= para.grid_center_row + (para.car_length/2)/para.grid_size;
            row++)
    {
        for(int col = para.grid_center_col - (para.car_width/2)/para.grid_size;
                col <= para.grid_center_col + (para.car_width/2)/para.grid_size;
                col++)
        {
            grids[row][col].attribute = GRID_CAR_AREA;
            grids[row][col].known = true;
        }
    }
}

void POSITIVE_DETECTOR::update_grids(ALV_DATA *alv_data, const alv_Point3f **pointcloud, const int BEAM_NUM, const int BEAM_POINTSIZE)
{
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    // update grids
    for(int beam = 0; beam < BEAM_NUM; beam++)
    {
        for(int cnt = 0; cnt < BEAM_POINTSIZE; cnt++)
        {
            const alv_Point3f *pt = (alv_Point3f *)pointcloud + beam*BEAM_POINTSIZE+cnt;

            if(!pt->valid)
                continue;
            int row = (pt->y + para.map_range_rear)/para.grid_size;
            int col = (pt->x + para.map_range_left)/para.grid_size;

            if(!(row >= 0 && row < para.grid_rows && col >= 0 && col < para.grid_cols))
                continue;
            if(grids[row][col].attribute == GRID_CAR_AREA)
                continue;

            if(grids[row][col].known == false)
            {
                grids[row][col].known =true;
                grids[row][col].max_height = grids[row][col].min_height = pt->z;
                grids[row][col].ground_height = grids[row][col].min_height;
            }
            else
            {
                grids[row][col].max_height = pt->z > grids[row][col].max_height? pt->z:grids[row][col].max_height;
                grids[row][col].min_height = pt->z < grids[row][col].min_height? pt->z:grids[row][col].min_height;
                grids[row][col].ground_height = grids[row][col].min_height;
            }
            grids[row][col].ptNum++;
            grids[row][col].points.push_back(*pt);
            if(pt->intensity > 0 && pt->intensity < para.weak_flatness_threshold)
                grids[row][col].weak_intensity_cnt++;
        }
    }

    for(int row=0; row<para.grid_rows; row++)
    {
        for(int col=0; col<para.grid_cols; col++)
        {
            grids[row][col].dis_height = grids[row][col].max_height - grids[row][col].min_height;
        }
    }
}


void POSITIVE_DETECTOR::process_lidar1_data(ALV_DATA *alv_data)
{
    PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;

    // process LIDAR 1 data
    for(int row=0;row<para.grid_rows;row++){
        for(int col=0;col<para.grid_cols;col++){
            if (grids[row][col].attribute == GRID_CAR_AREA)
                continue;
            if(alv_data->lidar1_grids[row][col] == 255){
                grids[row][col].known = true;
                grids[row][col].attribute = GRID_POS_OBS;
                double old = (double)grids[row][col].dis_height;
                grids[row][col].dis_height =std::max(50.0,old);// lidar1 default height 50cm
            }
        }
    }
}


void POSITIVE_DETECTOR::remove_suspended_obs(ALV_DATA *alv_data) // 滤除悬空障碍物
{
    const PARA_TABLE &para = alv_data->para_table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;

    // 估计地面高度
    const int estimate_grid_front = 1500/para.grid_size;
    const int estimate_grid_rear = 1000/para.grid_size;
    const int estimate_grid_left = 500/para.grid_size;
    const int estimate_grid_right = 500/para.grid_size;
    int win_size = 2;
    for(int row = para.grid_center_row-estimate_grid_rear; row<=para.grid_center_row+estimate_grid_front; row++)
    {
        for(int col = para.grid_center_col-estimate_grid_left; col<=para.grid_center_col+estimate_grid_right; col++)
        {
            if(grids[row][col].attribute == GRID_CAR_AREA || grids[row][col].attribute == GRID_TRAVESABLE)
                continue;

            int validNum = 0;
            float neighbor_height = 0.0;
            for(int i = row-win_size; i <= row+win_size; i++)
            {
                for(int j = col-win_size; j <= col+win_size; j++)
                {
                    if(grids[i][j].attribute == GRID_CAR_AREA)
                        continue;
                    if(grids[i][j].attribute == GRID_TRAVESABLE || (grids[i][j].attribute != GRID_TRAVESABLE && grids[i][j].update_ground))
                    {
                        neighbor_height += grids[i][j].ground_height;
                        validNum++;
                    }
                }
            }
            if(validNum)
            {
                grids[row][col].ground_height = neighbor_height/(float)validNum;
                grids[row][col].update_ground = true;
            }
        }
    }

    // 标记悬空障碍物栅格
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if(grids[row][col].attribute == GRID_POS_OBS/* && grids[row][col].dis_height > (float)para.suspend_obs_ths*/)
            {
                // 障碍栅格内点云根据z轴按递增排序
                sort(grids[row][col].points.begin(), grids[row][col].points.end(), [](const alv_Point3f &p1, const alv_Point3f &p2)->bool{return p1.z < p2.z;});
				// 初始化prev_height为ground_height，然后依次检查后一个点是否比前一个点高度差超过阈值，若是，则标记为悬空障碍物
                float prev_height = grids[row][col].ground_height;
                for(int i=0; i<grids[row][col].points.size(); i++)
                {
                    if(grids[row][col].points[i].z - prev_height > (float)para.suspend_obs_ths)
                    {
                        grids[row][col].attribute = GRID_SUSPEND_OBS;
                        break;
                    }
                    prev_height = grids[row][col].points[i].z;
                }
            }
        }
    }

	// 再次过滤一部分障碍物栅格
    win_size = 2;
    int ths = 1;
    for(int row = win_size; row < Rows-win_size; row++)
    {
        for(int col = win_size; col < Cols-win_size; col++)
        {
            if(grids[row][col].attribute == GRID_POS_OBS)
            {
                int susp_cnt = 0;
                for(int i=row-win_size; i<=row+win_size; i++)
                {
                    for(int j = col-win_size; j <= col+win_size; j++)
                    {
                        if(grids[row][col].attribute == GRID_SUSPEND_OBS)
                            susp_cnt++;
                    }
                }
                if(susp_cnt >= ths)
                    grids[row][col].attribute = GRID_SUSPEND_OBS;
            }
        }
    }
}


void POSITIVE_DETECTOR::classify_unknown_grid(ALV_DATA *alv_data)
{
    const int *angle_grid_num = alv_data->para_table.polar_table.angle_grid_num;
    PIXEL **polar_table = alv_data->para_table.polar_table.table;
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    const float lidar_height = alv_data->para_table.lidar32_expara.T[2];

    // occulusion
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++){
        int grid_shadowtop = 0;
        for (int i = 5; i < angle_grid_num[angle_cnt]-1; i++){
            int row = polar_table[angle_cnt][i].row;
            int col = polar_table[angle_cnt][i].col;

            if (grids[row][col].attribute == GRID_POS_OBS){
                //calculate shadow top and update
                int grid_height = grids[row][col].max_height;
                int top;
                if(lidar_height > grid_height)
                    top = (int)((lidar_height * i) / (lidar_height - grid_height));
                else
                    top = angle_grid_num[angle_cnt];
                if (top > grid_shadowtop)
                    grid_shadowtop = top;
            }
            else if(i <= grid_shadowtop)
                grids[row][col].attribute = GRID_OCCULUSION;
        }
    }

    // dangerous
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if(grids[row][col].attribute != GRID_UNKNOWN)
            {
                int beam = alv_data->para_table.grid_2_polar_beam[row][col];
                int angle = alv_data->para_table.grid_2_polar_angle[row][col];
                alv_data->polar_grids[beam][angle].valid++;
            }
        }
    }

    int win_size = 1;
    for(int beam = win_size; beam < POLAR_BEAM_NUM_2-win_size; beam++)
    {
        for(int angle = win_size; angle < POLAR_ANGLE_NUM_2-win_size; angle++)
        {
            int validCnt = 0;
            for(int i = beam-win_size; i <= beam + win_size; i++)
            {
                for(int j = angle-win_size; j <= angle + win_size; j++)
                {
                    if(alv_data->polar_grids[i][j].valid != 0)
                        validCnt++;
                }
            }
            if(validCnt > 5)
                alv_data->polar_grids[beam][angle].valid = 1;
        }
    }
    filt_dangerous(alv_data);
}

void POSITIVE_DETECTOR::filt_dangerous(ALV_DATA *alv_data)
{
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    int area_ths = alv_data->para_table.dangerous_area_threshold;
    Mat dangerous = Mat::zeros(Rows, Cols, CV_8U);
    for(int row = 0; row < Rows; row++)
    {
        uint8_t* di = dangerous.ptr<uint8_t>(row);
        for(int col = 0; col < Cols; col++)
        {
            if(row >= alv_data->para_table.grid_center_row - 600/ alv_data->para_table.grid_size &&
                    row <= alv_data->para_table.grid_center_row + 600/ alv_data->para_table.grid_size &&
                    col >= alv_data->para_table.grid_center_col - 600/ alv_data->para_table.grid_size &&
                    col <= alv_data->para_table.grid_center_col + 600/ alv_data->para_table.grid_size)
                continue;
            int beam = alv_data->para_table.grid_2_polar_beam[row][col];
            int angle = alv_data->para_table.grid_2_polar_angle[row][col];
            if(alv_data->polar_grids[beam][angle].valid == 0)
                di[col] = 255;
        }
    }

//    flip(dangerous, dangerous, 0);
//    imshow("dangerous", dangerous);
//    waitKey(1);

    // find contours
    vector<vector<Point> > contours;
    findContours(dangerous, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int n = contours.size();

    //find big area
    Mat img_tmp = Mat::zeros(Rows, Cols, CV_8U);
    Scalar color( 255, 255, 255);
    for (int i=0; i<n; ++i) {
        vector<Point>& contour = contours[i];
        int m = contour.size();
        double area = contourArea(contour, false);
        if(area > area_ths){
            int mark = 0;
            for(int j = 0; j < m; j++){
                if((contour[j].x-75)*(contour[j].x-75) + (contour[j].y-75-12)*(contour[j].y-75-12) < 150*150)
                    mark = 1;
            }
            if(mark == 1)
                drawContours( img_tmp, contours, i, color, CV_FILLED);
        }

    }

    GRID_ATTRIBUTE **known = new GRID_ATTRIBUTE*[Rows];
    for(int i=0; i<Rows; i++)
        known[i] = new GRID_ATTRIBUTE[Cols];
    for(int row = 0; row < Rows; row++){
        for(int col = 0; col < Cols; col++){
            known[row][col] = alv_data->grid_map.grids[row][col].attribute;
        }
    }

    for(int row = 0; row < Rows; row ++){
        uint8_t* di = img_tmp.ptr<uint8_t>(row);
        for(int col = 0; col < Cols; col ++){
            if(di[col] == 255)
                alv_data->grid_map.grids[row][col].attribute = GRID_DANGEROUS;
        }
    }

//    // 将被抹掉的非危险区域重新填入
//    for(row = 0; row < row_max; row ++){
//        for(col = 0; col < col_max; col ++){
//            if(known[row][col] != 4)
//                lidar_grids[row][col].grid_prop_feature.known = known[row][col];
//        }
//    }

    // free memory
    for(int i=0; i<Rows; i++)
        delete[] known[i];
    delete[] known;
}

void POSITIVE_DETECTOR::filt_grid(ALV_DATA *alv_data, int win_size, int ths)
{
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = win_size; row<Rows-win_size; row++)
    {
        for(int col = win_size; col<Cols-win_size; col++)
        {
            if(grids[row][col].attribute != GRID_POS_OBS)
                continue;

            int obs_cnt = 0;
            for(int i=row-win_size; i<=row+win_size; i++)
            {
                for(int j=col-win_size; j<=col+win_size; j++)
                {
                    if(grids[i][j].attribute == GRID_POS_OBS)
                        obs_cnt++;
                }
            }
            if(obs_cnt <= ths)
                grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }
}


void POSITIVE_DETECTOR::detect_obstacle_grid(ALV_DATA *alv_data)
{
    mark_car_area(alv_data);
    cout<<HDL32_BEAM_NUM<<" "<<HDL32_BEAM_POINTSIZE<<endl;
    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar32_pointcloud, HDL32_BEAM_NUM, HDL32_BEAM_POINTSIZE);
    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar16_pointcloud_L, VLP16_BEAM_NUM, VLP16_BEAM_POINTSIZE);
    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar16_pointcloud_R, VLP16_BEAM_NUM, VLP16_BEAM_POINTSIZE);
    update_grids(alv_data, (const alv_Point3f**)alv_data->lidar4_pointcloud, LIDAR4_BEAM_NUM, LIDAR4_BEAM_POINTSIZE);

    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
            if( !grids[row][col].known || grids[row][col].attribute == GRID_CAR_AREA)
                continue;
            if(grids[row][col].dis_height >= alv_data->para_table.obs_threshold)
            {
                grids[row][col].attribute = GRID_POS_OBS;
                if(row>=alv_data->para_table.grid_center_row
                   && row <= alv_data->para_table.grid_center_row+35
                   && col >= alv_data->para_table.grid_center_col-7
                   && col <= alv_data->para_table.grid_center_col+7
                   && grids[row][col].dis_height < alv_data->para_table.near_obstacle_threshold)
                    grids[row][col].attribute = GRID_TRAVESABLE;
            }
            else
                grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }

//#ifndef OFF_LINE
    process_lidar1_data(alv_data);
//#endif

    // single point filter
    filt_grid(alv_data, 1, 1);
//    std::cout<<grids[1][1].points[5].x<<std::endl;
}


void POSITIVE_DETECTOR::detect_water_surface(ALV_DATA *alv_data)
{
    GRID **grids = alv_data->grid_map.grids;
    int Rows = alv_data->para_table.grid_rows;
    int Cols = alv_data->para_table.grid_cols;
    for(int row = 0; row<Rows; row++)
    {
        for(int col = 0; col<Cols; col++)
        {
            if(row>=alv_data->para_table.grid_center_row + 10
                && row <= alv_data->para_table.grid_center_row+35
                && col >= alv_data->para_table.grid_center_col-7
                && col <= alv_data->para_table.grid_center_col+7
                && grids[row][col].attribute == GRID_TRAVESABLE)
            {
                if(grids[row][col].weak_intensity_cnt >= WATER_GRID_THRESHOLD)
                    grids[row][col].attribute = GRID_WATER;
            }
        }
    }
    int win_size = 2;
    for(int row = alv_data->para_table.grid_center_row + 10; row <= alv_data->para_table.grid_center_row+35; row++)
    {
        for(int col = alv_data->para_table.grid_center_col-7; col <= alv_data->para_table.grid_center_col+7; col++)
        {
            int neighbor = 0;
            if(grids[row][col].attribute != GRID_WATER)
                continue;
            for(int i=row-win_size; i<=row+win_size; i++)
            {
                for(int j = col-win_size; j <= col+win_size; j++)
                {
                    if(grids[i][j].attribute == GRID_WATER)
                        neighbor++;
                }
            }
            if(neighbor < 3)
                grids[row][col].attribute = GRID_TRAVESABLE;
        }
    }
}

/* 正障碍检测 */
void POSITIVE_DETECTOR::detect(ALV_DATA *alv_data)
{
    detect_obstacle_grid(alv_data);	// 标记出障碍物栅格、车体范围
    detect_water_surface(alv_data);
    remove_suspended_obs(alv_data);	// 标记悬空障碍物
    classify_unknown_grid(alv_data);// 标记阴影区、危险区域
}
