/* Created  :   Linhui
 * Date     :   2016-05-17
 * Usage    :   Definition of member function of class ALV_DATA
*/
#include "alv_data.h"
#include "program.h"
#include <iostream>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <cassert>
#include <cmath>
#include <cctype>
#include<python2.7/Python.h>
#include "detect_negative_obstacle.h"
#include "detect_positive_obstacle.h"
using namespace std;
using namespace cv;
/* alv_Point3f
*/
cv::Point3f alv_Point3f::toCvPoint3f()
{
    return Point3f(x,y,z);
}

/* HDL32_INTRINSIC_PARA part:
 *  constructor
*/
HDL32_INTRINSIC_PARA::HDL32_INTRINSIC_PARA()
{
    memset(angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
    memset(beam_order, 0, sizeof(int)*HDL32_BEAM_NUM);
    memset(sin_angleH, 0, sizeof(float)*36000);
    memset(cos_angleH, 0, sizeof(float)*36000);
}

/* LIDAR_EXTRINSIC_PARA part:
 *  constructor
*/
LIDAR_EXTRINSIC_PARA::LIDAR_EXTRINSIC_PARA()
{
    memset(R, 0, sizeof(float)*3*3);
    memset(T, 0, sizeof(float)*3);
}

/* VLP16_INTRINSIC_PARA part:
 *  constructor
*/
VLP16_INTRINSIC_PARA::VLP16_INTRINSIC_PARA()
{
    memset(angleV, 0, sizeof(float)*VLP16_BEAM_NUM);
    memset(sin_angleV, 0, sizeof(float)*VLP16_BEAM_NUM);
    memset(cos_angleV, 0, sizeof(float)*VLP16_BEAM_NUM);
    memset(sin_angleH, 0, sizeof(float)*36000);
    memset(cos_angleH, 0, sizeof(float)*36000);
    memset(beam_point_num, 0, sizeof(int)*VLP16_BEAM_NUM);
}


PARA_TABLE::PARA_TABLE()
{
    para_base_dir = "/home/wzq/QtProjects/Obstacle/bin/parameters";

    grid_size = 0;
    grid_rows = 0;
    grid_cols = 0;
    map_range_front = 0;
    map_range_rear = 0;
    map_range_left = 0;
    map_range_right = 0;
    blind_area_front = 0;
    blind_area_rear = 0;
    blind_area_left = 0;
    blind_area_right = 0;
    obs_threshold = 0;
    near_obstacle_threshold = 0;
    weak_flatness_threshold = 0;
    suspend_obs_ths = 0;
    grid_center_row = 0;
    grid_center_col = 0;
    car_length = 0;
    car_width = 0;

    for(int i=0; i<POLAR_BEAM_NUM_2; i++)
        polar_beam_grids[i] = 0;
    environment = ENV_UNKNOWN;
}


PARA_TABLE::~PARA_TABLE()
{
    for(int row=0; row<grid_rows; row++)
    {
        delete[] grid_2_polar_angle[row];
        delete[] grid_2_polar_beam[row];
    }
    delete[] grid_2_polar_angle;
    delete[] grid_2_polar_beam;
}


/* GRID part:
 *  constructor, each grid is init empty with unknown attribute
*/
GRID::GRID()
{
    known = false;
    points.clear();
    attribute = GRID_UNKNOWN;
    max_height = 0.0;
    min_height = 0.0;
    dis_height = 0.0;
    ground_height = 99999.0;    // initially great enough
    weak_intensity_cnt = 0;
    update_ground = false;
    ptNum = 0;
}

void GRID::cleanup()
{
    known = false;
    points.clear();
    attribute = GRID_UNKNOWN;
    max_height = 0.0;
    min_height = 0.0;
    dis_height = 0.0;
    ground_height = 99999.0;    // initially great enough
    weak_intensity_cnt = 0;
    update_ground = false;
    ptNum = 0;
}

/* GRID_MAP part:
 *  constructor & destructor
*/
GRID_MAP::GRID_MAP()
{
    grids = NULL;
    rows = 0;
    cols = 0;
}

// overload constructor
void GRID_MAP::setup(int row_num, int col_num)
{
    rows = row_num;
    cols = col_num;
    grids = new GRID *[rows];
    for(int i=0; i<rows; i++)
        grids[i] = new GRID[cols];
}

GRID_MAP::~GRID_MAP()
{
    for(int i=0; i<rows; i++)
        delete[] grids[i];
    delete[] grids;
}

void GRID_MAP::cleanup()
{
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            grids[i][j].cleanup();
        }
    }
}


/* ALV_DATA part:
 *  constructor & destructor
 *  initiation of parameters
 *  setup
 *  cleanup
*/

ALV_DATA::ALV_DATA()
{
    memset(lidar16_pointcloud_L, 0, sizeof(alv_Point3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);
    memset(lidar16_pointcloud_R, 0, sizeof(alv_Point3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);

    lidar4_Data = new LIDAR4_DATA;
    memset(lidar4_Data, 0, sizeof(LIDAR4_DATA));

    lidar1_grids = new unsigned char*[GRIDMAP_HEIGHT];
    for(int i=0; i<GRIDMAP_HEIGHT; i++)
    {
        lidar1_grids[i] = new unsigned char[GRIDMAP_WIDTH];
        memset(lidar1_grids[i], 0, sizeof(unsigned char)*GRIDMAP_WIDTH);
    }
}

ALV_DATA::~ALV_DATA()
{
    delete lidar4_Data;
    for(int i=0; i<GRIDMAP_HEIGHT; i++)
      delete[] lidar1_grids[i];
    delete[] lidar1_grids;
}

bool ALV_DATA::init_para()
{
    if(init_lidar4_para() && init_lidar16_para() && init_lidar32_para() && read_alv_config())
        return true;
    else
        return false;
}

bool ALV_DATA::init_lookup_table()
{
    for(int angle_cnt = 0; angle_cnt < POLAR_ANGLE_NUM; angle_cnt++)
    {
        for (int grid_cnt = 0; grid_cnt < ANGLE_GRID_NUM; grid_cnt++)
        {
            float angle = (float)angle_cnt/(float)POLAR_ANGLE_NUM*360.0;
            int row = grid_cnt*sin(angle*M_PI/180.0) + para_table.grid_center_row;
            int col = grid_cnt*cos(angle*M_PI/180.0) + para_table.grid_center_col;
            if(row>=0 && row<para_table.grid_rows && col>=0 && col<para_table.grid_cols)
            {
                para_table.polar_table.table[angle_cnt][grid_cnt] = PIXEL(row, col);
                para_table.polar_table.angle_grid_num[angle_cnt]++;
            }
            else
                break;
        }
    }

    int prev = 0;
    int tmp_table[POLAR_BEAM_NUM_2+1];
    tmp_table[0] = 0;
    for(int i=0; i<POLAR_BEAM_NUM_2; i++)
    {
        float height = para_table.lidar32_expara.T[2];
        float angleV = 90.0 + para_table.lidar32_inpara.angleV[para_table.lidar32_inpara.beam_order[i]];
        if(angleV >= 90.0)
            tmp_table[i+1] = prev;
        float dis = tan(angleV/180.0*M_PI)*height;
        tmp_table[i+1] = dis/para_table.grid_size;
        prev = tmp_table[i+1];
    }
    for(int i=0; i<POLAR_BEAM_NUM_2; i++)
    {
        para_table.polar_beam_grids[i] = (tmp_table[i] + tmp_table[i+1])/2;
    }


    para_table.grid_2_polar_beam = new int*[para_table.grid_rows];
    para_table.grid_2_polar_angle = new int*[para_table.grid_rows];
    for(int row = 0; row < para_table.grid_rows; row++)
    {
        para_table.grid_2_polar_beam[row] = new int[para_table.grid_cols];
        para_table.grid_2_polar_angle[row] = new int[para_table.grid_cols];
    }
    for(int row = 0; row < para_table.grid_rows; row++)
    {
        for(int col = 0; col < para_table.grid_cols; col++)
        {
            double act_row = row - para_table.grid_center_row;
            double act_col = col - para_table.grid_center_col;
            if(act_row == 0 && act_col == 0)
            {
                para_table.grid_2_polar_angle[row][col] = 0;
                para_table.grid_2_polar_beam[row][col] = 0;
                continue;
            }
            double  dis = sqrt(act_row*act_row + act_col*act_col);
            int beam = 0;
            for(beam = 0; beam < POLAR_BEAM_NUM_2; beam++)
            {
                if(dis <= para_table.polar_beam_grids[beam])
                    break;
            }
            para_table.grid_2_polar_beam[row][col] = beam;


            int angle = acos(act_col / dis)/M_PI*180.0;
            if(act_row < 0)
                angle = 359-angle;
            para_table.grid_2_polar_angle[row][col] = angle;
        }
    }

    return true;
}

bool CMP(pair<float, int> a, pair<float, int> b)
{
    return a.first < b.first;
}

bool ALV_DATA::arrange_beam_order()
{
    vector<pair<float, int> > beams;
    for(int i=0; i<HDL32_BEAM_NUM; i++)
    {
        beams.push_back(pair<float, int>(para_table.lidar32_inpara.angleV[i], i));
    }
    sort(beams.begin(), beams.end(), CMP);
    for(int i=0; i<beams.size(); i++)
        para_table.lidar32_inpara.beam_order[i] = beams[i].second;
}

bool ALV_DATA::init_lidar32_para()
{
    string lidar32para_filename = para_table.para_base_dir + "/lidar32para_.ini";
    ifstream infile;
    stringstream sline;
    string line;

    for(int k = 0; k<36000; k++)
    {
        para_table.lidar32_inpara.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar32_inpara.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }

    infile.open(lidar32para_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open hdl32 para file \""<<lidar32para_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Angle_V")
            {
                for(int i=0; i<32; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_inpara.angleV[i];

                    para_table.lidar32_inpara.sin_angleV[i] = (float)sin(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                    para_table.lidar32_inpara.cos_angleV[i] = (float)cos(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                }
                arrange_beam_order();
            }
            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_expara.R[i][0]>>para_table.lidar32_expara.R[i][1]>>para_table.lidar32_expara.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");
                sline.clear();
                sline<<line;
                sline>>para_table.lidar32_expara.T[0]>>para_table.lidar32_expara.T[1]>>para_table.lidar32_expara.T[2];
            }
        }
        line.clear();
    }
    infile.close();
    return true;
}


bool ALV_DATA::init_lidar16_para()
{
    /* init intrinsic para table
     *
    */    
    string lidar16para_L_filename = para_table.para_base_dir + "/lidar16para_L.ini";
    string lidar16para_R_filename = para_table.para_base_dir + "/lidar16para_R.ini";
    ifstream infile;
    stringstream sline;
    string line;

    // left
    for(int k = 0; k<36000; k++)
    {
        para_table.lidar16_inpara_L.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar16_inpara_L.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }

    infile.open(lidar16para_L_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open vlp16-L para file \""<<lidar16para_L_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Angle_V")
            {
                for(int i=0; i<VLP16_BEAM_NUM; i++)
                {
                    line.clear();   // remember to clear!!!!!
                    getline(infile, line);
                    sline.str("");
                    sline.clear();  // remember to clear!!!!!
                    sline<<line;
                    sline>>para_table.lidar16_inpara_L.angleV[i];
                    para_table.lidar16_inpara_L.sin_angleV[i] = (float)sin(para_table.lidar16_inpara_L.angleV[i]/180.0*M_PI);
                    para_table.lidar16_inpara_L.cos_angleV[i] = (float)cos(para_table.lidar16_inpara_L.angleV[i]/180.0*M_PI);
                }
            }

            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar16_expara_L.R[i][0]>>para_table.lidar16_expara_L.R[i][1]>>para_table.lidar16_expara_L.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");  // remember to clear!!!!!
                sline.clear();
                sline<<line;
                sline>>para_table.lidar16_expara_L.T[0]>>para_table.lidar16_expara_L.T[1]>>para_table.lidar16_expara_L.T[2];
                break;
            }
        }
        line.clear();
    }
    infile.close();

    // right
    for(int k = 0; k<36000; k++)
    {
        para_table.lidar16_inpara_R.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar16_inpara_R.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }
    infile.open(lidar16para_R_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open vlp16-R para file \""<<lidar16para_R_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }

            if(flag == "Angle_V")
            {
                for(int i=0; i<VLP16_BEAM_NUM; i++)
                {
                    line.clear();   // remember to clear!!!!!
                    getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar16_inpara_R.angleV[i];
                    para_table.lidar16_inpara_R.sin_angleV[i] = (float)sin(para_table.lidar16_inpara_R.angleV[i]/180.0*M_PI);
                    para_table.lidar16_inpara_R.cos_angleV[i] = (float)cos(para_table.lidar16_inpara_R.angleV[i]/180.0*M_PI);
                }
            }
            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");  // remember to clear!!!!!
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar16_expara_R.R[i][0]>>para_table.lidar16_expara_R.R[i][1]>>para_table.lidar16_expara_R.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");  // remember to clear!!!!!
                sline.clear();
                sline<<line;
                sline>>para_table.lidar16_expara_R.T[0]>>para_table.lidar16_expara_R.T[1]>>para_table.lidar16_expara_R.T[2];
            }

        }
        line.clear();
    }
    infile.close();


    return true;
}

bool ALV_DATA::init_lidar4_para()
{
    string lidar4para_filename = para_table.para_base_dir + "/lidar4para.ini";
    ifstream infile;
    stringstream sline;
    string line;


    infile.open(lidar4para_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open ibeo4 para file \""<<lidar4para_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.ibeo4_expara.R[i][0]>>para_table.ibeo4_expara.R[i][1]>>para_table.ibeo4_expara.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");
                sline.clear();
                sline<<line;
                sline>>para_table.ibeo4_expara.T[0]>>para_table.ibeo4_expara.T[1]>>para_table.ibeo4_expara.T[2];
            }
        }
        line.clear();
    }
    infile.close();
    return true;
}

// read from a single configure file line, determine which type the parameter is,
// and get the value of the parameter
void ReadConfigLine(const string line, string &flag, int &value)
{
    flag.clear();
    value = 0;
    for(auto c : line)
    {
        if(isalpha(c) || c == '_')
            flag = flag + c;
        if(c == '=' || c == ' ')
            continue;
        if(isdigit(c))
            value = value*10+(c-'0');
    }
}
/* init extrinsic para table
*/
bool ALV_DATA::read_alv_config()
{
    ifstream infile;
    stringstream sline;
    string line;

    string alv_config_filename = para_table.para_base_dir + "/alv_config.conf";
    infile.open(alv_config_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open config file \""<<alv_config_filename<<"\""<<endl;
        return false;
    }

    line.clear();
    while(getline(infile, line))
    {
        if(line[0] == '#' || line[0] == '[' || line.empty())
            continue;
        else
        {
            string flag;
            int value;
            ReadConfigLine(line, flag, value);
            if(flag == "map_front_range")
                para_table.map_range_front = value;
            else if (flag == "map_rear_range")
                para_table.map_range_rear = value;
            else if (flag == "map_left_range")
                para_table.map_range_left = value;
            else if (flag == "map_right_range")
                para_table.map_range_right = value;
            else if (flag == "blind_area_front")
                para_table.blind_area_front = value;
            else if (flag == "blind_area_rear")
                para_table.blind_area_rear = value;
            else if (flag == "blind_area_left")
                para_table.blind_area_left = value;
            else if (flag == "blind_area_right")
                para_table.blind_area_right = value;
            else if (flag == "obstacle_threshold")
                para_table.obs_threshold = value;
            else if(flag == "near_obstacle_threshold")
                para_table.near_obstacle_threshold = value;
            else if(flag == "weak_flatness_threshold")
                para_table.weak_flatness_threshold = value;
            else if(flag == "suspend_obs_threshold")
                para_table.suspend_obs_ths = value;
            else if (flag == "grid_size")
                para_table.grid_size = value;
            else if (flag == "car_length")
                para_table.car_length = value;
            else if (flag == "car_width")
                para_table.car_width = value;
            else if (flag == "dangerous_area_threshold")
                para_table.dangerous_area_threshold = value;
        }
        line.clear();
    }
    infile.close();

    para_table.grid_center_row = para_table.map_range_rear / para_table.grid_size;
    para_table.grid_center_col = para_table.map_range_left / para_table.grid_size;

    return true;
}

bool ALV_DATA::setup()
{
    if(!init_para())
        return false;

    assert(para_table.map_range_front > 0);
    assert(para_table.map_range_rear > 0);
    assert(para_table.map_range_left > 0);
    assert(para_table.map_range_right > 0);
    assert(para_table.grid_size > 0);
    para_table.grid_rows = (para_table.map_range_front + para_table.map_range_rear) / para_table.grid_size;
    para_table.grid_cols = (para_table.map_range_left + para_table.map_range_right) / para_table.grid_size;
    grid_map.setup(para_table.grid_rows, para_table.grid_cols);

    cv::Mat color_map = cv::Mat::zeros(para_table.grid_rows, para_table.grid_cols, CV_8UC3);
    result = color_map;

    init_lookup_table();
    return true;
}

void ALV_DATA::cleanup()
{
    memset(lidar16_pointcloud_L, 0, sizeof(alv_Point3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);
    memset(lidar16_pointcloud_R, 0, sizeof(alv_Point3f)*VLP16_BEAM_NUM*VLP16_BEAM_POINTSIZE);
    memset(lidar32_pointcloud, 0, sizeof(alv_Point3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    memset(lidar4_Data, 0, sizeof(LIDAR4_DATA));
    memset(lidar4_pointcloud, 0, sizeof(alv_Point3f)*LIDAR4_BEAM_NUM*LIDAR4_BEAM_POINTSIZE);
    for(int i=0; i<GRIDMAP_HEIGHT; i++)
        memset(lidar1_grids[i], 0, sizeof(unsigned char)*GRIDMAP_WIDTH);

    memset(para_table.lidar16_inpara_L.beam_point_num, 0, sizeof(int)*VLP16_BEAM_NUM);
    memset(para_table.lidar16_inpara_R.beam_point_num, 0, sizeof(int)*VLP16_BEAM_NUM);
    grid_map.cleanup();
    for(int beam=0; beam<POLAR_BEAM_NUM_2; beam++)
    {
        for(int angle = 0; angle < POLAR_ANGLE_NUM_2; angle++)
            polar_grids[beam][angle].cleanup();
    }

    // clean up picture
    int Rows = para_table.grid_rows;
    int Cols = para_table.grid_cols;
    for(int row = 0; row < Rows; row++)
        for(int col = 0; col < Cols; col++)
          result.at<Vec3b>(row, col) = Vec3b(0,0,0);      	// BGR, 黑
}

PyObject* ALV_DATA::show_result(char* name)
{
    GRID **grids = grid_map.grids;
    int Rows = para_table.grid_rows;
    int Cols = para_table.grid_cols;
    PyObject* python_result = PyList_New(0);
    PyList_Append(python_result,PyInt_FromLong(Rows));
    PyList_Append(python_result,PyInt_FromLong(Cols));
    for(int row = 0; row < Rows; row++)
    {
        for(int col = 0; col < Cols; col++)
        {
//            if(grids[row][col].attribute == GRID_TRAVESABLE)
//                result.at<Vec3b>(row, col) = Vec3b(50,50,50);      	// BGR
//            else if(grids[row][col].attribute == GRID_CAR_AREA)
//                result.at<Vec3b>(row, col) = Vec3b(255,255,255);   	// 白
//            else if(grids[row][col].attribute == GRID_NEG_OBS)
//                result.at<Vec3b>(row, col) = Vec3b(255,255,0);          // 浅蓝
//            else if(grids[row][col].attribute == GRID_WATER)
//                result.at<Vec3b>(row, col) = Vec3b(50,50,50);           // 灰
//            else
//                result.at<Vec3b>(row, col) = Vec3b(0,0,0);      	// BGR, 黑


            if(grids[row][col].attribute == GRID_UNKNOWN)
                result.at<Vec3b>(row, col) = Vec3b(0,0,0);      	// BGR, 黑
            else if(grids[row][col].attribute == GRID_CAR_AREA)
                result.at<Vec3b>(row, col) = Vec3b(255,255,255);   	// 白
            else if(grids[row][col].attribute == GRID_TRAVESABLE/* || grids[row][col].attribute == GRID_ESTIMATED_GROUND*/)
                result.at<Vec3b>(row, col) = Vec3b(50,50,50);   	// 灰
            else if(grids[row][col].attribute == GRID_POS_OBS)
            {
                result.at<Vec3b>(row, col) = Vec3b(0,0,255);     	// 红
                PyList_Append(python_result,PyInt_FromLong(row*Cols+col));
            }
            else if(grids[row][col].attribute == GRID_NEG_OBS)
                result.at<Vec3b>(row, col) = Vec3b(255,255,0);  	// 浅蓝
            else if(grids[row][col].attribute == GRID_DANGEROUS)
                result.at<Vec3b>(row, col) = Vec3b(0,255,255);  	// 黄
            else if(grids[row][col].attribute == GRID_ROAD_EDGE)
                result.at<Vec3b>(row, col) = Vec3b(255,0,255);  	// 紫
            else if(grids[row][col].attribute == GRID_OCCULUSION)
                result.at<Vec3b>(row, col) = Vec3b(255,0,0);  		// 蓝
            else if(grids[row][col].attribute == GRID_WATER)
                result.at<Vec3b>(row, col) = Vec3b(50,50,50); 	// 灰
            else if(grids[row][col].attribute == GRID_SUSPEND_OBS)
                result.at<Vec3b>(row, col) = Vec3b(0,255,0);    	// 绿

//            else if(grids[row][col].attribute == GRID_FRONT_EDGE)
//                result.at<Vec3b>(row, col) = Vec3b(0,255,255);    //
//            else if(grids[row][col].attribute == GRID_BACK_EDGE)
//                result.at<Vec3b>(row, col) = Vec3b(255,0,255);    //
//            else if(grids[row][col].attribute == GRID_FLAT)
//                result.at<Vec3b>(row, col) = Vec3b(100,100,200);    //
//            else if(grids[row][col].attribute == GRID_CORNER)
//                result.at<Vec3b>(row, col) = Vec3b(200,100,100);    //
        }
    }

    int offset = para_table.car_length/2;
    // 5 m
//    line(result, Point(0,para_table.grid_center_row+(offset+500)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+500)/para_table.grid_size), Scalar(150,150,150), 1, 4);
//    // 10 m
//    line(result, Point(0,para_table.grid_center_row+(offset+1000)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+1000)/para_table.grid_size), Scalar(150,150,150), 1, 4);
//    // 15 m
//    line(result, Point(0,para_table.grid_center_row+(offset+1500)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+1500)/para_table.grid_size), Scalar(150,150,150), 1, 4);
//    // 20 m
//    line(result, Point(0,para_table.grid_center_row+(offset+2000)/para_table.grid_size), Point(Cols-1,para_table.grid_center_row+(offset+2000)/para_table.grid_size), Scalar(150,150,150), 1, 4);

//    line(result, Point(0, 128), Point(Cols-1, 128), Scalar(100,100,100), 1);
//    line(result, Point(100,0), Point(100,Rows-1), Scalar(100,100,100), 1);

    int MatHight = para_table.grid_rows - 1;
    flip(result, result, 0);
    putText(result, "5m", Point(0, MatHight-(para_table.grid_center_row+(offset+500)/para_table.grid_size)),
            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));
    putText(result, "10m", Point(0, MatHight-(para_table.grid_center_row+(offset+1000)/para_table.grid_size)),
            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));
    putText(result, "15m", Point(0, MatHight-(para_table.grid_center_row+(offset+1500)/para_table.grid_size)),
            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));
    putText(result, "20m", Point(0, MatHight-(para_table.grid_center_row+(offset+2000)/para_table.grid_size)),
            cv::FONT_HERSHEY_COMPLEX, 0.5, Scalar(250,250,250));

    cv::Size dsize = cv::Size(2*Cols, 2*Rows);
    Mat dresult = Mat::zeros(dsize, CV_8UC3);
    resize(result, dresult, dsize);
    imshow(name, dresult);
    waitKey(1);
    return python_result;
}

void ALV_DATA::show_cloud()
{
    cv::Mat cloud = cv::Mat::zeros(600, 600, CV_8UC3);
    int row, col;
    for(size_t beam=0; beam<HDL32_BEAM_NUM; beam++)
    {
        for(size_t cnt = 0; cnt < HDL32_BEAM_POINTSIZE; cnt++)
        {
            const alv_Point3f &pt = lidar32_pointcloud[beam][cnt];
            if(pt.valid)
            {
                col = (pt.x+3000)/10;
                row = (pt.y+2000)/10;
                if(row>=0 && row <600 && col>=0 && col<600)
                     cloud.at<Vec3b>(row, col) = Vec3b(200,200,200);
            }
        }
    }
    for(size_t beam=0; beam<VLP16_BEAM_NUM; beam++)
    {
        for(size_t cnt = 0; cnt < VLP16_BEAM_POINTSIZE; cnt++)
        {
            const alv_Point3f &pt_L = lidar16_pointcloud_L[beam][cnt];
            if(pt_L.valid)
            {
                col = (pt_L.x+3000)/10;
                row = (pt_L.y+2000)/10;
                if(row>=0 && row <600 && col>=0 && col<600)
                {
                    cloud.at<Vec3b>(row, col) = Vec3b(200,200,200);
//                    if(pt_L.attribute == PT_FLAT)
//                        cloud.at<Vec3b>(row, col) = Vec3b(255,0,0); // BGR
//                    else if(pt_L.attribute == PT_LESS_FLAT)
//                        cloud.at<Vec3b>(row, col) = Vec3b(0,255,0);
//                    else if(pt_L.attribute == PT_SHARP)
//                        cloud.at<Vec3b>(row, col) = Vec3b(0,0,255);
//                    else
//                        cloud.at<Vec3b>(row, col) = Vec3b(0,255,85);
                }

            }
            const alv_Point3f &pt_R = lidar16_pointcloud_R[beam][cnt];
            if(pt_R.valid)
            {
                col = (pt_R.x+3000)/10;
                row = (pt_R.y+2000)/10;
                if(row>=0 && row <600 && col>=0 && col<600)
                {
                    cloud.at<Vec3b>(row, col) = Vec3b(200,200,200);
//                    if(pt_R.attribute == PT_FLAT)
//                        cloud.at<Vec3b>(row, col) = Vec3b(255,0,0);
//                    else if(pt_R.attribute == PT_LESS_FLAT)
//                        cloud.at<Vec3b>(row, col) = Vec3b(0,255,0);
//                    else if(pt_R.attribute == PT_SHARP)
//                        cloud.at<Vec3b>(row, col) = Vec3b(0,0,255);
//                    else
//                        cloud.at<Vec3b>(row, col) = Vec3b(0,170,255);
                }
            }
        }
    }
    flip(cloud, cloud, 0);
    imshow("cloud", cloud);
    waitKey(1);
}

bool ALV_DATA::parse_16_process(u_int8_t *cache_L, u_int8_t *cache_R)
{
    /* parse left point cloud
     * 参考VLP16雷达用户手册
    */
    int packet_cnt = 0;
    u_int8_t *fp = cache_L + VLP16_PACKET_SIZE*packet_cnt;
    u_int8_t *fq = fp + 1;
    u_int16_t val = (*fp)*256 + *fq;

    float *cos_angH_L = para_table.lidar16_inpara_L.cos_angleH;
    float *sin_angH_L = para_table.lidar16_inpara_L.sin_angleH;
    float *cos_angV_L = para_table.lidar16_inpara_L.cos_angleV;
    float *sin_angV_L = para_table.lidar16_inpara_L.sin_angleV;
    float *angleV_L = para_table.lidar16_inpara_L.angleV;

    int idx1 = 0;
    int idx2 = VLP16_BEAM_POINTSIZE/4;
    int idx3 = VLP16_BEAM_POINTSIZE/4*3-1;
    int idx4 = VLP16_BEAM_POINTSIZE-1;
    int idx;
    while(val == 0xffee)
    {
        // parse 1206 bytes
        u_int32_t angleH[24];
        u_int8_t *pAngL = fq + 1;
        u_int8_t *pAngH = pAngL + 1;
        for(int i=0; i<12; i++)                     // read the actual angle
        {
            angleH[2*i] = (*pAngH)*256 + (*pAngL);  // little endian mode
            pAngL += 100;
            pAngH += 100;
        }
        for(int i=0; i<11; i++)                     // interpolate angle
        {
            if(angleH[2*i] < angleH[2*i+2])
                angleH[2*i+1] = ((angleH[2*i] + angleH[2*i+2])/2) % 36000;
            else
            {
                angleH[2*i+1] = ((angleH[2*i] + 36000 + angleH[2*i+2])/2) % 36000;
            }
        }
        if(angleH[20] < angleH[22])
            angleH[23] = (angleH[22] + (angleH[22] - angleH[20])/2) % 36000;
        else
            angleH[23] = (angleH[22] + (angleH[22] + 36000 - angleH[20])/2) % 36000;


        // generate point position
        for(int i=0; i<12; i++)
        {
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;

            // 车前方由近到远按照下标由小到大存储在数组前半部分，后方由近及远按照同理存在数组后半部分，这样是为了检测凹障碍方便
            if(angleH[2*i] >= 0 && angleH[2*i] <= 9000)             // 0~90
            {
                idx = idx2++;
                if(idx >= VLP16_BEAM_POINTSIZE/2)
                    idx = VLP16_BEAM_POINTSIZE/2-1;
            }
            else if(angleH[2*i] >= 9000 && angleH[2*i] <= 18000)    // 180~90
            {
                idx = idx4--;
                if(idx < VLP16_BEAM_POINTSIZE/4*3)
                    idx = VLP16_BEAM_POINTSIZE/4*3;
            }
            else if(angleH[2*i] >= 18000 && angleH[2*i] <= 27000)   // 270~180
            {
                idx = idx3--;
                if(idx < VLP16_BEAM_POINTSIZE/2)
                    idx = VLP16_BEAM_POINTSIZE/2;
            }
            else                                                    // 270~360
            {
                idx = idx1++;
                if(idx >= VLP16_BEAM_POINTSIZE/4)
                    idx = VLP16_BEAM_POINTSIZE/4-1;
            }

            for(int j=0; j<VLP16_BEAM_NUM; j++)
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i] >= VLP16_INVALID_ANGLE_LL && angleH[2*i] <= VLP16_INVALID_ANGLE_LH))
                {
                    lidar16_pointcloud_L[j][idx].valid = true;
                    lidar16_pointcloud_L[j][idx].distance = distance;
                    lidar16_pointcloud_L[j][idx].intensity = *pVal;
                    lidar16_pointcloud_L[j][idx].angleH = angleH[2*i];
                    lidar16_pointcloud_L[j][idx].angleV = int16_t(angleV_L[j]*100);
                    lidar16_pointcloud_L[j][idx].x = (float)distance/10.0 * cos_angV_L[j] * sin_angH_L[angleH[2*i]];
                    lidar16_pointcloud_L[j][idx].y = (float)distance/10.0 * cos_angV_L[j] * cos_angH_L[angleH[2*i]];
                    lidar16_pointcloud_L[j][idx].z = (float)distance/10.0 * sin_angV_L[j];
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }

            // 车前方由近到远按照下标由小到大存储在数组前半部分，后方由近及远按照同理存在数组后半部分，这样是为了检测凹障碍方便
            if(angleH[2*i+1] >= 0 && angleH[2*i+1] <= 9000)             // 0~90
            {
                idx = idx2++;
                if(idx >= VLP16_BEAM_POINTSIZE/2)
                    idx = VLP16_BEAM_POINTSIZE/2-1;
            }
            else if(angleH[2*i+1] >= 9000 && angleH[2*i+1] <= 18000)    // 180~90
            {
                idx = idx4--;
                if(idx < VLP16_BEAM_POINTSIZE/4*3)
                    idx = VLP16_BEAM_POINTSIZE/4*3;
            }
            else if(angleH[2*i+1] >= 18000 && angleH[2*i+1] <= 27000)   // 270~180
            {
                idx = idx3--;
                if(idx < VLP16_BEAM_POINTSIZE/2)
                    idx = VLP16_BEAM_POINTSIZE/2;
            }
            else                                                    // 270~360
            {
                idx = idx1++;
                if(idx >= VLP16_BEAM_POINTSIZE/4)
                    idx = VLP16_BEAM_POINTSIZE/4-1;
            }

            for(int j=0; j<VLP16_BEAM_NUM; j++)
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i+1] >= VLP16_INVALID_ANGLE_LL && angleH[2*i+1] <= VLP16_INVALID_ANGLE_LH))
                {
                    lidar16_pointcloud_L[j][idx].valid = true;
                    lidar16_pointcloud_L[j][idx].distance = distance;
                    lidar16_pointcloud_L[j][idx].intensity = *pVal;
                    lidar16_pointcloud_L[j][idx].angleH = angleH[2*i+1];
                    lidar16_pointcloud_L[j][idx].angleV = int16_t(angleV_L[j]*100);
                    lidar16_pointcloud_L[j][idx].x = (float)distance/10.0 * cos_angV_L[j] * sin_angH_L[angleH[2*i+1]];
                    lidar16_pointcloud_L[j][idx].y = (float)distance/10.0 * cos_angV_L[j] * cos_angH_L[angleH[2*i+1]];
                    lidar16_pointcloud_L[j][idx].z = (float)distance/10.0 * sin_angV_L[j];
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > VLP16_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache_L + VLP16_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }


    /* parse right point cloud
     *
    */
    packet_cnt = 0;
    fp = cache_R + VLP16_PACKET_SIZE*packet_cnt;
    fq = fp + 1;
    val = (*fp)*256 + *fq;

    float *cos_angH_R = para_table.lidar16_inpara_R.cos_angleH;
    float *sin_angH_R = para_table.lidar16_inpara_R.sin_angleH;
    float *cos_angV_R = para_table.lidar16_inpara_R.cos_angleV;
    float *sin_angV_R = para_table.lidar16_inpara_R.sin_angleV;
    float *angleV_R = para_table.lidar16_inpara_R.angleV;

    idx1 = VLP16_BEAM_POINTSIZE/4-1;
    idx2 = VLP16_BEAM_POINTSIZE/2-1;
    idx3 = VLP16_BEAM_POINTSIZE/2;
    idx4 = VLP16_BEAM_POINTSIZE/4*3;

    while(val == 0xffee)
    {
        // parse 1206 bytes
        u_int32_t angleH[24];
        u_int8_t *pAngL = fq + 1;
        u_int8_t *pAngH = pAngL + 1;
        for(int i=0; i<12; i++)                     // read the actual angle
        {
            angleH[2*i] = (*pAngH)*256 + (*pAngL);  // little endian mode
            pAngL += 100;
            pAngH += 100;
        }
        for(int i=0; i<11; i++)                     // interpolate angle
        {
            if(angleH[2*i] < angleH[2*i+2])
                angleH[2*i+1] = ((angleH[2*i] + angleH[2*i+2])/2) % 36000;
            else
            {
                angleH[2*i+1] = ((angleH[2*i] + 36000 + angleH[2*i+2])/2) % 36000;
            }
        }
        if(angleH[20] < angleH[22])
            angleH[23] = (angleH[22] + (angleH[22] - angleH[20])/2) % 36000;
        else
            angleH[23] = (angleH[22] + (angleH[22] + 36000 - angleH[20])/2) % 36000;


        // generate point position
        for(int i=0; i<12; i++)
        {
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;

            // 将每一根线的点按照方向分成4部分，分别是[车前由朝地到水平的90度范围，车前水平到朝上的90度范围，车后朝地到水平的90度范围，车后水平到朝上的90度范围]
            if(angleH[2*i] >= 0 && angleH[2*i] <= 9000)             // 90~0
            {
                idx = idx1--;
                if(idx < 0)
                    idx = 0;
            }
            else if(angleH[2*i] >= 9000 && angleH[2*i] <= 18000)    // 90~180
            {
                idx = idx3++;
                if(idx >= VLP16_BEAM_POINTSIZE/4*3)
                    idx = VLP16_BEAM_POINTSIZE/4*3-1;
            }
            else if(angleH[2*i] >= 18000 && angleH[2*i] <= 27000)   // 180~270
            {
                idx = idx4++;
                if(idx >= VLP16_BEAM_POINTSIZE)
                    idx = VLP16_BEAM_POINTSIZE-1;
            }
            else                                                    // 360~270
            {
                idx = idx2--;
                if(idx < VLP16_BEAM_POINTSIZE/4)
                    idx = VLP16_BEAM_POINTSIZE/4;
            }
            for(int j=0; j<VLP16_BEAM_NUM; j++)
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i] >= VLP16_INVALID_ANGLE_RL && angleH[2*i] <= VLP16_INVALID_ANGLE_RH))
                {
                    lidar16_pointcloud_R[j][idx].valid = true;
                    lidar16_pointcloud_R[j][idx].distance = distance;
                    lidar16_pointcloud_R[j][idx].intensity = *pVal;
                    lidar16_pointcloud_R[j][idx].angleH = angleH[2*i];
                    lidar16_pointcloud_R[j][idx].angleV = int16_t(angleV_R[j]*100);
                    lidar16_pointcloud_R[j][idx].x = (float)distance/10.0 * cos_angV_R[j] * sin_angH_R[angleH[2*i]];
                    lidar16_pointcloud_R[j][idx].y = (float)distance/10.0 * cos_angV_R[j] * cos_angH_R[angleH[2*i]];
                    lidar16_pointcloud_R[j][idx].z = (float)distance/10.0 * sin_angV_R[j];
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }

            // 车前方由近到远按照下标由小到大存储在数组前半部分，后方由近及远按照同理存在数组后半部分，这样是为了检测凹障碍方便
            if(angleH[2*i+1] >= 0 && angleH[2*i+1] <= 9000)             // 90~0
            {
                idx = idx1--;
                if(idx < 0)
                    idx = 0;
            }
            else if(angleH[2*i+1] >= 9000 && angleH[2*i+1] <= 18000)    // 90~180
            {
                idx = idx3++;
                if(idx >= VLP16_BEAM_POINTSIZE/4*3)
                    idx = VLP16_BEAM_POINTSIZE/4*3-1;
            }
            else if(angleH[2*i+1] >= 18000 && angleH[2*i+1] <= 27000)   // 180~270
            {
                idx = idx4++;
                if(idx >= VLP16_BEAM_POINTSIZE)
                    idx = VLP16_BEAM_POINTSIZE-1;
            }
            else                                                        // 360~270
            {
                idx = idx2--;
                if(idx < VLP16_BEAM_POINTSIZE/4)
                    idx = VLP16_BEAM_POINTSIZE/4;
            }
            for(int j=0; j<VLP16_BEAM_NUM; j++)
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= VLP16_VALID_RADIUS_L && !(angleH[2*i+1] >= VLP16_INVALID_ANGLE_RL && angleH[2*i+1] <= VLP16_INVALID_ANGLE_RH))
                {
                    lidar16_pointcloud_R[j][idx].valid = true;
                    lidar16_pointcloud_R[j][idx].distance = distance;
                    lidar16_pointcloud_R[j][idx].intensity = *pVal;
                    lidar16_pointcloud_R[j][idx].angleH = angleH[2*i+1];
                    lidar16_pointcloud_R[j][idx].angleV = int16_t(angleV_R[j]*100);
                    lidar16_pointcloud_R[j][idx].x = (float)distance/10.0 * cos_angV_R[j] * sin_angH_R[angleH[2*i+1]];
                    lidar16_pointcloud_R[j][idx].y = (float)distance/10.0 * cos_angV_R[j] * cos_angH_R[angleH[2*i+1]];
                    lidar16_pointcloud_R[j][idx].z = (float)distance/10.0 * sin_angV_R[j];
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > VLP16_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache_R + VLP16_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }
    return true;
}

bool ALV_DATA::parse_16data_online(LIDAR16_MSG *lidar16_msg)
{
    u_int8_t *cache_L = new u_int8_t[VLP16_BUFFER_SIZE];
    u_int8_t *cache_R = new u_int8_t[VLP16_BUFFER_SIZE];
    memcpy(cache_L, lidar16_msg->frame_dataL, VLP16_BUFFER_SIZE);
    memcpy(cache_R, lidar16_msg->frame_dataR, VLP16_BUFFER_SIZE);

    parse_16_process(cache_L, cache_R);
    calib_16data();

    delete[] cache_L;
    delete[] cache_R;
    return true;
}

bool ALV_DATA::parse_16data_offline(const string filename_L, const string filename_R)
{
    char file_L[256];
    char file_R[256];
    strcpy(file_L, filename_L.c_str());
    strcpy(file_R, filename_R.c_str());
    FILE *f_L = fopen(file_L, "rb");
    FILE *f_R = fopen(file_R, "rb");

    if(f_L == nullptr || f_R == nullptr)
        return false;

    u_int8_t *cache_L = new u_int8_t[VLP16_BUFFER_SIZE];
    u_int8_t *cache_R = new u_int8_t[VLP16_BUFFER_SIZE];
    if(!fread(cache_L, sizeof(u_int8_t), VLP16_BUFFER_SIZE, f_L))
    {
        cerr<<"***Error: fread file "<<filename_L<<" failed!"<<endl;
        return false;
    }
    if(!fread(cache_R, sizeof(u_int8_t), VLP16_BUFFER_SIZE, f_R))
    {
        cout<<"***Error: fread file "<<filename_R<<" failed!"<<endl;
    }
    fclose(f_L);
    fclose(f_R);

    parse_16_process(cache_L, cache_R);
    calib_16data();

    delete[] cache_L;
    delete[] cache_R;
    return true;
}

void ALV_DATA::calib_16data()
{
    for(int beam=0; beam<VLP16_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<VLP16_BEAM_POINTSIZE; cnt++)
        {
            alv_Point3f *p = &lidar16_pointcloud_L[beam][cnt];
            alv_Point3f tmp = lidar16_pointcloud_L[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar16_expara_L.R[0][0]+tmp.y*para_table.lidar16_expara_L.R[0][1]+tmp.z*para_table.lidar16_expara_L.R[0][2] + para_table.lidar16_expara_L.T[0];
                p->y = tmp.x*para_table.lidar16_expara_L.R[1][0]+tmp.y*para_table.lidar16_expara_L.R[1][1]+tmp.z*para_table.lidar16_expara_L.R[1][2] + para_table.lidar16_expara_L.T[1];
                p->z = tmp.x*para_table.lidar16_expara_L.R[2][0]+tmp.y*para_table.lidar16_expara_L.R[2][1]+tmp.z*para_table.lidar16_expara_L.R[2][2] + para_table.lidar16_expara_L.T[2];
            }

            p = &lidar16_pointcloud_R[beam][cnt];
            tmp = lidar16_pointcloud_R[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar16_expara_R.R[0][0]+tmp.y*para_table.lidar16_expara_R.R[0][1]+tmp.z*para_table.lidar16_expara_R.R[0][2] + para_table.lidar16_expara_R.T[0];
                p->y = tmp.x*para_table.lidar16_expara_R.R[1][0]+tmp.y*para_table.lidar16_expara_R.R[1][1]+tmp.z*para_table.lidar16_expara_R.R[1][2] + para_table.lidar16_expara_R.T[1];
                p->z = tmp.x*para_table.lidar16_expara_R.R[2][0]+tmp.y*para_table.lidar16_expara_R.R[2][1]+tmp.z*para_table.lidar16_expara_R.R[2][2] + para_table.lidar16_expara_R.T[2];
            }
        }
    }

    for(int beam=0; beam<VLP16_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<VLP16_BEAM_POINTSIZE; cnt++)
        {
            alv_Point3f *p = &lidar16_pointcloud_L[beam][cnt];
            alv_Point3f tmp = lidar16_pointcloud_L[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }

            p = &lidar16_pointcloud_R[beam][cnt];
            tmp = lidar16_pointcloud_R[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }
        }
    }
}


// save lidar16 point cloud coordinates to text files
// each line corresponds to a point x, y, z, in cm.
void ALV_DATA::save_16pt_txt(const std::string &filename_L, const std::string &filename_R)
{
    fstream fileout;
    fileout.open(filename_L, ios::out);
    if(!fileout)
        cerr<<"***Error: Can't open file \""<<filename_L<<"\""<<endl;
    for(int beam = 0; beam < VLP16_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < VLP16_BEAM_POINTSIZE; angle++)
        {
            if(lidar16_pointcloud_L[beam][angle].valid)
            {
//                fileout << lidar16_pointcloud_L[beam][angle].x << "\t\t"<<
//                           lidar16_pointcloud_L[beam][angle].y << "\t\t"<<
//                           lidar16_pointcloud_L[beam][angle].z << endl;
                fileout << lidar16_pointcloud_L[beam][angle].x << "\t\t"<<
                           lidar16_pointcloud_L[beam][angle].y << "\t\t"<<
                           lidar16_pointcloud_L[beam][angle].z << "\t\t"<<
                           lidar16_pointcloud_L[beam][angle].z << "\t\t";
//                if(lidar16_pointcloud_L[beam][angle].neighbor_dis != INVALID_VAL)
//                    fileout<<lidar16_pointcloud_L[beam][angle].neighbor_dis<<"\t\t";
//                else
//                    fileout<<0<<"\t\t";
//                if(lidar16_pointcloud_L[beam][angle].flatness != INVALID_VAL)
//                    fileout<<lidar16_pointcloud_L[beam][angle].flatness<<"\t\t";
//                else
//                    fileout<<0;
                fileout<<endl;
            }
        }
    }
    fileout.close();

    fileout.open(filename_R, ios::out);
    if(!fileout)
        cerr<<"***Error: Can't open file \""<<filename_R<<"\""<<endl;
    for(int beam = 0; beam < VLP16_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < VLP16_BEAM_POINTSIZE; angle++)
        {
            if(lidar16_pointcloud_R[beam][angle].valid)
            {
//                fileout << lidar16_pointcloud_R[beam][angle].x << "\t\t"<<
//                           lidar16_pointcloud_R[beam][angle].y << "\t\t"<<
//                           lidar16_pointcloud_R[beam][angle].z << endl;
                fileout << lidar16_pointcloud_R[beam][angle].x << "\t\t"<<
                           lidar16_pointcloud_R[beam][angle].y << "\t\t"<<
                           lidar16_pointcloud_R[beam][angle].z << "\t\t"<<
                           lidar16_pointcloud_R[beam][angle].z << "\t\t";
//                if(lidar16_pointcloud_R[beam][angle].neighbor_dis != INVALID_VAL)
//                    fileout<<lidar16_pointcloud_R[beam][angle].neighbor_dis<<"\t\t";
//                else
//                    fileout<<0<<"\t\t";

//                if(lidar16_pointcloud_R[beam][angle].flatness != INVALID_VAL)
//                    fileout<<lidar16_pointcloud_R[beam][angle].flatness<<"\t\t";
//                else
//                    fileout<<0;
                fileout<<endl;
            }
        }
    }
    fileout.close();
}


/*
 * Lidar32 part
*/
bool ALV_DATA::parse_32_process(u_int8_t *cache)
{
    int packet_cnt = 0;
    u_int8_t *fp = cache + HDL32_PACKET_SIZE*packet_cnt;
    u_int8_t *fq = fp + 1;
    u_int16_t val = (*fp)*256 + *fq;

    float *cos_angH = para_table.lidar32_inpara.cos_angleH;
    float *sin_angH = para_table.lidar32_inpara.sin_angleH;
    float *cos_angV = para_table.lidar32_inpara.cos_angleV;
    float *sin_angV = para_table.lidar32_inpara.sin_angleV;
    float *angleV = para_table.lidar32_inpara.angleV;


    u_int16_t start_angleH = 1;	// 记录这一帧初始的水平向激光角度
    bool start_angleH_generated = false;
    while(val == 0xffee)
    {
        // parse packet
        for(int i=0; i<HDL32_NUM_SHOTS; i++)
        {
            u_int8_t *pAngL = fp + 100*i + 2;
            u_int8_t *pAngH = pAngL + 1;
            u_int16_t angleH = ((*pAngH)*256 + (*pAngL) + 9000)%36000;
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;
            int idx = packet_cnt*HDL32_NUM_SHOTS + i;

            for(int j=0; j<HDL32_BEAM_NUM; j++)
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= HDL32_VALID_RADIUS)
                {
                    lidar32_pointcloud[j][idx].valid = true;
                    lidar32_pointcloud[j][idx].distance = distance;
                    lidar32_pointcloud[j][idx].intensity = *pVal;
                    lidar32_pointcloud[j][idx].angleH = angleH;
                    lidar32_pointcloud[j][idx].angleV = int16_t(angleV[j]*100);
                    lidar32_pointcloud[j][idx].x = (float)distance/10.0 * cos_angV[j] * sin_angH[angleH];
                    lidar32_pointcloud[j][idx].y = (float)distance/10.0 * cos_angV[j] * cos_angH[angleH];
                    lidar32_pointcloud[j][idx].z = (float)distance/10.0 * sin_angV[j];
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > HDL32_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache + HDL32_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }
    return true;
}
/*
bool ALV_DATA::parse_32data_online(LIDAR32_MSG *lidar32_msg)
{
    u_int8_t *cache = new u_int8_t[HDL32_BUFFER_SIZE];
    memcpy(cache, lidar32_msg->frame_data, sizeof(u_int8_t)*HDL32_BUFFER_SIZE);
    parse_32_process(cache);
    calib_32data();
    delete[] cache;
    return true;
}

bool ALV_DATA::parse_32data_offline(const std::string filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    FILE *fp = fopen(file, "rb");
    if(fp==NULL)
        return false;

    u_int8_t *cache = new u_int8_t[HDL32_BUFFER_SIZE];
    if(!fread(cache, sizeof(u_int8_t), HDL32_BUFFER_SIZE, fp))
    {
        cerr<<"***Error: fread file "<<filename<<" failed!"<<endl;
        return false;
    }
    fclose(fp);

    parse_32_process(cache);
    calib_32data();
    delete[] cache;
    return true;
}
*/
//wangzhiqing parse data from ply
int num;
bool ALV_DATA::parse_32data_offline(const std::string filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    ifstream Lidar32(file);
    string temp;
    if(!Lidar32.is_open())
    {
        cout<<"***Error:fread file "<<file<<" failed"<<endl;
        return false;
    }
    while(getline(Lidar32,temp))
        if(temp=="end_header")
            break;
    int i=0;
    float x,y,z;
    while(!Lidar32.eof())
    {
        for(int j=0;j<HDL32_BEAM_NUM;j++)
        {
            lidar32_pointcloud[j][i].valid = true;
            lidar32_pointcloud[j][i].intensity = 255;
            lidar32_pointcloud[j][i].angleH = i;//useless
            lidar32_pointcloud[j][i].angleV = int16_t(j*100);//useless
            Lidar32>>x>>y>>z;
            lidar32_pointcloud[j][i].x = 100 * x;
            lidar32_pointcloud[j][i].y = -100 * y;
            lidar32_pointcloud[j][i].z = -100 * z;
            lidar32_pointcloud[j][i].distance = sqrt(x*x+y*y+z*z) * 1000;
        }
        i++;
    }
    num = i;
    cout<<"wangzhiqing "<<num<<endl;
    calib_32data();
    return true;
}
bool ALV_DATA::parse_data_online(float *point,int length)
{
    int i = 0;
    cout<<"wangzhiqing"<<endl;
    while(i<length)
    {
        for(int j=0;j<32;j++)
        {
            lidar32_pointcloud[j][i].valid = true;
            lidar32_pointcloud[j][i].intensity = 255;
            lidar32_pointcloud[j][i].angleH = i;//useless
            lidar32_pointcloud[j][i].angleV = int16_t(j*100);//useless

            lidar32_pointcloud[j][i].x = 100 * point[i];
            lidar32_pointcloud[j][i].y = -100 * point[i+1];
            lidar32_pointcloud[j][i].z = -100 * point[i+2];
            lidar32_pointcloud[j][i].distance = sqrt(point[i]*point[i]
                                                     +point[i+1]*point[i+1]
                    +point[i+2]*point[i+2]) * 1000;
        }
        i=i+1;
    }
//    cout<<lidar32_pointcloud[0][0].x<<" "<<lidar32_pointcloud[0][0].y<<" "<<lidar32_pointcloud[0][0].z<<" "<<endl;
    num = length;
    return true;
}

void ALV_DATA::calib_32data()
{
    for(int beam=0; beam<HDL32_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<num; cnt++)
        {
            alv_Point3f *p = &lidar32_pointcloud[beam][cnt];
            const alv_Point3f tmp = lidar32_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }
        }
    }
}

/*
void ALV_DATA::calib_32data()
{
    for(int beam=0; beam<HDL32_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<HDL32_BEAM_POINTSIZE; cnt++)
        {
            alv_Point3f *p = &lidar32_pointcloud[beam][cnt];
            const alv_Point3f tmp = lidar32_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }
        }
    }
}
*/
void ALV_DATA::save_32pt_txt(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < HDL32_BEAM_POINTSIZE; angle++)
        {
            if(lidar32_pointcloud[beam][angle].valid)
            {
//                fileout << lidar32_pointcloud[beam][angle].x << "\t\t"<<
//                           lidar32_pointcloud[beam][angle].y << "\t\t"<<
//                           lidar32_pointcloud[beam][angle].z << endl;
                fileout << lidar32_pointcloud[beam][angle].x << "\t\t"<<
                           lidar32_pointcloud[beam][angle].y << "\t\t"<<
                           lidar32_pointcloud[beam][angle].z << "\t\t"<<
                           lidar32_pointcloud[beam][angle].z << endl;
            }
        }
    }
    fileout.close();
}

bool ALV_DATA::parse_4data_offline(const std::string &filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    FILE *fp = fopen(file, "r");
    if(fp==NULL)
        return false;

    double rad1,rad2;
    double height;  //for carlibration

    int idx0 = 0;
    int idx1 = 0;
    int idx2 = 0;
    int idx3 = 0;
    for(int i=0;i<LIDAR4_MAX_POINT_SIZE;i++)
    {
        fscanf(fp, "%d %lf %d", &(lidar4_Data->origine_point[i].layer),
               &(lidar4_Data->origine_point[i].angle),
               &(lidar4_Data->origine_point[i].distance));

        if(lidar4_Data->origine_point[i].layer!=-1)
            lidar4_Data->length++;
        else
            break;

        switch(lidar4_Data->origine_point[i].layer)
        {
           case 0:
               /* minus 0.6 degree for pitch carlibration */
               rad1=(-1.2)/360.0*2*PI;
               height=-(lidar4_Data->origine_point[i].distance*sin(rad1));
               //printf("%d %f  %d  %f\n",origine_point[i].layer,-origine_point[i].angle,origine_point[i].distance,height);
            break;
           case 1:
               rad1=(-0.4)/360.0*2*PI;
               height=-(lidar4_Data->origine_point[i].distance*sin(rad1));
               //printf("%d %f  %d  %f\n",origine_point[i].layer,-origine_point[i].angle,origine_point[i].distance,height);
            break;
           case 2:
               rad1=(0.4)/360.0*2*PI;
            break;
           case 3:
               rad1=(1.2)/360.0*2*PI;
            break;
           default:
            break;
        }
        rad2=-(lidar4_Data->origine_point[i].angle/360.0*2*PI);  //convert to -rad

        // save to local data
        lidar4_Data->position_point[i].layer   =   lidar4_Data->origine_point[i].layer;
        lidar4_Data->position_point[i].posi_x  =   lidar4_Data->origine_point[i].distance*cos(rad1)*sin(rad2);
        lidar4_Data->position_point[i].posi_y  =   lidar4_Data->origine_point[i].distance*cos(rad1)*cos(rad2);
        lidar4_Data->position_point[i].posi_z  =   lidar4_Data->origine_point[i].distance*sin(rad1);


        // calibration and transfer
        lidar4_Data->carli_point[i].layer  =   lidar4_Data->position_point[i].layer;
        lidar4_Data->carli_point[i].x      =   lidar4_Data->position_point[i].posi_x;
        lidar4_Data->carli_point[i].y      =   lidar4_Data->position_point[i].posi_y;
        lidar4_Data->carli_point[i].z      =   lidar4_Data->position_point[i].posi_z;
        lidar4_Data->length++;

        switch(lidar4_Data->carli_point[i].layer)
        {
        case 0:
            if(idx0 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[0][idx0].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[0][idx0].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[0][idx0].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx0].flatness = 0;
            lidar4_pointcloud[0][idx0].valid = true;
            idx0++;
            break;
        case 1:
            if(idx1 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[1][idx1].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[1][idx1].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[1][idx1].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx1].flatness = 0;
            lidar4_pointcloud[1][idx1].valid = true;
            idx1++;
            break;
        case 2:
            if(idx2 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[2][idx2].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[2][idx2].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[2][idx2].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx2].flatness = 0;
            lidar4_pointcloud[2][idx2].valid = true;
            idx2++;
            break;
        case 3:
            if(idx3 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[3][idx3].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[3][idx3].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[3][idx3].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx3].flatness = 0;
            lidar4_pointcloud[3][idx3].valid = true;
            idx3++;
            break;
        default:
            break;
        }
    }

    calib_4data();
    return true;
}

bool ALV_DATA::parse_4data_online(LIDAR4_MSG *lidar4_Msg)
{
    double rad1,rad2;
    double height;  //for carlibration

//
    int idx0 = 0;
    int idx1 = 0;
    int idx2 = 0;
    int idx3 = 0;
    for(int i=0;i<LIDAR4_MAX_POINT_SIZE;i++)
    {
        lidar4_Data->origine_point[i].layer=lidar4_Msg->layer[i];
        //lidar4_Data->origine_point[i].echo=Lidar4_Data->echo[i];
        lidar4_Data->origine_point[i].angle=lidar4_Msg->angle[i];
        lidar4_Data->origine_point[i].distance=lidar4_Msg->distance[i];

        if(lidar4_Data->origine_point[i].layer!=-1)
            lidar4_Data->length++;
        else
            break;

        switch(lidar4_Data->origine_point[i].layer)
        {
           case 0:
               /* minus 0.6 degree for pitch carlibration */
               rad1=(-1.2-0.6)/360.0*2*PI;
               height=-(lidar4_Data->origine_point[i].distance*sin(rad1));
               //printf("%d %f  %d  %f\n",origine_point[i].layer,-origine_point[i].angle,origine_point[i].distance,height);
            break;
           case 1:
               rad1=(-0.4-0.6)/360.0*2*PI;
               height=-(lidar4_Data->origine_point[i].distance*sin(rad1));
               //printf("%d %f  %d  %f\n",origine_point[i].layer,-origine_point[i].angle,origine_point[i].distance,height);
            break;
           case 2:
               rad1=(0.4-0.6)/360.0*2*PI;
            break;
           case 3:
               rad1=(1.2-0.6)/360.0*2*PI;
            break;
           default:
            break;
        }
        rad2=-(lidar4_Data->origine_point[i].angle/360.0*2*PI);  //convert to -rad

        // save to local data
        lidar4_Data->position_point[i].layer   =   lidar4_Data->origine_point[i].layer;
        lidar4_Data->position_point[i].posi_x  =   lidar4_Data->origine_point[i].distance*cos(rad1)*sin(rad2);
        lidar4_Data->position_point[i].posi_y  =   lidar4_Data->origine_point[i].distance*cos(rad1)*cos(rad2);
        lidar4_Data->position_point[i].posi_z  =   lidar4_Data->origine_point[i].distance*sin(rad1);


        // calibration and transfer
        lidar4_Data->carli_point[i].layer  =   lidar4_Data->position_point[i].layer;
        lidar4_Data->carli_point[i].x      =   lidar4_Data->position_point[i].posi_x;
        lidar4_Data->carli_point[i].y      =   lidar4_Data->position_point[i].posi_y;
        lidar4_Data->carli_point[i].z      =   lidar4_Data->position_point[i].posi_z;
        lidar4_Data->length++;

        switch(lidar4_Data->carli_point[i].layer)
        {
        case 0:
            if(idx0 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[0][idx0].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[0][idx0].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[0][idx0].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx0].flatness = 0;
            lidar4_pointcloud[0][idx0].valid = true;
            idx0++;
            break;
        case 1:
            if(idx1 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[1][idx1].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[1][idx1].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[1][idx1].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx1].flatness = 0;
            lidar4_pointcloud[1][idx1].valid = true;
            idx1++;
            break;
        case 2:
            if(idx2 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[2][idx2].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[2][idx2].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[2][idx2].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx2].flatness = 0;
            lidar4_pointcloud[2][idx2].valid = true;
            idx2++;
            break;
        case 3:
            if(idx3 >= LIDAR4_BEAM_POINTSIZE)
                break;
            lidar4_pointcloud[3][idx3].x = lidar4_Data->carli_point[i].x;
            lidar4_pointcloud[3][idx3].y = lidar4_Data->carli_point[i].y;
            lidar4_pointcloud[3][idx3].z = lidar4_Data->carli_point[i].z;
            lidar4_pointcloud[3][idx3].flatness = 0;
            lidar4_pointcloud[3][idx3].valid = true;
            idx3++;
            break;
        default:
            break;
        }
    }

    calib_4data();
    return true;
}

void ALV_DATA::calib_4data()
{
    for(int beam=0; beam<LIDAR4_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<LIDAR4_BEAM_POINTSIZE; cnt++)
        {
            alv_Point3f *p = &lidar4_pointcloud[beam][cnt];
            alv_Point3f tmp = lidar4_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.ibeo4_expara.R[0][0]+tmp.y*para_table.ibeo4_expara.R[0][1]+tmp.z*para_table.ibeo4_expara.R[0][2] + para_table.ibeo4_expara.T[0];
                p->y = tmp.x*para_table.ibeo4_expara.R[1][0]+tmp.y*para_table.ibeo4_expara.R[1][1]+tmp.z*para_table.ibeo4_expara.R[1][2] + para_table.ibeo4_expara.T[1];
                p->z = tmp.x*para_table.ibeo4_expara.R[2][0]+tmp.y*para_table.ibeo4_expara.R[2][1]+tmp.z*para_table.ibeo4_expara.R[2][2] + para_table.ibeo4_expara.T[2];
            }
        }
    }

    for(int beam=0; beam<LIDAR4_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<LIDAR4_BEAM_POINTSIZE; cnt++)
        {
            alv_Point3f *p = &lidar4_pointcloud[beam][cnt];
            alv_Point3f tmp = lidar4_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }
        }
    }
}

void ALV_DATA::save_4pt_txt(const std::string filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < LIDAR4_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < LIDAR4_BEAM_POINTSIZE; angle++)
        {
            if(lidar4_pointcloud[beam][angle].valid)
            {
                fileout << lidar4_pointcloud[beam][angle].x << "\t\t"<<
                           lidar4_pointcloud[beam][angle].y << "\t\t"<<
                           lidar4_pointcloud[beam][angle].z << endl;
            }
        }
    }
    fileout.close();
}

bool ALV_DATA::parse_1data_offline(const std::string &filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    FILE *fp = fopen(file, "r");
    if(fp==NULL)
        return false;

    int cntL = 0, cntR = 0;
    fscanf(fp, "%d %d", &cntL, &cntR);
    int CNT = max(cntL, cntR);
    int rowL, colL, rowR, colR;

    for(int i=0; i<CNT; i++)
    {
        fscanf(fp, "%d %d %d %d", &colL, &rowL, &colR, &rowR);
        if(i < cntL)
            lidar1_grids[rowL][colL] = 255;
        if(i < cntR)
            lidar1_grids[rowR][colR] = 255;
    }
    return true;
}


bool ALV_DATA::parse_1data_online(LIDAR1_MSG *Lidar1_Data)
{
    for(int i = 0; i < Lidar1_Data->cntL; i++){
        // 0 col
        // 1 row
        int row = (int)Lidar1_Data->frame_dataL[i][1];
        int col = (int)Lidar1_Data->frame_dataL[i][0];
        lidar1_grids[row][col] = 255;
    }
    for(int i = 0; i < Lidar1_Data->cntR; i++){
        int row = (int)Lidar1_Data->frame_dataR[i][1];
        int col = (int)Lidar1_Data->frame_dataR[i][0];
        lidar1_grids[row][col] = 255;
    }
    return true;
}


float point_distance(const alv_Point3f &p1, const alv_Point3f &p2)
{
    return sqrt(pow(double(p1.x-p2.x), 2) + pow(double(p1.y-p2.y),2) + pow(double(p1.z-p2.z),2));
}


void ALV_DATA::save_grid_map(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int row = 0; row<para_table.grid_rows; row++)
        for(int col=0; col<para_table.grid_cols; col++)
            fileout<<col*100<<"\t"<<row*100<<"\t"
                  <<grid_map.grids[row][col].dis_height<<"\t"
                  <<grid_map.grids[row][col].dis_height<<endl;
    fileout.close();
}
extern "C"
{


    PyObject* tmp(float *a,int b,char *c){
        POSITIVE_DETECTOR *positive_detector = POSITIVE_DETECTOR::get_instance();
        ALV_DATA *lidar = new ALV_DATA;
        if(!lidar->setup())
        {
            exit(-1);
        }
        lidar->parse_data_online(a,b);
    positive_detector->detect(lidar);
    PyObject* python_result = lidar->show_result(c);
    return python_result;}
}
