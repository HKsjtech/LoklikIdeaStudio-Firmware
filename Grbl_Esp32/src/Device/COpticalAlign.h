#pragma once
#include "./Common.h"
#include <cstdint> 

class COpticalAlign {
private:

    // 统一单位为丝，1cm=10mm=1000s
    //巡边起始位置偏移量机型补偿：0-一代刻字机；1-二代刻字机
    const int cutting_machine_x[2] = {0, -1900};
    const int cutting_machine_y[2] = {0, -4300};//2代机-3300+3000-4000(进料加深对应)
    const int find_start_x_min_val= 0;//丝
    const int find_start_x_max_val= 6000;// x-500:x+3000
    const int find_start_y_min_val= 2000;//丝
    int find_start_offset_y= 0;//丝
    // X、Y最小坐标
    const int min_coordinate_x= -1750;
    const int min_coordinate_y= -4500;
    // 同边两点搜索相隔间距
    const int next_range_offset_y= 1000;//
    const int next_range_offset_x= 1000;//
    // 搜索开始起始
    const int range_min_val= 500;//x-500
    // 最大搜索范围
    const int range_max_val= 3000;// x-500:x+3000
    //标记范围宽度
    const int marker_width=18200; // 单位丝，宽度调整18700-500
    //标记范围高度
    const int marker_height=23935; // 单位丝，高度调整23985-50
    //标记P1.X偏移
    const int marker_offset_x=1150;//1150; // 单位丝
    //标记P1.Y偏移
    const int marker_offset_y=1250;//1250; // 单位丝
    //传感器X偏移
    const int sensor_offset_x=1380; // 单位丝35.26
    //传感器Y偏移
    const int sensor_offset_y=3480; // 单位丝
    //最后巡边结果xy偏移量机型补偿：0-一代刻字机；1-二代刻字机
    const float machine_offset_x[2] = {0, 9.7};
    const float machine_offset_y[2] = {0, 0.5};

    //物理坐标整体偏移
    const int m_coord_offset_x=125;  
    const int m_coord_offset_y=-120;  

public:
    std::array<SPointInfo, 24> cacleP(int taskindex);
    void reviseOffset(int taskindex,int param_index);
    SPointInfo findMarkByRange(SValRange range,int model,int taskindex);

    static SLineInfo cacleline_4_insert(SPointInfo p1, SPointInfo p2, SPointInfo p3, SPointInfo p4);
    static SLineInfo cacleline_2(SPointInfo p1, SPointInfo p2);

    static float caclenum_p4(SPointInfo p1, SPointInfo p2,SPointInfo p3, SPointInfo p4,SLineInfo L1);

    static SPointInfo cacle_FindPoins(SPointInfo p1, SPointInfo p2, SPointInfo p3, SPointInfo p4, SPointInfo p5, SPointInfo p6, SPointInfo p7, SPointInfo p8, SLineInfo L1, SLineInfo L2);
    
    static SLineInfo caclelines(SPointInfo p1, SPointInfo p2);
    static float cacleAngle(SPointInfo p1, SPointInfo p2, SPointInfo p3);
    int Maxarry1(int arry[]);
    int Maxarry2(int arry[]);
    int choosepoint(int ary[], int crest1, int crest2);
};
