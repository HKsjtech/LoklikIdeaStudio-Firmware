#include "../COpticalAlign.h"
#include "../CPointQueue.h"

#ifdef DEBUG_SIMULATOR
#    include <cmath>
#    include <chrono>
#    include <thread>
#else
#    include "../../Grbl.h"
#    include "../SJMotorCtrl.h"
#endif

//LT=(A1,A2,An)∩(B1,B2,Bn)
std::array<SPointInfo, 24> COpticalAlign::cacleP(int taskindex) {
    std::array<SPointInfo, 24> result = {
        SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false },
        SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false },
        SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false },
        SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false },
        SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false },
        SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false }, SPointInfo { 0, 0, false },
    };
    //计算B1点 LT水平线
    int        B1_y_min = find_start_offset_y + 400 + cutting_machine_y[MACHINE_GENERATION];//二代刻字机调试:巡边起始位置Y+3000;
    if (min_coordinate_y > B1_y_min){
        B1_y_min = min_coordinate_y;
    }
    int        B1_y_max = B1_y_min + range_max_val;
    int        B1_x = find_start_y_min_val + cutting_machine_x[MACHINE_GENERATION];  //从2MM位置开始
    SValRange  B1_range = { B1_y_min, B1_y_max, B1_x, FindDir::T2B };
    SPointInfo BP1 = this->findMarkByRange(B1_range, 2, taskindex);
    if (!BP1.status) {
        B1_range = { B1_y_min, B1_y_max, B1_x + 500, FindDir::T2B };
        BP1 = this->findMarkByRange(B1_range, 2, taskindex);
    }
    if (!BP1.status || BP1.y == 0) {
        B1_range = { B1_y_min, B1_y_max, B1_x + 1000, FindDir::T2B };
        BP1 = this->findMarkByRange(B1_range, 2, taskindex);
    }
    if (!BP1.status || BP1.y == 0) {
        return result;
    }
    result[0] = BP1;

    //计算B2点 LT水平线
    int        B2_y_min = BP1.y - range_min_val;
    if (min_coordinate_y > B2_y_min){
        B2_y_min = min_coordinate_y;
    }
    int        B2_y_max = BP1.y + range_min_val * 3;
    int        B2_x = BP1.x + (next_range_offset_x - 400); 
    SValRange  B2_range = { B2_y_min, B2_y_max, B2_x, FindDir::T2B };
    SPointInfo B2 = this->findMarkByRange(B2_range, 0, taskindex);
    if (!B2.status || B2.y == 0) {
        B2_range = { B2_y_min, B2_y_max, B2_x - 1500, FindDir::T2B };
        B2 = this->findMarkByRange(B2_range, 1, taskindex);
    }
    result[1] = B2;
    //计算B3点 LT水平线
    int        B3_y_min = B2_y_min;
    int        B3_y_max = BP1.y + range_min_val * 3;
    int        B3_x = BP1.x + (next_range_offset_x + 200);  
    SValRange  B3_range = { B3_y_min, B3_y_max, B3_x, FindDir::T2B };
    SPointInfo B3 = this->findMarkByRange(B3_range, 0, taskindex);
    if (!B3.status || B3.y == 0) {
        B3_range = { B3_y_min, B3_y_max, B3_x - 2500, FindDir::T2B };
        B3 = this->findMarkByRange(B3_range, 1, taskindex);
    }
    result[2] = B3;
    //计算B4点 LT水平线
    int        B4_y_min = B2_y_min;
    int        B4_y_max = BP1.y + range_min_val * 3;
    int        B4_x = BP1.x + (next_range_offset_x + 800);  
    SValRange  B4_range = { B4_y_min, B4_y_max, B4_x, FindDir::T2B };
    SPointInfo B4 = this->findMarkByRange(B4_range, 0, taskindex);
    if (!B4.status || B4.y == 0) {
        B4_range = { B4_y_min, B4_y_max, B4_x - 2500, FindDir::T2B };
        B4 = this->findMarkByRange(B4_range, 1, taskindex);
    }
    result[3] = B4;

    //计算A1点 LT竖线
    int        A1_x_min = find_start_x_min_val  + 400 + cutting_machine_x[MACHINE_GENERATION];
    if (min_coordinate_x > A1_x_min){
        A1_x_min = min_coordinate_x;
    }
    // int        A1_x_max = find_start_x_max_val;
    int        A1_x_max = A1_x_min + range_min_val * 4;
    int        A1_y = BP1.y + (next_range_offset_y - 300);  
    SValRange  A1_range = { A1_x_min, A1_x_max, A1_y, FindDir::L2R };
    SPointInfo A1 = this->findMarkByRange(A1_range, 0, taskindex);
    if (!A1.status || A1.y == 0) {
        A1_range = { A1_x_min, A1_x_max, A1_y + 500, FindDir::L2R };
        A1 = this->findMarkByRange(A1_range, 1, taskindex);
    }
    if (!A1.status || A1.y == 0) {
        return result;
    }
    result[4] = A1;
    //计算A2点
    int        A2_x_min = A1.x - range_min_val;
    if (min_coordinate_x > A2_x_min){
        A2_x_min = min_coordinate_x;
    }
    int        A2_x_max = A1.x + range_min_val * 3;
    int        A2_y = A1.y + (next_range_offset_y - 400);
    SValRange  A2_range = { A2_x_min, A2_x_max, A2_y, FindDir::L2R };
    SPointInfo A2 = this->findMarkByRange(A2_range, 0, taskindex);
    result[5] = A2;
    //计算A3点
    int        A3_x_min = A2_x_min;
    int        A3_x_max = A1.x + range_min_val * 3;
    int        A3_y = A1.y + (next_range_offset_y + 200);
    SValRange  A3_range = { A3_x_min, A3_x_max, A3_y, FindDir::L2R };
    SPointInfo A3 = this->findMarkByRange(A3_range, 0, taskindex);
    result[6] = A3;
    //计算A4点 LT竖线
    int        A4_x_min = A2_x_min;
    int        A4_x_max = A1.x + range_min_val * 3;
    int        A4_y = A1.y + (next_range_offset_y + 800);
    SValRange  A4_range = { A4_x_min, A4_x_max, A4_y, FindDir::L2R };
    SPointInfo A4 = this->findMarkByRange(A4_range, 0, taskindex);
    result[7] = A4;

    // grbl_sendf(CLIENT_ALL, "LT Point:OK!!!!!!!!!!!!!!!!!!! ");

    //计算E1点  RT横线
    int        E1_y_min = BP1.y - (range_min_val * 2);
    if (min_coordinate_y > E1_y_min){
        E1_y_min = min_coordinate_y;
    }
    int        E1_y_max = E1_y_min + (range_min_val * 5);
    int        E1_x = BP1.x + marker_width - 3200; 
    if (A1.x != 0)
    {
        E1_x = A1.x + marker_width - 2700;
    }
    
    SValRange  E1_range = { E1_y_min, E1_y_max, E1_x, FindDir::T2B };
    SPointInfo E1 = this->findMarkByRange(E1_range, 2, taskindex);
    if (!E1.status || E1.y == 0) {
        E1_range = { E1_y_min, E1_y_max, E1_x += 500, FindDir::T2B };
        E1 = this->findMarkByRange(E1_range, 2, taskindex);
    }
    if (!E1.status || E1.y == 0) {
        return result;
    }
    result[8] = E1;
    //计算E2点 
    int        E2_y_min = E1.y - range_min_val;
    if (min_coordinate_y > E2_y_min){
        E2_y_min = min_coordinate_y;
    }
    int        E2_y_max = E1.y + range_min_val * 3;
    int        E2_x = E1.x + (next_range_offset_x - 400);  
    SValRange  E2_range = { E2_y_min, E2_y_max, E2_x, FindDir::T2B };
    SPointInfo E2 = this->findMarkByRange(E2_range, 0, taskindex);
    result[9] = E2;
    //计算E3点 
    int        E3_y_min = E2_y_min;
    int        E3_y_max = E1.y + range_min_val * 3;
    int        E3_x = E1.x + (next_range_offset_x + 200); 
    SValRange  E3_range = { E3_y_min, E3_y_max, E3_x, FindDir::T2B };
    SPointInfo E3 = this->findMarkByRange(E3_range, 0, taskindex);
    result[10] = E3;
    //计算E4点  RT横线
    int        E4_y_min = E2_y_min;
    int        E4_y_max = E1.y + range_min_val * 3;
    int        E4_x = E1.x + (next_range_offset_x + 800); 
    SValRange  E4_range = { E4_y_min, E4_y_max, E4_x, FindDir::T2B };
    SPointInfo E4 = this->findMarkByRange(E4_range, 0, taskindex);
    result[11] = E4;

    //计算F1点  RT竖线
    int        F1_x_min = E1.x + (range_min_val * 3);
    int        F1_x_max = F1_x_min + (range_min_val * 5);
    int        F1_y = E1.y + next_range_offset_x - 300; 
    SValRange  F1_range = { F1_x_min, F1_x_max, F1_y, FindDir::R2L };
    SPointInfo F1 = this->findMarkByRange(F1_range, 1, taskindex);
    if (!F1.status || F1.y == 0) {
        F1_range = { F1_x_min, F1_x_max, F1_y + 500, FindDir::R2L };
        F1 = this->findMarkByRange(F1_range, 1, taskindex);
    }
    if (!F1.status || F1.y == 0) {
        return result;
    }
    int        F1s_x_min = F1.x - range_min_val;
    int        F1s_x_max = F1.x + (range_min_val * 3);
    int        F1s_y = F1.y; 
    SValRange  F1s_range = { F1s_x_min, F1s_x_max, F1s_y, FindDir::L2R };
    SPointInfo F1s = this->findMarkByRange(F1s_range, 1, taskindex);
    result[12] = F1s;
    //计算F2点
    int        F2_x_min = F1.x - range_min_val;
    int        F2_x_max = F1.x + (range_min_val * 3);
    int        F2_y = A2.y;
    SValRange  F2_range = { F2_x_min, F2_x_max, F2_y, FindDir::L2R };
    SPointInfo F2 = this->findMarkByRange(F2_range, 0, taskindex);
    result[13] = F2;
    //计算F3点
    int        F3_x_min = F1.x - range_min_val;
    int        F3_x_max = F1.x + (range_min_val * 3);
    int        F3_y = F1.y + next_range_offset_x + 200;
    SValRange  F3_range = { F3_x_min, F3_x_max, F3_y, FindDir::L2R };
    SPointInfo F3 = this->findMarkByRange(F3_range, 0, taskindex);
    result[14] = F3;
    //计算F4点  RT竖线
    int        F4_x_min = F1.x - range_min_val;
    int        F4_x_max = F1.x + (range_min_val * 3);
    int        F4_y = F1.y + next_range_offset_x + 800;
    SValRange  F4_range = { F4_x_min, F4_x_max, F4_y, FindDir::L2R };
    SPointInfo F4 = this->findMarkByRange(F4_range, 0, taskindex);
    result[15] = F4;

    // grbl_sendf(CLIENT_ALL, "RT Point:OK!!!!!!!!!!!!!!!!!!! ");

    //计算C1点  LB横线
    float        Scale_x_rough = 1.0;
    for (int i = 0; i < 4; i++){
        bool flag = false;
        for (int j = 0; j < 4; j++){
            if (result[i + 4].x != 0 && result[j + 12].x != 0){
                Scale_x_rough = (float)(result[j + 12].x - result[i + 4].x)/marker_width;
                flag = true;
                break;
            }
        }
        if (flag)
        break;
    }
    SLineInfo line_T_rough = cacleline_2(result[0], result[8]);
    int       offset_x_rough = (int)(line_T_rough.a * (marker_height * Scale_x_rough));

    int        C1_y_max = BP1.y + (marker_height * Scale_x_rough) + range_min_val * 2;
    int        C1_y_min = C1_y_max - range_min_val * 6;
    int        C1_x = BP1.x - offset_x_rough;
    SValRange  C1_range = { C1_y_min, C1_y_max, C1_x, FindDir::B2T };
    SPointInfo C1 = this->findMarkByRange(C1_range, 2, taskindex);
    if (!C1.status || C1.y == 0) {
        C1_range = { C1_y_min, C1_y_max, C1_x + 500, FindDir::B2T };
        C1 = this->findMarkByRange(C1_range, 2, taskindex);
    }
    if (!C1.status || C1.y == 0) {
        return result;
    }
    int        C1s_y_min = C1.y - range_min_val;
    int        C1s_y_max = C1.y + range_min_val * 3;
    int        C1s_x = C1.x;
    SValRange  C1s_range = { C1s_y_min, C1s_y_max, C1s_x, FindDir::T2B };
    SPointInfo C1s = this->findMarkByRange(C1s_range, 0, taskindex);
    result[16] = C1s;
    //计算C2点
    int        C2_y_min = C1.y - range_min_val;
    int        C2_y_max = C1.y + range_min_val * 3;
    int        C2_x = B2.x;
    SValRange  C2_range = { C2_y_min, C2_y_max, C2_x, FindDir::T2B };
    SPointInfo C2 = this->findMarkByRange(C2_range, 0, taskindex);
    result[17] = C2;
    //计算C3点
    int        C3_y_min = C1.y - range_min_val;
    int        C3_y_max = C1.y + range_min_val * 3;
    int        C3_x = C1.x + next_range_offset_y + 200;
    SValRange  C3_range = { C3_y_min, C3_y_max, C3_x, FindDir::T2B };
    SPointInfo C3 = this->findMarkByRange(C3_range, 0, taskindex);
    result[18] = C3;
    //计算C4点
    int        C4_y_min = C1.y - range_min_val;
    int        C4_y_max = C1.y + range_min_val * 3;
    int        C4_x = C1.x + next_range_offset_y + 800;  
    SValRange  C4_range = { C4_y_min, C4_y_max, C4_x, FindDir::T2B };
    SPointInfo C4 = this->findMarkByRange(C4_range, 0, taskindex);
    result[19] = C4;
    
    //计算D1点  BL竖线
    int        D1_x_min = A1.x - offset_x_rough - range_min_val - 200;
    if (min_coordinate_x > D1_x_min){
        D1_x_min = min_coordinate_x;
    }
    int        D1_x_max = D1_x_min + (range_min_val * 6);
    int        D1_y = C1.y - next_range_offset_x + 300;  
    SValRange  D1_range = { D1_x_min, D1_x_max, D1_y, FindDir::L2R };
    SPointInfo D1 = this->findMarkByRange(D1_range, 1, taskindex);
    if (!D1.status || D1.y == 0) {
        D1_range = { D1_x_min, D1_x_max, D1_y - 500, FindDir::L2R };
        D1 = this->findMarkByRange(D1_range, 1, taskindex);
    }
    if (!D1.status || D1.y == 0) {
        return result;
    }
    result[20] = D1;
    //计算D2点
    int        D2_x_min = D1.x - range_min_val;
    if (min_coordinate_x > D2_x_min){
        D2_x_min = min_coordinate_x;
    }
    int        D2_x_max = D1.x + range_min_val * 3;
    int        D2_y = D1.y - (next_range_offset_x - 400);
    SValRange  D2_range = { D2_x_min, D2_x_max, D2_y, FindDir::L2R };
    SPointInfo D2 = this->findMarkByRange(D2_range, 0, taskindex);
    result[21] = D2;
    //计算D3点
    int        D3_x_min = D2_x_min;
    int        D3_x_max = D1.x + range_min_val * 3;
    int        D3_y = D1.y - (next_range_offset_x + 200);
    SValRange  D3_range = { D3_x_min, D3_x_max, D3_y, FindDir::L2R };
    SPointInfo D3 = this->findMarkByRange(D3_range, 0, taskindex);
    result[22] = D3;
    //计算D4点
    int        D4_x_min = D2_x_min;
    int        D4_x_max = D1.x + range_min_val * 3;
    int        D4_y = D1.y - (next_range_offset_x + 800);
    SValRange  D4_range = { D4_x_min, D4_x_max, D4_y, FindDir::L2R };
    SPointInfo D4 = this->findMarkByRange(D4_range, 0, taskindex);
    result[23] = D4;
    if (-2 == taskindex)
    {
        for (int m = 0; m < 24; m++) {
            grbl_sendf(CLIENT_ALL, "result: %d,%d\r\n", result[m].x, result[m].y);
        }
    }
    return result;
}
SLineInfo COpticalAlign::cacleline_4_insert(SPointInfo p1, SPointInfo p2, SPointInfo p3, SPointInfo p4) {
    std::array<SPointInfo, 4> result = { p1, p2, p3, p4 };
    float                     a = 0.0;
    float                     b = 0.0;
    float                     flag = 0.0;
    int                       fonumd_x = 0;
    int                       fonumd_y = 0;
    SLineInfo                 line1 = { 0.0, 0.0, false };
    SLineInfo                 line2 = { 0.0, 0.0, false };
    int pointunm = 4;
    for (int i = 0; i < 4; i++) {
        if (result[i].y == 0) {
            pointunm -= 1;
        }
        for (int j = 0; j < 4; j++) {
            if (j > i + 1) {
                line2 = caclelines(result[i], result[j]);

                int fonumd_m1 = abs(result[i].x - result[j].x);
                if (fonumd_x == 0)
                    fonumd_x = fonumd_m1;
                if (fonumd_m1 > fonumd_x)
                    fonumd_x = fonumd_m1;

                int fonumd_m2 = abs(result[i].y - result[j].y);
                if (fonumd_y == 0)
                    fonumd_y = fonumd_m2;
                if (fonumd_m2 > fonumd_y)
                    fonumd_y = fonumd_m2;

                float flag1 = COpticalAlign::caclenum_p4(p1, p2, p3, p4, line2);
                // grbl_sendf(CLIENT_SERIAL, "distance:%f\r\n",flag1);
                if (flag == 0.0)
                    flag = flag1;
                if (flag1 <= flag) {
                    flag = flag1;
                    line1 = line2;
                }
            }
        }
    }
    if (fonumd_x <= 20) {
        line1.a = 0.0;
        line1.b = 0.0;
    }else if (fonumd_y <= 20)
    {
        line1.a = 0.0;
        line1.b = 0.0;//待优化
        if (p1.y == p2.y || p1.y == p3.y || p1.y == p4.y ) {
            line1.b = p1.y;
        }else if (p2.y == p3.y || p2.y == p4.y) {
            line1.b = p2.y;
        }else if (p3.y == p4.y) {
            line1.b = p3.y;
        }else if(pointunm != 0) {
            line1.b = (p1.y + p2.y + p3.y + p4.y) / pointunm;
        }else{
            line1.b = 0;
        }
    }
    
    return line1;
}


SLineInfo COpticalAlign::cacleline_2(SPointInfo p1, SPointInfo p2) {
    SLineInfo line = { 0.0, 0.0, false };

    // 两点相同，无法确定直线
    if (p1.x == p2.x && p1.y == p2.y) {
        return line;
    }

    if (p1.x == p2.x) {
        // 竖直线，斜率无穷大，设阈值为2600.0
        line.a = 2600.0;
        line.b = 0.0;
    } else if (p1.y == p2.y) {
        // 水平线
        line.a = 0.0;
        line.b = (float)p1.y;
    } else {
        // 一般情况: y = a*x + b
        line.a = (float)(p1.y - p2.y) / (float)(p1.x - p2.x);
        line.b = (float)p1.y - line.a * (float)p1.x;
    }

    line.status = true;
    return line;
}

SLineInfo COpticalAlign::caclelines(SPointInfo p1, SPointInfo p2) {
    SLineInfo line1 = { 0.0, 0.0, false };
    float     fsq = 0.0;
    if (p1.x == p2.x)
        fsq = 0.000001;//优化垂直情况下的误差
    else
        fsq = float(p2.x - p1.x);
    float as = float(p2.y - p1.y);
    line1.a = as / fsq;
    line1.b = float(p1.y) - float(p1.x) * line1.a;
    // grbl_sendf(CLIENT_SERIAL, "line1.a:%f,%f\r\n", line1.a,line1.b);
    return line1;
}

SPointInfo COpticalAlign::cacle_FindPoins(SPointInfo p1, SPointInfo p2, SPointInfo p3, SPointInfo p4, SPointInfo p5, 
                                            SPointInfo p6, SPointInfo p7, SPointInfo p8, SLineInfo L1, SLineInfo L2) {
    std::array<SPointInfo, 8> p1list = { p1, p2, p3, p4, p5, p6, p7, p8 };
    SLineInfo line1 = L1;
    SLineInfo line2 = L2;
    SPointInfo p_out = { 0, 0, false };
    if ((line1.a == 0.0 || abs(line1.a) < 0.0001) && (line2.a == 0.0 || abs(line2.a) > 2500)){
        int pointunm = 4;
        for (int m = 0; m < 4; m++) {
            if (p1list[m].x == 0) {
                pointunm -= 1;
            }
        }
        if (pointunm != 0) {
            if (p1list[0].y == p1list[1].y || p1list[0].y == p1list[2].y || p1list[0].y == p1list[3].y ) {
                p_out.y = p1list[0].y;
            }else if (p1list[1].y == p1list[2].y || p1list[1].y == p1list[3].y) {
                p_out.y = p1list[1].y;
            }else if (p1list[2].y == p1list[3].y) {
                p_out.y = p1list[2].y;
            }else {
                p_out.y = (p1list[0].y + p1list[1].y + p1list[2].y + p1list[3].y) / pointunm;
            }
        } else {
            p_out.x = 0.0;
            p_out.y = 0.0;
            // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 11\r\n");
            return p_out;
        }
        pointunm = 4;
        for (int m = 4; m < 8; m++) {
            if (p1list[m].x == 0) {
                pointunm -= 1;
            }
        }
        if (pointunm != 0) {
            if (p1list[4].x == p1list[5].x || p1list[4].x == p1list[6].x || p1list[4].x == p1list[7].x ) {
                p_out.x = p1list[4].x;
            }else if (p1list[5].x == p1list[6].x || p1list[5].x == p1list[7].x) {
                p_out.x = p1list[5].x;
            }else if (p1list[6].x == p1list[7].x) {
                p_out.x = p1list[7].x;
            }else {
                p_out.x = (p1list[4].x + p1list[5].x + p1list[6].x + p1list[7].x) / pointunm;
            }
            p_out.status = true;
            // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 01\r\n");
            return p_out;
        } else {
            p_out.x = 0.0;
            p_out.y = 0.0;
            // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 12\r\n");
            return p_out;
        }
    } else if ((line1.a == 0.0 || abs(line1.a) < 0.0001) && (line2.a != 0.0 && abs(line2.a) <= 2500)){
        int pointunm = 4;
        for (int m = 0; m < 4; m++) {
            if (p1list[m].x == 0) {
                pointunm -= 1;
            }
        }
        if (pointunm != 0) {
            if (p1list[0].y == p1list[1].y || p1list[0].y == p1list[2].y || p1list[0].y == p1list[3].y ) {
                p_out.y = p1list[0].y;
            }else if (p1list[1].y == p1list[2].y || p1list[1].y == p1list[3].y) {
                p_out.y = p1list[1].y;
            }else if (p1list[2].y == p1list[3].y) {
                p_out.y = p1list[2].y;
            }else {
                p_out.y = (p1list[0].y + p1list[1].y + p1list[2].y + p1list[3].y) / pointunm;
            }
        } else {
            p_out.x = 0.0;
            p_out.y = 0.0;
            // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 13\r\n");
            return p_out;
        }
        p_out.x = (p_out.y - line2.b)/line2.a;
        // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 02\r\n");
        p_out.status = true;
        return p_out;
    } else if ((line1.a != 0.0 && abs(line1.a) >= 0.0001) && (line2.a == 0.0 || abs(line2.a) > 2500)){
        int pointunm = 4;
        for (int m = 4; m < 8; m++) {
            if (p1list[m].x == 0) {
                pointunm -= 1;
            }
        }
        if (pointunm != 0) {
            if (p1list[4].x == p1list[5].x || p1list[4].x == p1list[6].x || p1list[4].x == p1list[7].x ) {
                p_out.x = p1list[4].x;
            }else if (p1list[5].x == p1list[6].x || p1list[5].x == p1list[7].x) {
                p_out.x = p1list[5].x;
            }else if (p1list[6].x == p1list[7].x) {
                p_out.x = p1list[7].x;
            }else {
                p_out.x = (p1list[4].x + p1list[5].x + p1list[6].x + p1list[7].x) / pointunm;
            }
        } else {
            p_out.x = 0.0;
            p_out.y = 0.0;
            // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 14\r\n");
            return p_out;
        }
        p_out.y = p_out.x * line1.a + line1.b;
        // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 03\r\n");
        // grbl_sendf(CLIENT_SERIAL,"result_false:p_out.x: %d,line1.a:%f,line1.b:%f\r\n",p_out.x,line1.a,line1.b);
        p_out.status = true;
        return p_out;
    }else if ((line1.a != 0.0 && abs(line1.a) >= 0.0001) && (line2.a != 0.0 && abs(line2.a) <= 2500)){
        if (line2.a - line1.a != 0.0){
            p_out.x = (line1.b - line2.b) / (line2.a - line1.a);
        } else {
            p_out.x = 0.0;
            p_out.y = 0.0;
            // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 15\r\n");
            return p_out;
        }
        p_out.y = p_out.x * line1.a + line1.b;
        // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 04\r\n");
        // grbl_sendf(CLIENT_SERIAL,"result_false:p_out.x: %d,line1.a:%f,line1.b:%f\r\n",p_out.x,line1.a,line1.b);
        p_out.status = true;
        return p_out;
    }
    // grbl_sendf(CLIENT_SERIAL,"result_false:p_out: 16\r\n");
    return p_out;
}

float COpticalAlign::caclenum_p4(SPointInfo p1, SPointInfo p2, SPointInfo p3, SPointInfo p4, SLineInfo L1) {
    std::array<SPointInfo, 4> result = { p1, p2, p3, p4 };
    float                     flag = 0.0;
    float             distance_max = 0.0;
    for (int i = 0; i < 4; i++) {
        // grbl_sendf(CLIENT_SERIAL, "(x,y):%d,%d\r\n", result[i].x,result[i].y);
        float distance = fabs(L1.a * float(result[i].x) - float(result[i].y) + L1.b) / sqrt(L1.a * L1.a + 1);
        flag += distance;
        if (0 == distance_max || distance_max < distance)
        {
            distance_max = distance;
        }
    }
    flag -= distance_max;
    return flag;
}

int COpticalAlign::Maxarry1(int arry[]) {
    int max_value = 0;
    int max_lot = 0;
    int i = 0;
    for (i = 0; i < 18; i++) {
        if (arry[i] > max_value) {
            //            printf("MAXVALUE:%d\r\n",max_value);
            max_value = arry[i];
            max_lot = i;
        }
    }
    return max_lot;
}

int COpticalAlign::Maxarry2(int arry[]) {
    int max_value = 0;
    int max_lot = 0;
    int i = 0;
    for (i = 0; i < 18; i++) {
        if (arry[35 - i] > max_value) {
            //            printf("MAXVALUE:%d\r\n",max_value);
            max_value = arry[35 - i];
            max_lot = 35 - i;
        }
    }
    return max_lot;
}

int COpticalAlign::choosepoint(int ary[], int crest1, int crest2) {
    int min_value = ary[crest1];
    int min = 0;
    for (int i = crest1; i < crest2; i++)
    {
        if (ary[i] < min_value) {
            min_value = ary[i];
            min = i;
        }
    }
    if ((ary[min] < ary[crest1] - 400 || (ary[min] < ary[crest1] - 100 && ary[min] > ary[0] + 400)) && 
        (ary[min] < ary[crest2] - 400 || (ary[min] < ary[crest2] - 100 && ary[min] > ary[0] + 400)) && 
        (crest2 - crest1) > 8 && (crest2 - crest1) < 26 ){
        return min;
    }
    else{
        return 0;
    }
}

float COpticalAlign::cacleAngle(SPointInfo p1, SPointInfo p2, SPointInfo p3) {
    int   x1 = (p1.x - p3.x);
    int   y1 = (p1.y - p3.y);
    int   x2 = (p2.x - p3.x);
    int   y2 = (p2.y - p3.y);
    float dot = (x1 * x2 + y1 * y2);
    float det = (x1 * y2 - y1 * x2);
    float angle = atan2(det, dot) / PI * 180;
    float ang = fmod(angle, 360.0);
    if (ang < 0.0)
        ang = 360.0 + ang;
    return ang;
}

SPointInfo COpticalAlign::findMarkByRange(SValRange range, int model, int taskindex) {
    SPointInfo result_point = { 0, 0, false };
    int        p = -1;
    int        start_traversal = -1;
    int        pointarray[36] = { 0 };
    int        a = 0;
    int        b = 0;
    int        c1 = 0;
    int        c2 = 0;
    int        min = 0;
    float*     parray = new float[10];
    int        pindex = 0;
    int        step_lenght = 0;
    // 光传感器容差范围,分为四个区间0:传感器异常，1:白纸，2:垫板，3:标记物
    int tolerance_range[4][2] = { { 0, 1 }, { 1, 2500 }, { 2500, 3000 }, { 3200, 5500 } };
    // 步长
    int step = 10;
    int stepwucx = 10;
    int stepwucy = 10;
    
    if (range.dir == FindDir::L2R || range.dir == FindDir::T2B) {
        stepwucx = -stepwucx;
        stepwucy = -stepwucy;
    }
    // 点队列
    CPointQueue pointQueue;
    while (true) {
        int tValue = analogRead(PIN_MENU_KEY);
        if (tValue < 3000) {
            break;
        }
        SPointInfo point = { 0, 0, true };
        // 初始化和递进
        if (range.dir == FindDir::L2R || range.dir == FindDir::T2B) {
            if (start_traversal == -1){
                start_traversal = 1;
                p = range.min;
            }
            p = p + step;
            if (p > range.max){
                break;
            }
        } else if (range.dir == FindDir::R2L || range.dir == FindDir::B2T) {
            if (start_traversal == -1){
                start_traversal = 1;
                p = range.max;
            }
            p = p - step;
            if (p < range.min){
                break;
            }
        }

        // 判断变化量是X或Y
        if (range.dir == FindDir::L2R || range.dir == FindDir::R2L) {
            point = { p, range.safe, true };
            stepwucy = 0;
        } else if (range.dir == FindDir::T2B || range.dir == FindDir::B2T) {
            point = { range.safe, p, true };
            stepwucx = 0;
        }

        int psval = 0;
#ifdef DEBUG_SIMULATOR
        //获取寻边传感器值
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        psval = js_seek_marker(point.x, point.y);
#else
        float asix_x = ((sensor_offset_x + point.x) * 0.01) + 0.5;
        float asix_y = ((sensor_offset_y + point.y) * 0.01) + 0.5;
        psval = SJMotorCtrl::SeekPointByXY(asix_x, asix_y);

        // 记录值
        parray[pindex] = psval;
        pindex++;
        if (pindex >= 10) {
            pindex = 0;
            //调试输出
            if (-2 == taskindex)
            {
                grbl_sendf(CLIENT_SERIAL,
                       "SEEKVAL:%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,\r\n",
                       parray[0],
                       parray[1],
                       parray[2],
                       parray[3],
                       parray[4],
                       parray[5],
                       parray[6],
                       parray[7],
                       parray[8],
                       parray[9]);
            }
        }

#endif

        //将识别内容位置记录到队列

        pointarray[a] = psval;
        step_lenght += 1;
        if (a == 35) {
            c1 = Maxarry1(pointarray);
            c2 = Maxarry2(pointarray);
        }

        if (c1 < 14 && pointarray[c1] > 400 && c2 > 22 && pointarray[c2] > 400){
            b = choosepoint(pointarray, c1, c2);
        }
        
        if (b != 0) {
            result_point.x = point.x + (stepwucx * b);
            result_point.y = point.y + (stepwucy * b);
            result_point.status = true;
            b = 0;
            break;
        }
        if (a == 35) {
            int temp = 0;
            for (int i = 0; i < 35; i++) {
                pointarray[i] = pointarray[i + 1];
            }
            pointarray[35] = temp;
            a = 34;
        }
        a++;
    }
    delete[] parray;
    pointQueue.clear();
    return result_point;
}

void COpticalAlign::reviseOffset(int taskindex, int param_index) {
    SAlignResult result = { 0, 0, 0, 0, 0, false };
#ifndef DEBUG_SIMULATOR
    SJMotorCtrl::SeekActionStart(5000);
#endif
    find_start_offset_y = 0;
    std::array<SPointInfo, 24> p1list = this->cacleP(taskindex);

    if (p1list[0].status && p1list[4].status && p1list[8].status && 
        p1list[12].status && p1list[16].status && p1list[20].status) {
        SLineInfo line_T1 = cacleline_4_insert(p1list[0], p1list[1], p1list[2], p1list[3]);
        SLineInfo line_T2 = cacleline_4_insert(p1list[8], p1list[9], p1list[10], p1list[11]);
        SLineInfo line_L1 = cacleline_4_insert(p1list[4], p1list[5], p1list[6], p1list[7]);
        SLineInfo line_L2 = cacleline_4_insert(p1list[20], p1list[21], p1list[22], p1list[23]);
        SLineInfo line_R = cacleline_4_insert(p1list[12], p1list[13], p1list[14], p1list[15]);
        SLineInfo line_B = cacleline_4_insert(p1list[16], p1list[17], p1list[18], p1list[19]);
        
        if (-2 == taskindex)
        {
            grbl_sendf(CLIENT_SERIAL, "line_T1: %f,%f\r\n", line_T1.a, line_T1.b);
            grbl_sendf(CLIENT_SERIAL, "line_T2: %f,%f\r\n", line_T2.a, line_T2.b);
            grbl_sendf(CLIENT_SERIAL, "line_L1: %f,%f\r\n", line_L1.a, line_L1.b);
            grbl_sendf(CLIENT_SERIAL, "line_L2: %f,%f\r\n", line_L2.a, line_L2.b);
            grbl_sendf(CLIENT_SERIAL, "line_B: %f,%f\r\n", line_B.a, line_B.b);
            grbl_sendf(CLIENT_SERIAL, "line_R: %f,%f\r\n", line_R.a, line_R.b);
        }

        SPointInfo piont_LT = cacle_FindPoins(p1list[0], p1list[1], p1list[2], p1list[3], 
                                                p1list[4], p1list[5], p1list[6],p1list[7], line_T1, line_L1);
        SPointInfo piont_LB = cacle_FindPoins(p1list[16], p1list[17], p1list[18], p1list[19], 
                                                p1list[20], p1list[21], p1list[22],p1list[23], line_B, line_L2);
        SPointInfo piont_RT = cacle_FindPoins(p1list[8], p1list[9], p1list[10], p1list[11], 
                                                p1list[12], p1list[13], p1list[14], p1list[15], line_T2, line_R);
        if (-2 == taskindex) {
            grbl_sendf(CLIENT_SERIAL,"piont_LT:%d, %d\r\n",piont_LT.x,piont_LT.y);
            grbl_sendf(CLIENT_SERIAL,"piont_LB:%d, %d\r\n",piont_LB.x,piont_LB.y);
            grbl_sendf(CLIENT_SERIAL,"piont_RT:%d, %d\r\n",piont_RT.x,piont_RT.y);
        }
        
        ///////
        SPointInfo vsp;
        vsp.x = piont_LT.x;
        vsp.y = piont_LB.y;
        float angelesp = cacleAngle(vsp, piont_LB, piont_LT);
        if (angelesp < 183.0 && angelesp > 177.0){
            angelesp += 180.0;
        }
        if (angelesp < 0.0){
            angelesp += 360.0;
        }
        else if (angelesp > 360.0){
            angelesp -= 360.0;
        }
        ///////

        float offset_x = (piont_LT.x - marker_offset_x + m_coord_offset_x) * 0.01;
        float offset_y = (piont_LT.y - marker_offset_y + m_coord_offset_y) * 0.01;
        float homing_pulloff_dist = homing_pulloff->get();
        float work_coord_x_offset = sj_cfg_x_offset_11[param_index]->get();
        float work_coord_y_offset = sj_cfg_y_offset_11[param_index]->get();
        // grbl_sendf(CLIENT_SERIAL,"SEEKMARK: param_index:%d, x_offset:%f, y_offset:%f\n",param_index,work_coord_x_offset,work_coord_y_offset);
        offset_x = work_coord_x_offset + homing_pulloff_dist + DEFAULT_WORKCOORD_X_OFFSET + DEFAULT_X_OFFSET + offset_x + machine_offset_x[MACHINE_GENERATION];
        offset_y = work_coord_y_offset + DEFAULT_WORKCOORD_Y_OFFSET + DEFAULT_Y_OFFSET + offset_y + machine_offset_y[MACHINE_GENERATION];
        int x_scale_compensate = 160;
        int y_scale_compensate = 160;
        
        float x_finally_compens = ((sj_cfg_x_compens_11[param_index]->get() != 0.0) ? sj_cfg_x_compens_11[param_index]->get() : 1.0);
        float y_finally_compens = ((sj_cfg_y_compens_11[param_index]->get() != 0.0) ? sj_cfg_y_compens_11[param_index]->get() : 1.0);
        float multiplying_x = x_finally_compens * sqrt((p1list[13].x - p1list[5].x) * (p1list[13].x - p1list[5].x) + (p1list[13].y - p1list[5].y) * (p1list[13].y - p1list[5].y)) / (marker_width + x_scale_compensate);
        float multiplying_y = y_finally_compens * sqrt((p1list[17].x - p1list[1].x) * (p1list[17].x - p1list[1].x) + (p1list[17].y - p1list[1].y) * (p1list[17].y - p1list[1].y)) / (marker_height + y_scale_compensate);

        result = { offset_x, offset_y, angelesp, multiplying_x, multiplying_y, true };
		if (-2 == taskindex) {
                grbl_sendf(CLIENT_ALL,
                   "result_pre: %d,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f\r\n",
                   taskindex,
                   result.offset_x,
                   result.offset_y,
                   result.angle,
                   result.scale_x,
                   result.scale_y);
            }
	
        if (result.scale_x < 0.9 || result.scale_x > 1.15) {
            result.status = false;
			if (-2 == taskindex) {
                grbl_sendf(CLIENT_SERIAL,"result_false:result.scale_x: %f\r\n",result.scale_x);
            }
        }
        if (result.scale_y < 0.9 || result.scale_y > 1.15) {
            result.status = false;
			if (-2 == taskindex) {
                grbl_sendf(CLIENT_SERIAL,"result_false:result.scale_y: %f\r\n",result.scale_y);
            }
        }
        if (result.angle <= 357.0 && result.angle >= 3.0) {
            result.status = false;
            if (-2 == taskindex) {
                grbl_sendf(CLIENT_SERIAL,"result_false:result.angle: %f\r\n",result.angle);
            }
        }
    }

#ifndef DEBUG_SIMULATOR
    SJMotorCtrl::SeekActionEnd();
    if (!result.status) {
        grbl_sendf(CLIENT_ALL, "SEEK: -1,0.0,0.0,0.0,1.0,1.0,%d\r\n",param_index);
    } else {
        grbl_sendf(CLIENT_ALL,
                   "SEEK: %d,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%d\r\n",
                   taskindex,
                   result.offset_x,
                   result.offset_y,
                   result.angle,
                   result.scale_x,
                   result.scale_y,
                   param_index);
    }
#endif
}
