#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
//#include <vector>

#define SHOW_THRESHOLD 30
#define DROP_THRESHOLD 20
#define MEMORY_LEVEL_MAX 1000

using namespace std;
using namespace cv;

enum Target_Type
{
    Target_Point = 0,
    Target_Line = 1, //Polar
};

struct MemoryTypeDef
{
    Vec2f data;
    int memory_level;
    bool updated;
};

//x y
//theta dist
struct TargetStabilizer
{
    //稳定器基本参数,一般不变动
    Target_Type object_type;
    Vec2f roi[2]; //只有类型为点时才会用到 [0]左上 [1]右下

    //数据记录部分
    vector<MemoryTypeDef> old_objects;

    //========================================

    bool inrange(Vec2f obj_a, Vec2f obj_b)
    {                                                             //line:theta,dist
        static const double maximun_valid_point_dist = 75;        //pixel
        static const double maximun_valid_line_delta_theta = 0.2; //rad
        static const double maximun_valid_line_delta_dist = 50;

        bool is_inrange = false;

        switch (object_type)
        {
        case Target_Point:
            if ((obj_a[0] - obj_b[0]) * (obj_a[0] - obj_b[0]) + (obj_a[1] - obj_b[1]) * (obj_a[1] - obj_b[1]) <= maximun_valid_point_dist * maximun_valid_point_dist)
            {
                is_inrange = true;
            }
            break;
        case Target_Line:
            if (abs(obj_a[0] - obj_b[0]) <= maximun_valid_line_delta_theta || abs(obj_a[0] - obj_b[0]) >= CV_PI - maximun_valid_line_delta_theta)
            {
                if (abs(obj_a[1] - obj_b[1]) <= maximun_valid_line_delta_dist)
                {
                    is_inrange = true;
                }
            }
            break;
        default:
            break;
        }
        return is_inrange;
    }

    //==========================================================================

    vector<Vec2f> update(vector<Vec2f> new_objects)
    {
        Vec2f old_all;
        for (size_t i = 0; i < new_objects.size(); i++)
        {
            bool find_this_object_in_memory = false;

            //在old_objects中寻找匹配的数据j
            for (size_t j = 0; j < old_objects.size(); j++)
            {
                //如果数据j未被更新过
                if (old_objects[j].updated == 0)
                {
                    if (inrange(new_objects[i], old_objects[j].data))
                    {
                        old_objects[j].data = new_objects[i]; //更新旧数据
                        find_this_object_in_memory = true;    //标记新数据i已经找到对应的旧数据j
                        old_objects[j].updated = true;        //旧数据j的更新标记

                        //记忆处理部分
                        if (old_objects[j].memory_level >= SHOW_THRESHOLD - 1) //如果这个目标当前是被显示的状态
                        {
                            old_objects[j].memory_level = SHOW_THRESHOLD + DROP_THRESHOLD - 1; //直接加到上限值
                        }
                        else //如果这个目标当前没有被显示
                        {
                            old_objects[j].memory_level += 1; //逐渐回忆起这个目标
                        }
                    }
                }
            }

            //如果没找到，当作新点处理
            if (!find_this_object_in_memory)
            {
                old_objects.push_back({new_objects[i], 1, 1});
            }
        }

        vector<Vec2f> array_for_display;
        for (size_t j = 0; j < old_objects.size(); j++)
        {
            if (old_objects[j].updated == 0) //本帧未检测到这个数据
            {
                if (old_objects[j].memory_level < SHOW_THRESHOLD) //如果忘记的多了，就彻底删除
                {
                    old_objects[j].memory_level = 0;
                }
                else
                {
                    switch (object_type)
                    {
                    case Target_Point:
                        if (old_objects[j].data[0] > roi[0][0] && old_objects[j].data[0] < roi[1][0] && old_objects[j].data[1] > roi[0][1] && old_objects[j].data[1] < roi[1][1])
                        {                                     //如果旧点在roi内
                            old_objects[j].memory_level -= 1; //逐渐忘记
                        }
                        else
                        { //旧点在roi外
                            old_objects[j].memory_level = 0;
                        }
                        break;
                    case Target_Line:
                        old_objects[j].memory_level -= 1; //逐渐忘记
                        break;
                    default:
                        break;
                    }
                }
            }
            else
            {
                old_objects[j].updated = 0; //为下一帧清空update标记
            }

            if (old_objects[j].memory_level >= SHOW_THRESHOLD)
            {
                array_for_display.push_back(old_objects[j].data);
            }
            else if (old_objects[j].memory_level <= 0)
            {
                //删除
                old_objects[j] = old_objects.back();
                old_objects.pop_back();
                j--;
            }
        }
        return array_for_display;
    }
};
