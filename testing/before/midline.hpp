//#ifndef __MIDLINE_HPP
//#define __MIDLINE_HPP 1

//#define EPS 1E-4                                            //浮点数判等条件
//#define EQ(x, y) (abs((x) - (y)) <= EPS) ? (true) : (false) //浮点数判等

#define LINE_DIST(A, B, C) (((C) * (C)) / ((A) * (A) + (B) * (B)))

#define THETA_THRESHOLD CV_PI / 36
//#define DIST_THRESHOLD 30000.0
#define DIST_THRESHOLD 10.0

#include <iostream>
#include <opencv2/opencv.hpp>
#include <windows.h>
using namespace cv;
using namespace std;

vector<int> x;
//结构体定义
struct PolarLineTypeDef
{
    double theta;
    double dist; //到原点距离
    double len;  //长度
};

// 函数声明

/**
 * @brief 寻找图像上最主要直线的中心线
 * @return 图像上最主要的直线的rho和theta(结构体存储)
 * @param image 目标图像
 * @param lines HoughLinesP存储直线的vector
 */
PolarLineTypeDef MiddleLine(Mat image, vector<Vec4i> &lines);

/**
 * @brief 排序用比较函数,从小到大,theta优先于dist
 * @example sort(arr,arr+n,PolarLineCmp);
 */
bool PolarLineCmp(PolarLineTypeDef a, PolarLineTypeDef b);

/**
 * @brief 检测两个数据的跳变程度
 * @return 存在跳变返回假,不存在返回真
 */
bool JumpDetect(PolarLineTypeDef a, PolarLineTypeDef b);

/**
 * @brief 排序用比较函数,len从大到小
 * @example sort(arr,arr+n,PolarLineLenCmp);
 */
bool PolarLineLenCmp(PolarLineTypeDef a, PolarLineTypeDef b);

//===============================================================================
// 函数实现
PolarLineTypeDef MiddleLine(vector<Vec4i> &lines)
{

    //----------S.1.数据预处理:转换为极坐标----------

    vector<PolarLineTypeDef> Polarlines;

    for (size_t i = 0; i < lines.size(); i++)
    {
        PolarLineTypeDef tmp; //暂存变量

        double x1, x2, y1, y2;
        x1 = lines[i][0];
        y1 = lines[i][1];
        x2 = lines[i][2];
        y2 = lines[i][3];

        //计算theta
        tmp.theta = atan2(y2 - y1, x2 - x1); //利用反正切计算theta
        if (tmp.theta < 0)                   //直线旋转180*为同一条直线,将theta统一到[0,pi]}
        {
            tmp.theta += CV_PI;
        }

        //计算dist
        tmp.dist = LINE_DIST(y2 - y1, -(x2 - x1), (x1 * (y2 - y1) - y1 * (x2 - x1)));
        tmp.dist = sqrt(tmp.dist); //DEBUG

        //计算len
        tmp.len = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

        //将这条线加入到vector的末尾
        Polarlines.push_back(tmp);
    }

    //----------S.2.数据排序,相似数据拟合----------

    //排序
    sort(Polarlines.begin(), Polarlines.end(), PolarLineCmp);
    /*
    cout << "sorted:" << endl;
    for (size_t i = 0; i < Polarlines.size(); i++)
    {
        cout << "theta=" << Polarlines[i].theta << " dist=" << Polarlines[i].dist << endl;
    }
    // */

    vector<PolarLineTypeDef> Majorlines; //拟合后的主要线段

    //排序后参数相近的

    //cout << "==========" << endl;
    for (size_t i = 0; i < Polarlines.size(); i++)
    {
        //cout << "i=" << i << endl;
        PolarLineTypeDef ave;
        int cnt = 1;
        ave.theta = Polarlines[i].theta;
        ave.dist = Polarlines[i].dist;
        ave.len = 0;

        for (size_t j = i + 1; (JumpDetect(Polarlines[j - 1], Polarlines[j])) && (j < Polarlines.size()); i++, j++)
        {
            ave.theta += Polarlines[j].theta;
            ave.dist += Polarlines[j].dist;
            ave.len += Polarlines[j].len;
            cnt++;
        }
        ave.theta /= cnt;
        ave.dist /= cnt;
        //cout << ave.theta << " " << ave.dist << endl;

        //存入Majorlines
        Majorlines.push_back(ave);
    }

    //注意：以下代码只适用于视野内只有一条直线的情况!!!!!!!!!
    //有时可能需要在霍夫变换前剪裁图像边缘

    //cout << "major " << Majorlines.size() << endl;

    /*
    double len_sum = 0;
    for (size_t i = 0; i < Majorlines.size(); i++)
    {
        len_sum += Majorlines[i].len;
    }
    

    PolarLineTypeDef ret;
    ret.theta = ret.dist = 0;
    int cnt = 0;
    for (size_t i = 0; i < Majorlines.size(); i++)
    {

        //if (Majorlines[i].dist < 10)
        //    continue;
        //cout << "theta=" << Majorlines[i].theta << " dist=" << Majorlines[i].dist << endl;
        ret.theta += Majorlines[i].theta * (Majorlines[i].len / len_sum);
        ret.dist += Majorlines[i].dist * (Majorlines[i].len / len_sum);
        cnt++;
    }
    */

    printf("%d\t", Majorlines.size());
    sort(Majorlines.begin(), Majorlines.end(), PolarLineLenCmp);
    PolarLineTypeDef ret;
    ret.theta = ret.dist = 0;
    int cnt = 0;
    for (size_t i = 0; i < Majorlines.size(); i++)
    {
        ret.theta += Majorlines[i].theta;
        ret.dist += Majorlines[i].dist;
        cnt++;
        if (cnt >= 2)
            break;
    }

    ret.theta /= cnt;
    ret.dist /= cnt;

    return ret;
}

bool PolarLineCmp(PolarLineTypeDef a, PolarLineTypeDef b)
{
    if (abs(a.theta - b.theta) > THETA_THRESHOLD)
    {
        return a.theta < b.theta;
    }
    else
    {
        return a.dist < b.dist;
    }
}

bool JumpDetect(PolarLineTypeDef a, PolarLineTypeDef b)
{

    if (abs(a.theta - b.theta) > THETA_THRESHOLD)
        return false;
    else if (abs(a.dist - b.dist) > DIST_THRESHOLD)
        return false;
    else
        return true;
}

bool PolarLineLenCmp(PolarLineTypeDef a, PolarLineTypeDef b)
{
    return a.len > b.len;
}

//#endif
