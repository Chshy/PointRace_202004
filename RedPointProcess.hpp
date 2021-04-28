#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

namespace RedPoint
{
    static const int Dn_S = 120, Up_S = 255; //饱和
    static const int Dn_V = 100, Up_V = 180; //明度

    static const int Center_ROI[4] = {160, 480, 120, 360}; //x,y

    Mat srcImage_HSV;
    Mat binImage, tmp;

    bool RedPointProcess(Mat srcImage_BGR)
    {
        cvtColor(srcImage_BGR, srcImage_HSV, COLOR_BGR2HSV, 0);
        inRange(srcImage_HSV, Scalar(0, Dn_S, Dn_V), Scalar(5, Up_S, Up_V), tmp);
        inRange(srcImage_HSV, Scalar(170, Dn_S, Dn_V), Scalar(179, Up_S, Up_V), binImage);
        binImage += tmp;
        morphologyEx(binImage, binImage, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(3, 3)));

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        Mat dst;
        findContours(binImage, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

        morphologyEx(binImage, binImage, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(5, 5)));
        morphologyEx(binImage, binImage, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(9, 9)));

        vector<Moments> mu(contours.size()); //计算轮廓矩
        for (int i = 0; i < contours.size(); i++)
        {
            mu[i] = moments(contours[i], false);
        }
        vector<Point2f> mc(contours.size()); //计算轮廓中心
        for (int i = 0; i < contours.size(); i++)
        {
            mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
        }

        /*
    //画轮廓及其质心并显示
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(srcImage, contours, i, Scalar(0, 255, 0), 1, 8, hierarchy, 0, Point()); //绘制轮廓
        circle(srcImage, mc[i], 3, Scalar(255, 0, 0));                                       //画中心圆
    }
    */
        if (contours.size())
        {
            if (mc[0].x >= Center_ROI[0] && mc[0].x <= Center_ROI[1] && mc[0].y >= Center_ROI[2] && mc[0].y <= Center_ROI[3])
            {
                return true;
            }
        }
        // cout << mc[0].x << " " << mc[0].y << endl;

        return false;
    }
} // namespace RedPoint