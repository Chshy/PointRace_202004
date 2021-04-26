#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

VideoCapture capture;
Mat srcImage;
Mat srcImage_HSV;

Mat dst1, dst2;

int main()
{
    capture.open(1);         //打开摄像头
    if (!capture.isOpened()) //如果视频不能正常打开则返回
    {
        cout << "摄像头打开失败！" << endl;
        return 0;
    }
    while (1)
    {
        capture >> srcImage;  //等价于 capture.read(frame);
        if (srcImage.empty()) //如果某帧为空则退出循环
        {
            cout << "摄像头断开！" << endl;
            break;
        }
        cvtColor(srcImage, srcImage_HSV, COLOR_BGR2HSV, 0);
        int Dn_S = 120, Up_S = 255; //饱和
        int Dn_V = 100, Up_V = 180; //明度
        inRange(srcImage_HSV, Scalar(0, Dn_S, Dn_V), Scalar(5, Up_S, Up_V), dst1);
        inRange(srcImage_HSV, Scalar(170, Dn_S, Dn_V), Scalar(179, Up_S, Up_V), dst2);
        dst1 += dst2;

        morphologyEx(dst1, dst1, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(5, 5)));
        morphologyEx(dst1, dst1, MORPH_CLOSE, getStructuringElement(MORPH_RECT, Size(9, 9)));

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        Mat dst;
        findContours(dst1, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

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
        //画轮廓及其质心并显示
        for (int i = 0; i < contours.size(); i++)
        {
            drawContours(srcImage, contours, i, Scalar(0, 255, 0), 1, 8, hierarchy, 0, Point()); //绘制轮廓
            circle(srcImage, mc[i], 3, Scalar(255, 0, 0));                                       //画中心圆
        }
        if (contours.size())
            cout << mc[0].x << " " << mc[0].y << endl;

        /*
        if (!contours.empty() && !hierarchy.empty())
        {
            int idx = 0;
            for (; idx >= 0; idx = hierarchy[idx][0])
            {
                Scalar color((rand() & 255), (rand() & 255), (rand() & 255));
                drawContours(srcImage, contours, idx, color, 1, 8, hierarchy);
            }
        }*/

        // morphologyEx(dst1, dst1, MORPH_OPEN, getStructuringElement(MORPH_RECT, Size(3, 3)));

        imshow("video", srcImage);
        imshow("inRange", dst1);
        waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
    }
    return 0;
}
