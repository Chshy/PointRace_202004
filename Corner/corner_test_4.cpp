
#include <iostream>
#include <opencv2/opencv.hpp>
#include <windows.h>

using namespace std;
using namespace cv;

VideoCapture capture;
Mat srcImage;

//Fliter
Point2f Corner;     //当前角点
Point2f LastCorner; //上一个有效角点

Point2f Delta; //误差

bool CornerExist = false;

#define AREA_LIM 50

#define EDGE_LIM 25

int Buf_p = 0;

int valid_cnt = 0;

#define CNT_LIM 10
//F
int main()
{
    // system("pause");
    capture.open(1);         //打开摄像头
    if (!capture.isOpened()) //如果视频不能正常打开则返回
    {
        cout << "can't open camera!" << endl;
        return 0;
    }
    while (1)
    {
        capture >> srcImage;  //等价于 capture.read(frame);
        if (srcImage.empty()) //如果某帧为空则退出循环
        {
            cout << "camera disconnected!" << endl;
            break;
        }

        Mat srcgray, dstImage, normImage, scaledImage;

        cvtColor(srcImage, srcgray, COLOR_BGR2GRAY);

        Mat srcbinary;
        threshold(srcgray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
        // morphologyEx(srcbinary, srcbinary, MORPH_OPEN, kernel, Point(-1, -1));

        //Shi-Tomasi算法：确定图像强角点
        vector<Point2f> corners; //提供初始角点的坐标位置和精确的坐标的位置
        int maxcorners = 200;
        double qualityLevel = 0.3; //角点检测可接受的最小特征值
        double minDistance = 1000; //角点之间最小距离
        int blockSize = 3;         //计算导数自相关矩阵时指定的领域范围
        double k = 0.04;           //权重系数

        goodFeaturesToTrack(srcgray, corners, maxcorners, qualityLevel, minDistance, Mat(), blockSize, false, k);
        //Mat():表示感兴趣区域；false:表示不用Harris角点检测

        //绘制角点

        if (corners.size() > 0 &&
            !(corners[0].x <= EDGE_LIM || corners[0].y <= EDGE_LIM || corners[0].x >= srcgray.cols - EDGE_LIM || corners[0].y >= srcgray.rows - EDGE_LIM))
        {

            Delta = corners[0] - LastCorner;
            if (abs(Delta.x) <= AREA_LIM && abs(Delta.y) <= AREA_LIM)
            {
                valid_cnt++;
                if (valid_cnt >= 3)
                {
                    CornerExist = true;
                    Corner = corners[0];
                    valid_cnt = 0;
                }
            }

            else
            {
                CornerExist = false;
            }

            LastCorner = corners[0];
        }
        else
        {
            if (valid_cnt > 0)
                valid_cnt--;
            CornerExist = false;
        }

        if (CornerExist)
        {
            circle(srcImage, Corner, 2, Scalar(0, 0, 255), -1, 8, 0);
            cout << Corner.x << " " << Corner.y << endl;
        }
        else
        {
            cout << "Corner Not Exist cnt=" << valid_cnt << endl;
        }

        imshow("video", srcImage);
        imshow("binary", srcbinary);

        waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
    }
    return 0;
}
