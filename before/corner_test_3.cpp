
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

VideoCapture capture;
Mat srcImage;

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
        Mat srcgray, dstImage, normImage, scaledImage;

        cvtColor(srcImage, srcgray, COLOR_BGR2GRAY);

        Mat srcbinary;
        threshold(srcgray, srcbinary, 0, 255, THRESH_OTSU | THRESH_BINARY);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(15, 15), Point(-1, -1));
        morphologyEx(srcbinary, srcbinary, MORPH_OPEN, kernel, Point(-1, -1));

        /*
	//1、Harris角点检测
	cornerHarris(srcgray, dstImage, 3, 3, 0.01, BORDER_DEFAULT);
	//归一化与转换
	normalize(dstImage, normImage, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(normImage, scaledImage);
	Mat binaryImage;
	threshold(scaledImage, binaryImage, 0, 255, THRESH_OTSU | THRESH_BINARY);
*/

        //2、Shi-Tomasi算法：确定图像强角点
        vector<Point2f> corners; //提供初始角点的坐标位置和精确的坐标的位置
        int maxcorners = 200;
        double qualityLevel = 0.04; //角点检测可接受的最小特征值
        double minDistance = 500;    //角点之间最小距离
        int blockSize = 3;          //计算导数自相关矩阵时指定的领域范围
        double k = 0.04;            //权重系数

        goodFeaturesToTrack(srcgray, corners, maxcorners, qualityLevel, minDistance, Mat(), blockSize, false, k);
        //Mat():表示感兴趣区域；false:表示不用Harris角点检测

        //输出角点信息
        // cout << "角点信息为：" << corners.size() << endl;

        //绘制角点
        // RNG rng(12345);
        for (unsigned i = 0; i < corners.size(); i++)
        {
            // circle(srcImage, corners[i], 2, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1, 8, 0);
            circle(srcImage, corners[i], 2, Scalar(0,0,255), -1, 8, 0);
            // cout << "角点坐标：" << corners[i] << endl;
        }
        imshow("video",srcImage);
/*
        //3、寻找亚像素角点
        Size winSize = Size(5, 5);    //搜素窗口的一半尺寸
        Size zeroZone = Size(-1, -1); //表示死区的一半尺寸
        //求角点的迭代过程的终止条件，即角点位置的确定
        TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 40, 0.001);
        //TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 40, 0.001);

        cornerSubPix(srcgray, corners, winSize, zeroZone, criteria);

        //输出角点信息
        cout << "角点信息为：" << corners.size() << endl;

        //绘制角点
        for (unsigned i = 0; i < corners.size(); i++)
        {
            circle(srcImage, corners[i], 2, Scalar(255, 0, 0), -1, 8, 0);
            cout << "角点坐标：" << corners[i] << endl;
        }
*/
        waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
    }
    return 0;
}
