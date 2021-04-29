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
        //imshow("video", srcImage);

        Mat img, templ, result;
        // templ = srcImage;
        resize(srcImage, templ, Size(640, 480));
        img = imread("Template.png");

        int result_cols = img.cols - templ.cols + 1;
        int result_rows = img.rows - templ.rows + 1;
        cout << result_cols << " " << result_rows << endl;
        result.create(result_cols, result_rows, CV_32FC1);

        matchTemplate(img, templ, result, TM_SQDIFF_NORMED); //这里我们使用的匹配算法是标准平方差匹配 method=CV_TM_SQDIFF_NORMED，数值越小匹配度越好
        normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

        double minVal = -1;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Point matchLoc;
        cout << "匹配度：" << minVal << endl;
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

        cout << "匹配度：" << minVal << endl;

        matchLoc = minLoc;

        rectangle(img, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar(0, 255, 0), 2, 8, 0);

        imshow("img", img);
        imshow("video", templ);

        waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
    }
    return 0;
}
