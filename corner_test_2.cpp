#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;
int thresh = 240;
int max_thresh = 255;

char *source_window = "Source image";
char *corners_window = "Corners detected";

VideoCapture capture;

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
        capture >> src;  //等价于 capture.read(frame);
        if (src.empty()) //如果某帧为空则退出循环
        {
            cout << "摄像头断开！" << endl;
            break;
        }
        cvtColor(src, src_gray, COLOR_BGR2GRAY);

        namedWindow(source_window, WINDOW_AUTOSIZE);
        createTrackbar("Threshold: ", source_window, &thresh, max_thresh);
        imshow(source_window, src);

        namedWindow(corners_window, WINDOW_AUTOSIZE);

        Mat dst, dst_norm, dst_norm_scaled;
        dst = Mat::zeros(src.size(), CV_32FC1);

        /// Detector parameters
        int blockSize = 2;
        int apertureSize = 5;
        double k = 0.04;

        /// Detecting corners
        cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);

        /// Normalizing
        normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);

        /// Drawing a circle around corners
        
        for (int j = 0; j < dst_norm.rows; j++)
        {
            for (int i = 0; i < dst_norm.cols; i++)
            {
                if ((int)dst_norm.at<float>(j, i) > thresh)
                {
                    circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
                }
            }
        }
        
        /// Showing the result

        imshow(corners_window, dst_norm_scaled);

        waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
    }
    return 0;
}

/** @function main */
/*int main()
{
    /// Load source image and convert it to gray
    // src = imread("map_full.png");
    // src = imread("map_corner.png");
    cvtColor(src, src_gray, COLOR_BGR2GRAY);

    /// Create a window and a trackbar
    namedWindow(source_window, WINDOW_AUTOSIZE);
    createTrackbar("Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo);
    imshow(source_window, src);

    cornerHarris_demo(0, 0);

    waitKey(0);
    return (0);
}
*/
