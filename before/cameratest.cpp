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
        imshow("video", srcImage);
        waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
    }
    return 0;
}
