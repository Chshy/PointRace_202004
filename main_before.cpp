#include <iostream>
#include <opencv2/opencv.hpp>
#include <windows.h>

#include "midline.hpp"

using namespace cv;
using namespace std;

VideoCapture capture;
Mat srcImage;  //原图像
Mat grayImage; //灰度图,高斯模糊
Mat binImage;  //阈值化的二值图,边缘检测

int main()
{

	capture.open(1);		 //打开摄像头
	if (!capture.isOpened()) //如果视频不能正常打开则返回
	{
		cout << "Can't open camera!" << endl;
		return 0;
	}

	while (1)
	{
		capture >> srcImage;
		if (srcImage.empty()) //如果某帧为空则退出循环
		{
			cout << "Camera disconnected！" << endl;
			break;
		}

		cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);		  //转换为灰度
		GaussianBlur(grayImage, grayImage, Size(9, 9), 2, 2); //高斯滤波

		threshold(grayImage, binImage, 100, 255, THRESH_BINARY_INV); //阈值化(变为二值图像)
		Canny(binImage, binImage, 50, 200, 3);						 //边缘检测

		vector<Vec4i> lines; //存储HoughLinesP输出的直线 直线以两点:Vec4i(x_1,y_1,x_2,y_2)表示

		HoughLinesP(binImage, lines, 1, CV_PI / 180, 50, 50, 10); //直线检测

		// /* HoughLinesP结果绘制
		for (size_t i = 0; i < lines.size(); i++)
		{
			Vec4i l = lines[i];																	 //取出第i条直线信息
			line(srcImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, LINE_AA); //画线
		}
		// */

		//寻找中线
		vector<PolarLineTypeDef> Majorlines = MiddleLine(lines);

		// /* MiddleLine结果绘制
		for (size_t i = 0; i < Majorlines.size(); i++)
		{
			int xa, xb, ya, yb;
			PolarLineTypeDef tmp = Majorlines[i];
			xa = tmp.dist * (1.0 / sin(tmp.theta));
			if (xa >= 0)
			{
				ya = 0;
			}
			else
			{
				ya = tmp.dist * (1.0 / cos(tmp.theta));
				xa = 0;
			}

			xb = 480 / tan(tmp.theta) + xa;
			if (xb <= 640)
			{
				yb = 480;
			}
			else
			{
				yb = 480 - tan(tmp.theta) * (xb - 640);
				xb = 640;
			}
			line(srcImage, Point(xa, ya), Point(xb, yb), Scalar(0, 0, 255), 3, LINE_AA);
			// */

			/*
			//数据输出
			double phi = tmp.theta - CV_PI / 2; //-pi/2,pi/2
			double dist2ctr = cos(phi) * 320 + sin(phi) * 240 - tmp.dist;

			printf("theta=%lf\tdist=%lf\td2c=%lf", phi, tmp.dist, dist2ctr);
			if (circles.size() > 0)
				printf("\tx=%lf\ty=%lf", circles[0][0], circles[0][1]);

			printf("\n");
*/}
			imshow("video", srcImage);
			waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
	}

	waitKey(0);
	return 0;
}
