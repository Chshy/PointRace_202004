#include <iostream>
#include <opencv2/opencv.hpp>
#include <windows.h>

#include "midline.hpp"
#include "target_stabilizer.hpp"

using namespace cv;
using namespace std;

#define IMAGE_DISPLAY

VideoCapture capture;
Mat srcImage;  //原图像
Mat grayImage; //灰度图,高斯模糊
Mat binImage;  //阈值化的二值图,边缘检测

TargetStabilizer Lines;

void DrawLines(Mat ImgtoDraw, vector<Vec2f> Draw_lines)
{ // /* MiddleLine结果绘制
	for (size_t i = 0; i < Draw_lines.size(); i++)
	{
		Vec2f cur = Draw_lines[i];
		int xa, xb, ya, yb;
		float theta = cur[0];
		float dist = cur[1];

		// xa = 320 + dist * cos(theta);
		// ya = 320 + dist * sin(theta);
		/*

		xa = 320;
		ya = 240;
		if (theta < CV_PI/2)
			dist = -dist;
		xb = xa + dist * sin(theta);
		yb = ya + dist * -cos(theta);*/

		// xb = xa + 50 * cos(theta);
		// yb = ya + 50 * sin(theta);
		/*
		theta = CV_PI - theta;
		xa = 320 + 240 / tan(theta);
		ya = 0;
		xb = 0;
		yb = 240 - 320 / tan(theta);
		*/

		if (theta == 0)
		{
			xa = 0;
			xb = 640;
			ya = abs(dist);
			yb = abs(dist);
		}
		else
		{
			xa = dist * (1.0 / sin(theta));
			if (xa >= 0)
			{
				ya = 0;
			}
			else
			{
				ya = -dist * (1.0 / cos(theta));
				xa = 0;
			}

			xb = xa + (480 - ya) / tan(theta);
			if (xb >= 0)
			{
				yb = 480;
			}
			else
			{
				yb = -dist * (1.0 / cos(theta));
				xb = 0;
			}
		}
		//cout << theta << " " << xa << " " << yb << endl;
		// cout << theta << " " << dist << endl;

		line(srcImage, Point(xa, ya), Point(xb, yb), Scalar(0, 0, 255), 3, LINE_AA);
	}
	return;
}

int main()
{
	Lines.object_type = Target_Line;
	// Lines.roi={{0,0},{640,480}};
	// /*
	capture.open(1);		 //打开摄像头
	if (!capture.isOpened()) //如果视频不能正常打开则返回
	{
		cout << "Can't open camera!" << endl;
		return 0;
	}
	//*/
	while (1)
	{
		capture >> srcImage;
		// srcImage = imread("Template.png");
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
#ifdef IMAGE_DISPLAY
		for (size_t i = 0; i < lines.size(); i++)
		{
			Vec4i l = lines[i];																	 //取出第i条直线信息
			line(srcImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, LINE_AA); //画线
		}
#endif
		// */

		//寻找中线
		vector<PolarLineTypeDef> Majorlines = MiddleLine(lines);

		vector<Vec2f> MajorLineStable;
		for (size_t i = 0; i < Majorlines.size(); i++)
		{
			MajorLineStable.push_back({(float)Majorlines[i].theta, (float)Majorlines[i].dist});
		}
		MajorLineStable = Lines.update(MajorLineStable);

#ifdef IMAGE_DISPLAY
		DrawLines(srcImage, MajorLineStable);
#endif

		// */

		/*
			//数据输出
			double phi = tmp.theta - CV_PI / 2; //-pi/2,pi/2
			double dist2ctr = cos(phi) * 320 + sin(phi) * 240 - tmp.dist;

			printf("theta=%lf\tdist=%lf\td2c=%lf", phi, tmp.dist, dist2ctr);
			if (circles.size() > 0)
				printf("\tx=%lf\ty=%lf", circles[0][0], circles[0][1]);

			printf("\n");
*/
		if (MajorLineStable.size() == 0)
		{
			
		}
		else if (MajorLineStable.size() == 1)
		{

		}
		else if (MajorLineStable.size() >= 2)
		{

		}

#ifdef IMAGE_DISPLAY
		imshow("video", srcImage);
#endif
		waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
	}

	waitKey(0);
	return 0;
}
