#include <iostream>
#include <opencv2/opencv.hpp>
#include <windows.h>

#include "midline.hpp"
#include "target_stabilizer.hpp"
#include "SDI_Msg.hpp"

using namespace cv;
using namespace std;

#define IMAGE_DISPLAY

VideoCapture capture;
Mat srcImage;  //原图像
Mat grayImage; //灰度图,高斯模糊
Mat binImage;  //阈值化的二值图,边缘检测

TargetStabilizer Lines;

//--------------

//--------------

void GetAxisPoint(float input_theta, float input_dist, int *xa, int *ya, int *xb, int *yb)
{
	if (input_theta == 0)
	{
		*xa = 0;
		*xb = 640;
		*ya = abs(input_dist);
		*yb = abs(input_dist);
	}
	else if (input_theta == CV_PI / 2)
	{
		*xa = abs(input_dist);
		*xb = abs(input_dist);
		*ya = 0;
		*yb = 480;
	}
	else
	{
		*xa = input_dist * (1.0 / sin(input_theta));
		if (*xa >= 0)
		{
			*ya = 0;
		}
		else
		{
			*ya = -input_dist * (1.0 / cos(input_theta));
			*xa = 0;
		}

		*xb = *xa + (480 - *ya) / tan(input_theta);
		if (*xb >= 0)
		{
			*yb = 480;
		}
		else
		{
			*yb = -input_dist * (1.0 / cos(input_theta));
			*xb = 0;
		}
	}

	return;
}

void DrawLines(Mat ImgtoDraw, vector<Vec2f> Draw_lines)
{ // /* MiddleLine结果绘制
	for (size_t i = 0; i < Draw_lines.size(); i++)
	{
		Vec2f cur = Draw_lines[i];
		int xa, xb, ya, yb;
		float theta = cur[0];
		float dist = cur[1];

		// printf("i=%d\n", i);

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

		//cout << theta << " " << xa << " " << yb << endl;
		// cout << theta << " " << dist << endl;

		GetAxisPoint(theta, dist, &xa, &ya, &xb, &yb);
		// printf("theta=%f dist=%f\n", theta, dist);

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
		// cout << MajorLineStable.size() << endl;
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
			// vec2pack(0xF0, 0, 0);
		}
		else if (MajorLineStable.size() == 1)
		{
			// vec2pack(0xF1, MajorLineStable[0][0], MajorLineStable[0][1]); //theta dist2(0,0)
		}
		else if (MajorLineStable.size() >= 2)
		{
			//求交点
			//目前是求的角度差最大的两条直线的交点，待优化

			//找角度最大和最小的直线
			Vec2f *theta_min_index = &MajorLineStable[0], *theta_max_index = &MajorLineStable[0];
			for (size_t i = 1; i < MajorLineStable.size(); i++)
			{
				if (MajorLineStable[i][0] < (*theta_min_index)[0]) //找到更小的角
				{
					theta_min_index = &MajorLineStable[i];
				}
				if (MajorLineStable[i][0] > (*theta_max_index)[0]) //找到更大的角
				{
					theta_max_index = &MajorLineStable[i];
				}
			}

			//求交点
			float line1_k, line2_k;
			int line1_xa, line1_xb, line1_ya, line1_yb;
			int line2_xa, line2_xb, line2_ya, line2_yb;
			GetAxisPoint((*theta_min_index)[0], (*theta_min_index)[1], &line1_xa, &line1_ya, &line1_xb, &line1_yb);
			GetAxisPoint((*theta_max_index)[0], (*theta_max_index)[1], &line2_xa, &line2_ya, &line2_xb, &line2_yb);

			line1_k = (float)(line1_yb - line1_ya) / (line1_xb - line1_xb);
			line2_k = (float)(line2_yb - line2_ya) / (line2_xb - line2_xb);

			Vec2f CrossPoint;
			CrossPoint[0] = (float)(line1_k * line1_xa - line2_k * line2_xa + line2_ya - line1_ya) / (line1_k - line2_k);
			CrossPoint[1] = ((float)(line2_k * line1_xa - line2_k * line2_xa + line2_ya - line1_ya) / (line1_k - line2_k)) * line1_k + line1_ya;

			// printf("size=%d x=%f y=%f\n", MajorLineStable.size(), CrossPoint[0], CrossPoint[1]);

			// vec2pack(0xF2, 0, 0);

			//判断是否已经稳定

			//当前直线+1
		}

#ifdef IMAGE_DISPLAY
		imshow("video", srcImage);
#endif
		waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
	}

	waitKey(0);
	return 0;
}
