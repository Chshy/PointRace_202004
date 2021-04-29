#include <iostream>
#include <opencv2/opencv.hpp>

#include "midline.hpp"
#include "target_stabilizer.hpp"
#include "SDI_Msg.hpp"
#include "RedPointProcess.hpp"

using namespace cv;
using namespace std;

#define IMAGE_DISPLAY
#define SEND_SDI

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

		//cout << theta << " " << xa << " " << yb << endl;
		// cout << theta << " " << dist << endl;

		GetAxisPoint(theta, dist, &xa, &ya, &xb, &yb);
		// printf("theta=%f dist=%f\n", theta, dist);

		line(srcImage, Point(xa, ya), Point(xb, yb), Scalar(0, 0, 255), 3, LINE_AA);
	}
	return;
}

//10个点，2x2个边界值
const float K_Threshold[10 + 4][4] = {
	//					k1<k2
	//		k1_les	k1_grt	k2_les	k2_grt
	/*P0*/ 2.60, 3.00, 3.21, 3.61,
	/*P1*/ 2.55, 2.95, 4.48, 4.88,
	/*P2*/ 1.38, 1.78, 3.86, 4.26,
	/*P3*/ 2.55, 2.95, 3.90, 4.30,
	/*P4*/ 1.97, 2.37, 2.57, 2.97,
	/*P5*/ 1.91, 2.31, 3.88, 4.28,
	/*P6*/ 3.25, 3.65, 3.85, 4.25,
	/*P7*/ 1.95, 2.35, 3.18, 3.58,
	/*P8*/ 1.90, 2.30, 4.35, 4.75,
	/*P9*/ 3.25, 3.65, 4.45, 4.85,

	/*P1*/ 1.37, 1.77, 2.55, 2.95,
	/*P2*/ 3.96, 4.36, 4.48, 4.88,
	/*P8*/ 1.37, 1.77, 1.95, 2.35,
	/*P9*/ 1.37, 1.77, 3.30, 3.70
	//4个带水平线的可能需要第二套阈值

};

//本函数变量仍然用k表示，但是实际上比较的应是theta(拟合的theta + CV_PI/2)
int GetCrossPointNum(float k1, float k2)
{
	int num = -1;
	if (k1 > k2)
	{
		swap(k1, k2);
	}
	int i = 0;
	for (i = 0; i < 14; i++)
	{
		if (k1 >= K_Threshold[i][0] &&
			k1 <= K_Threshold[i][1] &&
			k2 >= K_Threshold[i][2] &&
			k2 <= K_Threshold[i][3])
		{
			num = i;
			break;
		}
	}
	if (num == 10)
		num = 1;
	if (num == 11)
		num = 2;
	if (num == 12)
		num = 8;
	if (num == 13)
		num = 9;
	return num;
}

int CurrentLine = 0;
bool Leaving_Current_Corner = false;
bool RedPointCaptured = false;
Mat MyCapture = Mat::zeros(640, 480, CV_8UC3);

int main()
{
	Lines.object_type = Target_Line;
	// Lines.roi={{0,0},{640,480}};//线类型不需要roi
	// /*
	capture.open(1);		 //打开摄像头
	if (!capture.isOpened()) //如果视频不能正常打开则返回
	{
		cout << "Can't open camera!" << endl;
		return 0;
	}
	// capture.set(CAP_PROP_SETTINGS, 1);//打开摄像头参数设置窗口,仅windows有效
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

		if (!RedPointCaptured)
		{ //如果还没有拍过照
			if (RedPoint::RedPointProcess(srcImage))
			{ //如果有红点则处理

				MyCapture = srcImage;

				imshow("Captured", MyCapture);
				RedPointCaptured = true;

				printf("(Capture)SDI_Send=0xF5,0,0\n");
#ifdef SEND_SDI
				vec2pack(0xF5, 0, 0);
#endif
			}
		}

		cvtColor(srcImage, grayImage, COLOR_BGR2GRAY);			//转换为灰度
		GaussianBlur(grayImage, grayImage, Size(23, 23), 2, 2); //高斯滤波

		threshold(grayImage, binImage, 80, 255, THRESH_BINARY_INV); //阈值化(变为二值图像)

		Canny(binImage, binImage, 50, 200, 3); //边缘检测

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

		/////////////////////////////////////////////////////////
		//			注意！！msg.cmd要改成未占用的！！！！		 //
		////////////////////////////////////////////////////////

		// imshow("Captured2", MyCapture);
		if (MajorLineStable.size() == 0) //摄像头内没有直线
		{
			printf("size=%d SDI_Send=0xF0,0,0\n", MajorLineStable.size());
			//我也不知道该做什么
			//总之先发个包告诉飞控我啥都没看到
#ifdef SEND_SDI
			vec2pack(0xF0, 0, 0);
#endif
			Leaving_Current_Corner = false;
		}
		else if (MajorLineStable.size() == 1) //摄像头内有1条直线
		{
			printf("size=%d ", MajorLineStable.size());
			//把这条直线的角度和距离发出去
			//printf("theta=%f dist=%f\n", MajorLineStable[0][0], MajorLineStable[0][1]);
			printf("(theta,dist)SDI_Send=0xF1,%f,%f\n", MajorLineStable[0][0], MajorLineStable[0][1]);
#ifdef SEND_SDI
			vec2pack(0xF1, MajorLineStable[0][0], MajorLineStable[0][1]); //theta dist2(0,0)
#endif

			Leaving_Current_Corner = false;
		}
		else if (MajorLineStable.size() >= 2) //摄像头内有>=2条直线
		{
			printf("size=%d ", MajorLineStable.size());
			// Leaving_Current_Corner = false; //DEBUG
			if (Leaving_Current_Corner)
			{ //表示当前这两条(?)线的交点走过了，现在要离开
				// printf("Leaving Corner:%d ", CurrentLine - 1);

				//如果CurrentLine刚刚加到11,说明走回了起始点,

				//此处可能需要加一些处理函数

				if (CurrentLine >= 11)
				{
					//发送降落信号
					//保险起见 多发几次

					printf("(Landing)SDI_Send=0xF4,0,0\n");
#ifdef SEND_SDI
					vec2pack(0xF4, 0, 0);
					vec2pack(0xF4, 0, 0);
					vec2pack(0xF4, 0, 0);
#endif

					//后续处理加在这里

					//退出程序
					// if (RedPointCaptured)
					// {
					// 	imshow("Captured2", MyCapture);
					// }
					// waitKey(0);

					exit(0);
				}
				else
				{
					printf("(CurrentLine,y)SDI_Send=0xF3,%d,0\n", CurrentLine);
#ifdef SEND_SDI
					vec2pack(0xF3, CurrentLine, 0); //CurrentLine现在是即将走的那条直线的编号(1开始)
#endif
				}
			}
			else
			{ //表示当前这两条(?)线的交点没走过，现在要去交点
				// printf("size=%d ", MajorLineStable.size());
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

				//求和坐标轴的交点
				GetAxisPoint((*theta_min_index)[0], (*theta_min_index)[1], &line1_xa, &line1_ya, &line1_xb, &line1_yb);
				GetAxisPoint((*theta_max_index)[0], (*theta_max_index)[1], &line2_xa, &line2_ya, &line2_xb, &line2_yb);

				//计算斜率
				line1_k = (float)(line1_yb - line1_ya) / (line1_xb - line1_xa);
				line2_k = (float)(line2_yb - line2_ya) / (line2_xb - line2_xa);

				//计算交点
				Vec2f CrossPoint;
				CrossPoint[0] = (float)(line1_k * line1_xa - line2_k * line2_xa + line2_ya - line1_ya) / (line1_k - line2_k);
				CrossPoint[1] = ((float)(line2_k * line1_xa - line2_k * line2_xa + line2_ya - line1_ya) / (line1_k - line2_k)) * line1_k + line1_ya;

				// printf("size=%d x=%f y=%f\n", MajorLineStable.size(), CrossPoint[0], CrossPoint[1]);
				printf("(x,y)SDI_Send=0xF2,%f,%f\n", CrossPoint[0], CrossPoint[1]);

#ifdef SEND_SDI
				vec2pack(0xF2, CrossPoint[0], CrossPoint[1]); //CurrentLine现在是即将走的那条直线的编号(1开始)
#endif
				// printf("k1=%f k2=%f\n", line1_k, line2_k);

				// vec2pack(0xF2, 0, 0);

				//判断是否已经稳定

				// printf("size=%d x=%f y=%f corner=%d\n", MajorLineStable.size(), CrossPoint[0], CrossPoint[1], GetCrossPointNum(line1_k, line2_k));

				float line1_diff_theta, line2_diff_theta;
				line1_diff_theta = (*theta_min_index)[0] + CV_PI / 2; //将分界线调整至竖直方向
				line2_diff_theta = (*theta_max_index)[0] + CV_PI / 2; //说实话没什么用

				// printf("theta1=%f theta2=%f corner=%d\n", line1_diff_theta, line2_diff_theta, GetCrossPointNum(line1_diff_theta, line2_diff_theta));

				int cur_pos;
				cur_pos = GetCrossPointNum(line1_diff_theta, line2_diff_theta);
				//判断是否已经稳定
				if (abs(CrossPoint[0] - 320) <= 15 && abs(CrossPoint[1] - 240) <= 15)
				{
					if (cur_pos == CurrentLine) //角从0开始，边从1开始，同样数字的角在边的末端
						CurrentLine++;
					else if (cur_pos == CurrentLine - 10) //当绕到第二圈的时候
						CurrentLine++;					  //其实结束的代码在这里写也可以

					Leaving_Current_Corner = true;
				}

				//当前直线+1
			}
		}

#ifdef IMAGE_DISPLAY
		circle(srcImage, Point2f(320, 240), 3, Scalar(0, 255, 0), -1, 8, 0);
		imshow("video", srcImage);
		imshow("video2", binImage);
#endif
		waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
	}

	// waitKey(0);
	return 0;
}
