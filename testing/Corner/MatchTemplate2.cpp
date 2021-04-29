#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

VideoCapture capture;
Mat srcImage;

//相似度匹配算法之灰度值方差匹配法：
double get_variance(Mat &a, Mat &b)
{
    if (a.rows != b.rows || a.cols != b.cols || a.channels() != b.channels())
    {
        printf("not the same size!\n");
        return 0;
    }

    //处理图像相似度
    //1.求出每一行到灰度值均值,加入容器，作为特征值；
    //2.求出灰度值总平均值与每行平均值的方差；
    //3.行行比较与模版方差的接近程度

    vector<double> variance_a;
    vector<double> variance_b;
    double var_a = 0;
    double var_b = 0;
    double sum_a = 0;
    double sum_b = 0;
    double mean_a;
    double mean_b;
    double sum_variance = 0.0;
    //将每行灰度值均值存入容器
    for (int i = 0; i < a.rows; i++)
    {
        mean_a = 0;
        mean_b = 0;
        for (int j = 0; j < a.cols; j++)
        {
            mean_a += a.at<uchar>(i, j);
            mean_b += b.at<uchar>(i, j);
        }
        mean_a /= (double)(a.rows * a.cols);
        mean_b /= (double)(a.rows * a.cols);
        sum_a += mean_a;
        sum_b += mean_b;
        variance_a.push_back(mean_a);
        variance_b.push_back(mean_b);
    }
    //全图灰度值均值
    mean_a = sum_a / (double)variance_a.size();
    mean_b = sum_b / (double)variance_b.size();
    //灰度值方差之差累加
    for (int i = 0; i < variance_a.size(); i++)
    {
        var_a = (variance_a[i] - mean_a) * (variance_a[i] - mean_a);
        var_b = (variance_b[i] - mean_b) * (variance_b[i] - mean_b);
        sum_variance += abs(var_a - var_b);
    }

    return sum_variance;
}

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

        // imshow("video", srcImage);

        //加载图像
        Mat org = imread("Template.png");
        Mat my_template = srcImage;
        cvtColor(my_template, my_template, COLOR_RGB2GRAY);
        cvtColor(org, org, COLOR_RGB2GRAY);

        threshold(org, org, 100, 255, THRESH_BINARY); 
        threshold(my_template, my_template, 100, 255, THRESH_BINARY); 

        // imshow("video", my_template);

        int best_index;  //存储最佳匹配的序号
        double min_diff; //存储最小的方差值之差
        Rect best_rect;  //存储最佳的匹配框
        //循环缩放，当前模版为最大尺寸，每次循环缩小5%，循环10次
        for (int index = 0; index < 10; index++)
        {
            //获得缩放后的模版
            Mat temp_template = my_template.clone();
            int new_rows = my_template.rows - index * 0.05 * my_template.rows;
            int new_cols = my_template.cols - index * 0.05 * my_template.cols;
            resize(temp_template, temp_template, Size(new_cols, new_rows));

            //模版匹配
            // Mat result;
            // result.create(org.dims, org.size, org.type());
            // matchTemplate(org, temp_template, result, 0);

            Mat result;
            int result_cols = org.cols - temp_template.cols + 1;
            int result_rows = org.rows - temp_template.rows + 1;
            //cout << result_cols << " " << result_rows << endl;
            result.create(result_cols, result_rows, CV_32FC1);

            matchTemplate(org, temp_template, result, TM_SQDIFF_NORMED); //这里我们使用的匹配算法是标准平方差匹配 method=CV_TM_SQDIFF_NORMED，数值越小匹配度越好
            normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());

            //获取模版匹配得到的rect
            Point minPoint;
            Point maxPoint;
            double *minVal;
            double *maxVal;
            minMaxLoc(result, minVal, maxVal, &minPoint, &maxPoint);
            Rect rect(minPoint.x, minPoint.y, new_cols, new_rows);

            //显示
            Mat image_show = org.clone();
            rectangle(image_show, rect, Scalar(0, 0, 255));
            //imshow("window", image_show);
            //waitKey(0);

            //获取匹配部分的roi图像
            Mat result_img = org.clone();
            Mat result_img_roi = result_img(rect);
            //相似度比较部分：
            //比较相似度的算法很多，各有所长，这里用的是一个灰度值方差的相似度比较
            //variance_diff表示灰度值方差，方差越小，相似度越高；
            double variance_diff = get_variance(result_img_roi, temp_template);

            //默认值为index=0时获取的值；方便与之后的值最比较
            if (index == 0)
            {
                min_diff = variance_diff;
                best_index = index;
                best_rect = rect;
            }
            //当前值与目前的最小方差做比较
            if (variance_diff < min_diff)
            {
                min_diff = variance_diff;
                best_index = index;
                best_rect = rect;
            }
        } //for

        Mat image_show = org.clone();
        rectangle(org, best_rect, Scalar(0, 255, 0), 3);
        imshow("result", org);
        waitKey(1);

        //imshow("video", srcImage);
        //waitKey(1); //每帧延时 1 毫秒，如果不延时，图像将无法显示
    }
    return 0;
}
