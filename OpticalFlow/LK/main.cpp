// API calcOpticalFlowFarneback() comes from OpenCV, and this
// 2D dense optical flow algorithm from the following paper:
// Gunnar Farneback. "Two-Frame Motion Estimation Based on Polynomial Expansion".
// And the OpenCV source code locate in ..\opencv2.4.3\modules\video\src\optflowgf.cpp

#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

static Scalar random_color(RNG& rng)
{
    int icolor = (unsigned)rng;

    return Scalar(icolor&0xFF, (icolor>>8)&0xFF, (icolor>>16)&0xFF);
}
int main(int, char**)
{
    VideoCapture cap;
	cap.open(0);
	//cap.open("E://OpenCV_Projects//OpticalFlow//test1.avi");

    if( !cap.isOpened() )
        return -1;

    Mat prevgray, gray, frame,output;
	vector<Point2f> points[2];//前后两帧的特征点
	vector<Point2f> initial;//初始特征点
	vector<Point2f> features;//检测到的特征
    vector<uchar> status;//特征点被成功跟踪的标志
    vector<float> err;//跟踪时的特征点小区域误差和  
    int maxCout = 1000;//要跟踪特征的最大数目
    double minDis = 5;//特征点之间最小容忍距离
    double qLevel = 0.01;//特征检测的指标

	//RNG rng(0xFFFFFFFF);

	Mat motion2color;

    for(;;)
    {
		//double t = (double)cvGetTickCount();

        cap >> frame;
		//得到灰度图
        cvtColor(frame, gray, CV_BGR2GRAY);
		frame.copyTo (output);
		if(frame.data)imshow("original", frame);

		//若特征点数目少于10，则决定添加特征点
		if(gray.data&&points[0].size()<=0)
		{
			goodFeaturesToTrack(gray,features,maxCout,qLevel,minDis);
			points[0].insert (points[0].end (),features.begin (),features.end ());
			initial.insert (initial.end (),features.begin (),features.end ());
		}

        if( prevgray.data )
        {
			calcOpticalFlowPyrLK(
				prevgray,//前一帧灰度图
				gray,//当前帧灰度图
				points[0],//前一帧特征点位置
				points[1],//当前帧特征点位置
				status,//特征点被成功跟踪的标志
				err);//前一帧特征点点小区域和当前特征点小区域间的差，根据差的大小可删除那些运动变化剧烈的点
			//判别哪些属于运动的特征点
			int k = 0;
			for(int i=0;i<points[1].size();i++)
			{
				//状态要是1，并且坐标要移动下的那些点
				if(status[i]&&(abs(points[0][i].x-points[1][i].x)+
                  abs(points[0][i].y-points[1][i].y) >2))//若特征点在前后两帧移动了，则认为该点是目标点，且可被跟踪
				{
					initial[k]=initial[i];
					points[1][k++] = points[1][i];
				}
			}
			if(k == 0);

			points[1].resize(k);//截取
			initial.resize (k);
			for(int i=0;i<points[1].size ();i++){

                //当前特征点到初始位置用直线表示
                line(output,initial[i],points[1][i],CV_RGB(176,224,230) );
                //当前位置用圈标出
                circle(output,points[1][i],3,CV_RGB(242, 156, 177),(-1));
            }	
        }
        if(waitKey(10)>=0)
            break;
		std::swap(points[1],points[0]);
        cv::swap(prevgray, gray);


		if(output.data)imshow("features",output);
		//t = (double)cvGetTickCount() - t;
		//cout << "cost time: " << t / ((double)cvGetTickFrequency()*1000.) << endl;
    }
	prevgray.release();
	gray.release();
	frame.release();
	output.release();
    return 0;
}