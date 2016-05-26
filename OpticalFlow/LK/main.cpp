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
	vector<Point2f> points[2];//ǰ����֡��������
	vector<Point2f> initial;//��ʼ������
	vector<Point2f> features;//��⵽������
    vector<uchar> status;//�����㱻�ɹ����ٵı�־
    vector<float> err;//����ʱ��������С��������  
    int maxCout = 1000;//Ҫ���������������Ŀ
    double minDis = 5;//������֮����С���̾���
    double qLevel = 0.01;//��������ָ��

	//RNG rng(0xFFFFFFFF);

	Mat motion2color;

    for(;;)
    {
		//double t = (double)cvGetTickCount();

        cap >> frame;
		//�õ��Ҷ�ͼ
        cvtColor(frame, gray, CV_BGR2GRAY);
		frame.copyTo (output);
		if(frame.data)imshow("original", frame);

		//����������Ŀ����10����������������
		if(gray.data&&points[0].size()<=0)
		{
			goodFeaturesToTrack(gray,features,maxCout,qLevel,minDis);
			points[0].insert (points[0].end (),features.begin (),features.end ());
			initial.insert (initial.end (),features.begin (),features.end ());
		}

        if( prevgray.data )
        {
			calcOpticalFlowPyrLK(
				prevgray,//ǰһ֡�Ҷ�ͼ
				gray,//��ǰ֡�Ҷ�ͼ
				points[0],//ǰһ֡������λ��
				points[1],//��ǰ֡������λ��
				status,//�����㱻�ɹ����ٵı�־
				err);//ǰһ֡�������С����͵�ǰ������С�����Ĳ���ݲ�Ĵ�С��ɾ����Щ�˶��仯���ҵĵ�
			//�б���Щ�����˶���������
			int k = 0;
			for(int i=0;i<points[1].size();i++)
			{
				//״̬Ҫ��1����������Ҫ�ƶ��µ���Щ��
				if(status[i]&&(abs(points[0][i].x-points[1][i].x)+
                  abs(points[0][i].y-points[1][i].y) >2))//����������ǰ����֡�ƶ��ˣ�����Ϊ�õ���Ŀ��㣬�ҿɱ�����
				{
					initial[k]=initial[i];
					points[1][k++] = points[1][i];
				}
			}
			if(k == 0);

			points[1].resize(k);//��ȡ
			initial.resize (k);
			for(int i=0;i<points[1].size ();i++){

                //��ǰ�����㵽��ʼλ����ֱ�߱�ʾ
                line(output,initial[i],points[1][i],CV_RGB(176,224,230) );
                //��ǰλ����Ȧ���
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