#define _USE_OPENCV
#define _LJX_DEBUG
#include "EasyKinect.h"
#include <opencv2\opencv.hpp>
#include <math.h>
using namespace cv;
#include <iostream>
using namespace std;
#include <Windows.h>
#include <stdio.h>
#include "KinectOpenCvTools.h"

class myVideoWriter : public VideoWriter
{
public:
	~myVideoWriter()
	{
		if (isOpened())
			release();
	}
};

int main()
{
  ljxKinectSensor sensor;
  HRESULT result = sensor.init((FrameSourceTypes)(FrameSourceTypes_BodyIndex | FrameSourceTypes_Depth));
  if (FAILED(result))
  {
    cout << "Sensor init failed!" << endl; 
    return 1;
  }
	myVideoWriter videoWriter;

	while (1)
	{
		result = sensor.update();
		if (FAILED(result))
		{
			waitKey(10);
			continue;
		}
		//Mat inframat = sensor.getInfraredMat();
		//Mat depthmat = sensor.getDepthMat();
		//if (inframat.empty() || depthmat.empty())
		//{
		//	continue;
		//}
		//if (!videoWriter.isOpened())
		//{
		//	videoWriter.open("infrared.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, inframat.size());
		//}
		//Mat combinedMat = infraDepth2Mat(inframat, depthmat);
		//imshow("Combined Mat", combinedMat);
		//videoWriter << combinedMat;
		Mat index = sensor.getBodyIndexMat();
		Mat depthmat = sensor.getDepthMat();
		if (depthmat.empty() || index.empty())
		{
			continue;
		}
		Mat toshow(index.size(), CV_8UC3, Scalar::all(255));
		for (int i = 0; i < index.rows; i++)
		{
			for (int j = 0; j < index.cols; j++)
			{
				if (index.at<uchar>(i, j) < 6)
				{
					toshow.at<Vec3b>(i, j)[0] = 255;
					toshow.at<Vec3b>(i, j)[1] = 255 - depthmat.at<unsigned short>(i, j) % 256;
					toshow.at<Vec3b>(i, j)[2] = 255 - depthmat.at<unsigned short>(i, j) % 256;
				}
			}
		}
		imshow("toshow", toshow);
		char key = waitKey(10);
		if (key == 'q')
		{
			break;
		}
	}
	videoWriter.release();
}