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
#include "MatStream.h"

KinectSensor sensor;
VideoWriter videoWriter;

void WriteVideo()
{
	Mat inframat = sensor.getInfraredMat();
	Mat depthmat = sensor.getDepthMat();
	if (inframat.empty() || depthmat.empty())
	{
		return;
	}
	if (!videoWriter.isOpened())
	{
		videoWriter.open("1.avi", CV_FOURCC('D', 'I', 'V', 'X'), 30.0, inframat.size());
	}
	Mat combinedMat = infraDepth2Mat(inframat, depthmat);
	imshow("Combined Mat", combinedMat);
	videoWriter << combinedMat;
	char key = waitKey(10);
}

int main()
{
	
  HRESULT result = sensor.init((FrameSourceTypes)(FrameSourceTypes_Depth | FrameSourceTypes_Infrared));
	MatStream stream;
	MatStreamHeader head;
	head.bytesPerPixel = 2;
	head.channels = 1;
	head.height = NUI_DEPTH_RAW_HEIGHT;
	head.width = NUI_DEPTH_RAW_WIDTH;
	head.type = CV_16U;
	head.time = 30;
	stream.SetHead(head);
	stream.Open("test.cvmat", MatStream::Op::out);
	if (stream.Fail())
	{
		cout << "Failed to open stream" << endl;
		system("pause");
		return 0;
	}
  if (FAILED(result))
  {
    cout << "Sensor init failed!" << endl; 
    return 1;
  }
	KinectFusion fusion;
	result = fusion.init();
	if (FAILED(result))
	{
		cout << "Kinect Fusion init failed!" << endl;
		system("pause");
		return 2;
	}
	int framecount = 0;
	while (framecount<60)
	{
		result = sensor.update();
		if (FAILED(result))
		{
			waitKey(10);
			continue;
		}
		cout << "Frame #" << framecount++ << endl;

		Mat depthframe = sensor.getDepthMat();
		//fusion.ProcessDepth((UINT16*)depthframe.data);
		stream.Write(depthframe);

		char key = waitKey(10);
		if (key == 'q')
		{
			break;
		}
	}
	stream.Close();

	stream.Open("test.cvmat", MatStream::Op::in);
	if (stream.Fail())
	{
		cout << "Failed to open stream" << endl;
		system("pause");
		return 0;
	}
	cout << "In total: " << stream.FrameNum() << " frames" << endl;
	head = stream.GetHead();
	cout << "Height: " << head.height << endl;
	cout << "Width: " << head.width << endl;
	cout << "Bytes per pixel: " << head.bytesPerPixel << endl;
	cout << "Channels: " << head.channels << endl;
	cout << "Frame time(ms): " << head.time << endl;


	if (FAILED(result))
	{
		cout << "Failed to init Kinect Fusion" << endl;
		system("pause");
		return 0;
	}
	for (int i = 0; i < stream.FrameNum(); i++)
	{
		Mat frame = stream.Read();
		fusion.ProcessDepth((UINT16*)frame.data);
		Mat shaded(NUI_DEPTH_RAW_HEIGHT, NUI_DEPTH_RAW_WIDTH, CV_8UC4, Scalar::all(0));
		for (int j = 0; j < NUI_DEPTH_RAW_HEIGHT; j++)
		{
			for (int k = 0; k < NUI_DEPTH_RAW_WIDTH; k++)
			{
				for (int l = 0; l < 4; l++)
				{
					shaded.at<Vec4b>(j, k)[l] = fusion.shadedSurface->pFrameBuffer->pBits[(j*NUI_DEPTH_RAW_WIDTH + k) * 4 + l];
				}
			}
		}
		imshow("shaded", shaded);
		waitKey(head.time);
	}
}