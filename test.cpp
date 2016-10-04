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
#include "test.h"

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
  HRESULT result = sensor.init((FrameSourceTypes)(FrameSourceTypes_Infrared | FrameSourceTypes_Depth));
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
		Mat inframat = sensor.getInfraredMat();
		Mat depthmat = sensor.getDepthMat();
		if (inframat.empty() || depthmat.empty())
		{
			continue;
		}
		if (!videoWriter.isOpened())
		{
			videoWriter.open("infrared.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, inframat.size());
		}
		Mat combinedMat = infraDepth2Mat(inframat, depthmat);
		imshow("Combined Mat", combinedMat);
		videoWriter << combinedMat;
		char key = waitKey(10);
		if (key == 'q')
		{
			break;
		}
	}
	videoWriter.release();
}