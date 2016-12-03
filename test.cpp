#define _USE_OPENCV
#define _LJX_DEBUG
#include "EasyKinect.h"
#include <opencv2\opencv.hpp>
#include <math.h>
using namespace cv;
#include <iostream>
using namespace std;

int main()
{
  ljxKinectSensor sensor;
  HRESULT result = sensor.init(
    (FrameSourceTypes)(FrameSourceTypes_Color));
  if (FAILED(result))
  {
    cout << "Sensor init failed!" << endl; 
    system("pause");
    return 1;
  }
  int i = 0;
	VideoWriter videoWriter;
	videoWriter.open("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(960, 540));
  while (1)
  {
    result = sensor.update();
    if (FAILED(result))
    {
      //cout << "Failed to update" << endl;
      continue;
      //system("pause");
      //return 2;
    }
    Mat colormat = sensor.getColorMat();
		if (!colormat.empty() && videoWriter.isOpened())
		{
			Mat haha(960, 540, CV_8UC3, Scalar::all(255));
			resize(colormat, colormat, Size(960, 540));
			videoWriter << haha;
			imshow("Color", colormat);
		}

    i++;
    cout << "Frame #" << i << endl;
    waitKey(1);
  }
	return 0;
}