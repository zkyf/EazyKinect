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
    (FrameSourceTypes)(FrameSourceTypes_Depth |
    FrameSourceTypes_Color |
    FrameSourceTypes_Infrared |
    FrameSourceTypes_LongExposureInfrared));
  if (FAILED(result))
  {
    cout << "Sensor init failed!" << endl; 
    system("pause");
    return 1;
  }
  int i = 0;
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

    Mat depthmat = sensor.getDepthMat();
    if(!depthmat.empty()) imshow("Depth", depthmat);

    Mat colormat = sensor.getColorMat();
    if (!colormat.empty()) imshow("Color", colormat);

    Mat inframat = sensor.getInfraredMat();
    if (!inframat.empty()) imshow("Infrared", inframat);

    i++;
    cout << "Frame #" << i << endl;
    waitKey(1);
  }
}