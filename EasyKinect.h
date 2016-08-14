#ifndef _LJX_EASYKINECT_H
#define _LJX_EASTKINECT_H

#include <Kinect.h>
#include <Windows.h>
#include <Shlobj.h>
#include <iostream>
using namespace std;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
  if (pInterfaceToRelease != NULL)
  {
    pInterfaceToRelease->Release();
    pInterfaceToRelease = NULL;
  }
}

#if defined (_USE_OPENCV) && !defined(_OPENCV_USED)
#define _OPENCV_USED
#include <opencv2\opencv.hpp>
using namespace cv;

Mat depth2mat(IDepthFrame* depthframe)
{
  IFrameDescription* size = NULL;
  depthframe->get_FrameDescription(&size);
  int height = 0, width = 0;
  size->get_Height(&height);
  size->get_Width(&width);
  SafeRelease(size);
  Mat frame(height, width, CV_16U, Scalar::all(0));
  UINT16* depthbuffer = NULL;
  UINT buffersize = 0;
  if (SUCCEEDED(depthframe->AccessUnderlyingBuffer(&buffersize, &depthbuffer)))
  {
    for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width; j++)
      {
        frame.at<unsigned short>(i, j) = depthbuffer[i*width + j];
      }
    }
  }
  return frame;
}

Mat color2mat(IColorFrame* colorframe)
{
  IFrameDescription* size = NULL;
  colorframe->get_FrameDescription(&size);
  int height = 0, width = 0;
  size->get_Height(&height);
  size->get_Width(&width);
  SafeRelease(size);
  Mat frame(height, width, CV_8UC3, Scalar::all(0));
  static RGBQUAD* colorbuffer = new RGBQUAD[height * width];
  UINT buffersize = height * width * sizeof(RGBQUAD);
  colorframe->CopyConvertedFrameDataToArray(buffersize, reinterpret_cast<BYTE*>(colorbuffer), ColorImageFormat_Bgra);
  if (SUCCEEDED(colorframe))
  {
    for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width; j++)
      {
        frame.at<Vec3b>(i, j)[0] = colorbuffer[i*width + j].rgbBlue;
        frame.at<Vec3b>(i, j)[1] = colorbuffer[i*width + j].rgbGreen;
        frame.at<Vec3b>(i, j)[2] = colorbuffer[i*width + j].rgbRed;
      }
    }
  }
  return frame;
}

Mat infra2mat(IInfraredFrame* infraframe)
{
  IFrameDescription* size = NULL;
  infraframe->get_FrameDescription(&size);
  int height = 0, width = 0;
  size->get_Height(&height);
  size->get_Width(&width);
  SafeRelease(size);
  Mat frame(height, width, CV_16U, Scalar::all(0));
  UINT16* buffer = NULL;
  UINT buffersize = 0;
  if (SUCCEEDED(infraframe->AccessUnderlyingBuffer(&buffersize, &buffer)))
  {
    for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width; j++)
      {
        frame.at<unsigned short>(i, j) = buffer[i*width + j];
      }
    }
  }
  return frame;
}
#endif // _USE_OPENCV

class ljxKinectSensor
{
private:
  // Current Kinect sensor
  IKinectSensor* sensor;
  // Sensor coordinate mapper
  ICoordinateMapper* coordinatemapper;
  // Data stream reader
  IMultiSourceFrameReader* multireader;
  // Using Source
  FrameSourceTypes sourcetypes;
  // Data frame
  IMultiSourceFrame* frame;

public:
  ljxKinectSensor() :
    sensor(NULL),
    coordinatemapper(NULL),
    multireader(NULL),
    sourcetypes(FrameSourceTypes_None),
    frame(NULL)
  {
        
  }

  ~ljxKinectSensor()
  {
    SafeRelease(coordinatemapper);
    SafeRelease(multireader);
    SafeRelease(frame);
    if (sensor != NULL) { sensor->Close(); sensor->Release(); sensor = NULL; }
  }

  HRESULT init(FrameSourceTypes sources)
  {
    HRESULT result;
    result = GetDefaultKinectSensor(&sensor);
    if (FAILED(result) || !sensor) return result;
    result = sensor->get_CoordinateMapper(&coordinatemapper);
    if (FAILED(result) || !coordinatemapper) return result;
    result = sensor->Open();
    if (FAILED(result)) return result;
    result = sensor->OpenMultiSourceFrameReader(sources, &multireader);
    if (FAILED(result) || !multireader) return result;
    sourcetypes = sources;
    return result;
  }

  HRESULT update()
  {
    HRESULT result;
    SafeRelease(frame);
    result = multireader->AcquireLatestFrame(&frame);
    return result;
  }

  HRESULT getDepthFrame(IDepthFrame** depth)
  {
    HRESULT result;
    IDepthFrameReference* depthref = NULL;
    result = frame->get_DepthFrameReference(&depthref);
    if (SUCCEEDED(result))
    {
      result = depthref->AcquireFrame(depth);
      SafeRelease(depthref);
    }
#if defined (_LJX_DEBUG)
    else
      cout << "ljxDebug: Failed to get_DepthReference" << endl;
#endif
    return result;
  }

  HRESULT getColorFrame(IColorFrame** color)
  {
    HRESULT result;
    IColorFrameReference* colorref = NULL;
    result = frame->get_ColorFrameReference(&colorref);
    if (SUCCEEDED(result))
    {
      result = colorref->AcquireFrame(color);
      SafeRelease(colorref);
    }
#if defined (_LJX_DEBUG)
    else
      cout << "ljxDebug: Failed to get_ColorReference" << endl;
#endif
    return result;
  }
  
  HRESULT getBodyFrame(IBodyFrame** body)
  {
    HRESULT result;
    IBodyFrameReference* ref = NULL;
    result = frame->get_BodyFrameReference(&ref);
    if (SUCCEEDED(result))
    {
      result = ref->AcquireFrame(body);
      SafeRelease(ref);
    }
#if defined (_LJX_DEBUG)
    else
      cout << "ljxDebug: Failed to get_BodyReference" << endl;
#endif
    return result;
  }

  HRESULT getInfraredFrame(IInfraredFrame** infra)
  {
    HRESULT result;
    IInfraredFrameReference* ref = NULL;
    result = frame->get_InfraredFrameReference(&ref);
    if (SUCCEEDED(result))
    {
      result = ref->AcquireFrame(infra);
      SafeRelease(ref);
    }
#if defined (_LJX_DEBUG)
    else
      cout << "ljxDebug: Failed to get_InfrafedReference" << endl;
#endif
    return result;
  }

#ifdef _USE_OPENCV
  Mat getDepthMat()
  {
    HRESULT result;
    IDepthFrame* getframe = NULL;
    result = getDepthFrame(&getframe);
    if (SUCCEEDED(result))
    {
      Mat mat = depth2mat(getframe);
      SafeRelease(getframe);
      return mat;
    }
    return Mat();
  }

  Mat getColorMat()
  {
    HRESULT result;
    IColorFrame* getframe = NULL;
    result = getColorFrame(&getframe);
    if (SUCCEEDED(result))
    {
      Mat mat = color2mat(getframe);
      SafeRelease(getframe);
      return mat;
    }
    return Mat();
  }

  Mat getInfraredMat()
  {
    HRESULT result;
    IInfraredFrame* getframe = NULL;
    result = getInfraredFrame(&getframe);
    if (SUCCEEDED(result))
    {
      Mat mat = infra2mat(getframe);
      SafeRelease(getframe);
      return mat;
    }
    return Mat();
  }

#endif // _USE_OPENCV
};


#endif //_LJX_EASYKINECT_H