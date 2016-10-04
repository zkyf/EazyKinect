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

/// <summary>
/// Convert depthframe in IDepthFrame* to a CV_16U Mat.
/// </summary>
/// <param name="depthframe">The pointer to the obtained depth frame</param>
/// <returns>Returns a Mat in CV_16U containing the depth frame</returns>
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

/// <summary>
/// Convert colorframe in IColorFrame* to a CV_8UC3 Mat.
/// </summary>
/// <param name="colorframe">The pointer to the obtained color frame</param>
/// <returns>Returns a Mat in CV_8UC3 containing the color frame</returns>
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

/// <summary>
/// Convert infraredframe in IInfraredFrame* to a CV_16U Mat.
/// </summary>
/// <param name="infraframe">The pointer to the obtained infrared frame</param>
/// <returns>Returns a Mat in CV_16U containing the infrared frame</returns>
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

/// <summary>
/// Combine CV_16U infrared frame and CV_16U depth frame into a CV_8UC3 mat
/// </summary>
/// <param name="inframat">The Mat structure containing infrared frame</param>
/// <param name="depthmat">The Mat structure containing depth frame</param>
/// <returns>Returns a Mat in CV_8UC3 containing the combined frame</returns>
Mat infraDepth2Mat(Mat inframat, Mat depthmat)
{
	if (inframat.size() != depthmat.size())
		return Mat();
	Size size = inframat.size();
	Mat result(size, CV_8UC3, Scalar::all(0));
	for (int i = 0; i < size.height; i++)
	{
		for (int j = 0; j < size.width; j++)
		{
			result.at<Vec3b>(i, j)[0] = inframat.at<unsigned short>(i, j) / 256;
			result.at<Vec3b>(i, j)[1] = depthmat.at<unsigned short>(i, j) / 256 * 50;
			result.at<Vec3b>(i, j)[2] = depthmat.at<unsigned short>(i, j) % 256;
		}
	}
	return result;
}

/// <summary>
/// Combine CV_16U infrared frame and CV_16U depth frame into a CV_8UC3 mat
/// </summary>
/// <param name="inframat">The Mat structure containing infrared frame</param>
/// <param name="depthmat">The Mat structure containing depth frame</param>
/// <returns>Returns a Mat in CV_8UC3 containing the combined frame</returns>
Mat infraDepth2Mat(Mat inframat, Mat depthmat)
{
	if (inframat.size() != depthmat.size())
		return Mat();
	Size size = inframat.size();
	Mat result(size, CV_8UC3, Scalar::all(0));
	for (int i = 0; i < size.height; i++)
	{
		for (int j = 0; j < size.width; j++)
		{
			result.at<Vec3b>(i, j)[0] = inframat.at<unsigned short>(i, j) / 256;
			result.at<Vec3b>(i, j)[1] = depthmat.at<unsigned short>(i, j) / 256 * 50;
			result.at<Vec3b>(i, j)[2] = depthmat.at<unsigned short>(i, j) % 256;
		}
	}
	return result;
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

	void close()
	{
		SafeRelease(coordinatemapper);
		SafeRelease(multireader);
		SafeRelease(frame);
		if (sensor != NULL) { sensor->Close(); sensor->Release(); sensor = NULL; }
	}

	/// <summary>
	/// Initialize the Kinect sensor using the default sensor
	/// </summary>
	/// <param name="sources">The data streams to collect from the sensor. </param>
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

	/// <summary>
	/// Update the data streams specified.
	/// </summary>
  HRESULT update()
  {
    HRESULT result;
    SafeRelease(frame);
    result = multireader->AcquireLatestFrame(&frame);
    return result;
  }

	/// <summary>
	/// Get the depth frame and store it to the pointer passed as parameter
	/// </summary>
	/// <param name="depth">Pointer to a pointer to store the depth frame </param>
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

	/// <summary>
	/// Get the color frame and store it to the pointer passed as parameter
	/// </summary>
	/// <param name="depth">Pointer to a pointer to store the color frame </param>
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
  
	/// <summary>
	/// Get the body frame and store it to the pointer passed as parameter
	/// </summary>
	/// <param name="depth">Pointer to a pointer to store the body frame </param>
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

	/// <summary>
	/// Get the infra frame and store it to the pointer passed as parameter
	/// </summary>
	/// <param name="depth">Pointer to a pointer to store the infra frame </param>
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
	/// <summary>
	/// Get the depth frame and store it to a CV_16U Mat class
	/// </summary>
	/// <returns>Pointer to a pointer to store the depth frame </returns>
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

	/// <summary>
	/// Get the color frame and store it to a CV_8UC3 Mat class
	/// </summary>
	/// <returns>Pointer to a pointer to store the color frame </returns>
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

	/// <summary>
	/// Get the infrared frame and store it to a CV_16U Mat class
	/// </summary>
	/// <returns>Pointer to a pointer to store the infrared frame </returns>
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