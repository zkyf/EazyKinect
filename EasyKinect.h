#ifndef _LJX_EASYKINECT_H
#define _LJX_EASTKINECT_H

#include <Kinect.h>
#include <Windows.h>
#include <Shlobj.h>
#include <iostream>
#include <NuiKinectFusionApi.h>
using namespace std;

#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if (p) { delete (p); (p)=NULL; } }
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(p) { if (p) { delete[] (p); (p)=NULL; } }
#endif

#ifndef SAFE_FUSION_RELEASE_IMAGE_FRAME
#define SAFE_FUSION_RELEASE_IMAGE_FRAME(p) { if (p) { static_cast<void>(NuiFusionReleaseImageFrame(p)); (p)=NULL; } }
#endif

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

Mat bodyindex2mat(IBodyIndexFrame* bodyindex)
{
	IFrameDescription* size = NULL;
	bodyindex->get_FrameDescription(&size);
	int height = 0, width = 0;
	size->get_Height(&height);
	size->get_Width(&width);
	SafeRelease(size);
	Mat frame(height, width, CV_8U, Scalar::all(0));
	BYTE* buffer = NULL;
	UINT buffersize = 0;
	if (SUCCEEDED(bodyindex->AccessUnderlyingBuffer(&buffersize, &buffer)))
	{
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				frame.at<uchar>(i, j) = buffer[i*width + j];
			}
		}
		return frame;
	}
#ifdef _LJX_DEBUG
	else
		cout << "bodyindex->AccessUnderlyingBuffer failed." << endl;
#endif
	return Mat();
}


#endif // _USE_OPENCV

class KinectSensor
{
	friend class KinectFusion;

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
	// Status
	bool running;

	KinectSensor() :
    sensor(NULL),
    coordinatemapper(NULL),
    multireader(NULL),
    sourcetypes(FrameSourceTypes_None),
    frame(NULL),
		running(false)
  {
        
  }

  ~KinectSensor()
  {
    SafeRelease(coordinatemapper);
    SafeRelease(multireader);
    SafeRelease(frame);
		running = false;
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
		running = true;
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
      cout << "ljxDebug: Failed to get DepthReference" << endl;
#endif
    return result;
  }

	HRESULT getBodyIndexFrame(IBodyIndexFrame** bodyindex)
	{
		HRESULT result;
		IBodyIndexFrameReference* bodyref = NULL;
		result = frame->get_BodyIndexFrameReference(&bodyref);
		if (SUCCEEDED(result))
		{
			result = bodyref->AcquireFrame(bodyindex);
			SafeRelease(bodyref);
		}
#if defined (_LJX_DEBUG)
		else
			cout << "ljxDebug: Failed to get BodyIndexFrame" << endl;
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
      cout << "ljxDebug: Failed to get ColorReference" << endl;
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
      cout << "ljxDebug: Failed to get BodyReference" << endl;
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
      cout << "ljxDebug: Failed to get InfrafedReference" << endl;
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
	/// Get the body index frame and store it to a CV_8U Mat class
	/// </summary>
	/// <returns>Pointer to a pointer to store the body index frame </returns>
	Mat getBodyIndexMat()
	{
		HRESULT result;
		IBodyIndexFrame* getframe = NULL;
		result = getBodyIndexFrame(&getframe);
		if (SUCCEEDED(result))
		{
			Mat mat = bodyindex2mat(getframe);
			SafeRelease(getframe);
			return mat;
		}
#ifdef _LJX_DEBUG
		cout << "getBodyIndexFrame failed" << endl;
#endif
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


#pragma region KinectFusion

/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}

class KinectFusion
{

public:
	NUI_FUSION_RECONSTRUCTION_PARAMETERS fusionParameters;
	INuiFusionReconstruction* volume;
	Matrix4* worldToCameraTransform;
	Matrix4 defaultWorldToVolumeTransform;
	UINT16* depthPixelBuffer;
	NUI_FUSION_IMAGE_FRAME* depthFloatImage;
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE processorType;
	NUI_FUSION_CAMERA_PARAMETERS cameraParameters;
	WAITABLE_HANDLE coordinateMapChanged;
	DepthSpacePoint* depthDistortMap;
	UINT* depthDistortLT;
	bool cameraParametersValid;

public:
	NUI_FUSION_IMAGE_FRAME* pointCloud;
	NUI_FUSION_IMAGE_FRAME* shadedSurface;

	KinectFusion(
		int sourceCount = 1,
		NUI_FUSION_RECONSTRUCTION_PARAMETERS _fusionParameters = { 256, 384, 384, 384 },
		NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE _processorType = NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP) :
		processorType(_processorType),
		fusionParameters(_fusionParameters),
		volume(NULL),
		depthPixelBuffer(NULL),
		depthFloatImage(NULL),
		pointCloud(NULL)
	{
		cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
		cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
		cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
		cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;
		worldToCameraTransform = new Matrix4[sourceCount];
		for (int i = 0; i < sourceCount; i++)
		{
			SetIdentityMatrix(worldToCameraTransform[i]);
		}
		SetIdentityMatrix(defaultWorldToVolumeTransform);
	}

	~KinectFusion()
	{
		SafeRelease(volume);
		SAFE_DELETE_ARRAY(depthPixelBuffer);
		SAFE_DELETE_ARRAY(depthDistortLT);
		SAFE_DELETE_ARRAY(depthDistortMap);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(depthFloatImage);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(pointCloud);
		SAFE_FUSION_RELEASE_IMAGE_FRAME(shadedSurface);
	}

	HRESULT init()
	{
		//if (!sensor) return E_POINTER;
		//if (!sensor->running) return E_ACCESSDENIED;
		HRESULT hr = S_OK;
		WCHAR description[MAX_PATH];
		WCHAR instancePath[MAX_PATH];
		UINT memorySize = 0;
		if (FAILED(hr = NuiFusionGetDeviceInfo(
			processorType, -1,
			&description[0], ARRAYSIZE(description),
			&instancePath[0], ARRAYSIZE(instancePath), &memorySize)))
		{
			cout << "0x" << hex << hr << ": ";
			if (hr == E_NUI_BADINDEX)
			{
				cout << "No DirectX11 device detected, or invalid device index - Kinect Fusion requires a DirectX11 device for GPU-based reconstruction." << endl;
			}
			else
			{
				cout << "Failed in call to NuiFusionGetDeviceInfo." << endl;
			}
			cout << "description: " << description << endl;
			cout << "instancePath: " << instancePath << endl;
			return hr;
		}
		hr = NuiFusionCreateReconstruction(&fusionParameters, processorType, -1, &worldToCameraTransform[0], &volume);
		if (FAILED(hr)) return hr;

		hr = volume->GetCurrentWorldToVolumeTransform(&defaultWorldToVolumeTransform);
		if (FAILED(hr)) return hr;

		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, NUI_DEPTH_RAW_WIDTH, NUI_DEPTH_RAW_HEIGHT, nullptr, &depthFloatImage);
		if (FAILED(hr)) return hr;

		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, NUI_DEPTH_RAW_WIDTH, NUI_DEPTH_RAW_HEIGHT, &cameraParameters, &pointCloud);
		if (FAILED(hr)) return hr;

		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, NUI_DEPTH_RAW_WIDTH, NUI_DEPTH_RAW_HEIGHT, nullptr, &shadedSurface);
		if (FAILED(hr)) return hr;

		depthPixelBuffer = new(std::nothrow) UINT16[NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT];
		depthDistortMap = new(std::nothrow) DepthSpacePoint[NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT];
		depthDistortLT = new(std::nothrow) UINT[NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT];
		volume->ResetReconstruction(&worldToCameraTransform[0], &defaultWorldToVolumeTransform);
		return hr;
	}

	HRESULT ProcessDepth(IDepthFrame* depthFrame, int depthSource = 0)
	{
		UINT16* buffer = NULL;
		UINT buffersize = 0;
		HRESULT hr;
		hr = depthFrame->AccessUnderlyingBuffer(&buffersize, &buffer);
		return ProcessDepth(buffer, depthSource);
	}

	HRESULT ProcessDepth(UINT16* depthFrame, int depthSource = 0)
	{
		if (nullptr == depthFrame) return E_POINTER;
		
		HRESULT hr = S_OK;

		hr = volume->DepthToDepthFloatFrame(depthFrame, NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT * sizeof(UINT16), depthFloatImage, NUI_FUSION_DEFAULT_MINIMUM_DEPTH, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH, false);
		if (FAILED(hr))
		{
			cout << "Failed to convert depth frame to depthFloatFrame" << endl;
			return hr;
		}

		hr = volume->ProcessFrame(depthFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, nullptr, &worldToCameraTransform[depthSource]);
		if (FAILED(hr))
		{
			cout << "ProcessFrame failed" << endl;
			return hr;
		}

		Matrix4 calculatedCameraPose;
		hr = volume->GetCurrentWorldToCameraTransform(&calculatedCameraPose);
		if (FAILED(hr))
		{
			cout << "GetCurrentWorldToCameraTransform Failed" << endl;
			return hr;
		}

		worldToCameraTransform[depthSource] = calculatedCameraPose;
		volume->CalculatePointCloud(pointCloud, &worldToCameraTransform[depthSource]);
		if (FAILED(hr))
		{
			cout << "CalculatePointCloud Failed" << endl;
			return hr;
		}

		hr = NuiFusionShadePointCloud(pointCloud, &worldToCameraTransform[depthSource], nullptr, shadedSurface, nullptr);
		
		if (FAILED(hr))
		{
			cout << "0x" << hex << hr << ": NuiFusionShadePointCloud Failed" << endl;
			return hr;
		}

		return hr;
	}
};
#pragma endregion


#endif //_LJX_EASYKINECT_H