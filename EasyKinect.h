#pragma once

#ifndef _LJX_EASYKINECT_H
#define _LJX_EASTKINECT_H

#include <Kinect.h>
#include <Windows.h>
#include <Shlobj.h>
#include <iostream>
#include <NuiKinectFusionApi.h>
#include <iomanip>
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
Mat depth2mat(IDepthFrame* depthframe);

/// <summary>
/// Convert colorframe in IColorFrame* to a CV_8UC3 Mat.
/// </summary>
/// <param name="colorframe">The pointer to the obtained color frame</param>
/// <returns>Returns a Mat in CV_8UC3 containing the color frame</returns>
Mat color2mat(IColorFrame* colorframe);

/// <summary>
/// Convert infraredframe in IInfraredFrame* to a CV_16U Mat.
/// </summary>
/// <param name="infraframe">The pointer to the obtained infrared frame</param>
/// <returns>Returns a Mat in CV_16U containing the infrared frame</returns>
Mat infra2mat(IInfraredFrame* infraframe);

/// <summary>
/// Convert infraredframe in IInfraredFrame* to a CV_16U Mat.
/// </summary>
/// <param name="infraframe">The pointer to the obtained infrared frame</param>
/// <returns>Returns a Mat in CV_16U containing the infrared frame</returns>
Mat longinfra2mat(ILongExposureInfraredFrame* infraframe);

Mat bodyindex2mat(IBodyIndexFrame* bodyindex);


#endif // _USE_OPENCV

struct KinectBody
{
	Joint joints[JointType_Count];
	HandState left;
	HandState right;
	INT64 time;
	BOOLEAN tracked;
};

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

  enum BoneType
  {
    BoneType_Head_Neck                   = 0,
    BoneType_Neck_SpineShoulder          = 1,

    BoneType_SpineShoulder_ShoulderRight = 2,
    BoneType_ShoulderRight_ElbowRight    = 3,
    BoneType_ElbowRight_WristRight       = 4,
    BoneType_WristRight_HandRight        = 5,
    BoneType_HandRight_ThumbRight        = 6,
    BoneType_HandRight_HandTipRight      = 7,

    BoneType_SpineShoulder_ShoulderLeft  = 8,
    BoneType_ShoulderLeft_ElbowLeft      = 9,
    BoneType_ElbowLeft_WristLeft         = 10,
    BoneType_WristLeft_HandLeft          = 11,
    BoneType_HandLeft_ThumbLeft          = 12,
    BoneType_HandLeft_HandTipLeft        = 13,

    BoneType_SpineShoulder_SpineMid      = 14,
    BoneType_SpineMid_SpineBase          = 15,

    BoneType_SpineBase_HipRight          = 16,
    BoneType_HipRight_KneeRight          = 17,
    BoneType_KneeRight_AnkleRight        = 18,
    BoneType_AnkleRight_FootRight        = 19,

    BoneType_SpineBase_HipLeft           = 20,
    BoneType_HipLeft_KneeLeft            = 21,
    BoneType_KneeLeft_AnkleLeft          = 22,
    BoneType_AnkleLeft_FootLeft          = 23,

    BoneType_Count = (BoneType_AnkleLeft_FootLeft+1)
  };

  static JointType BoneFrom[BoneType_Count];

  static JointType BoneTo[BoneType_Count];

  static int Connected[JointType_Count][JointType_Count];

  static string JointName[JointType_Count];

  static string BoneName[BoneType_Count];

	KinectSensor() :
		sensor(NULL),
		coordinatemapper(NULL),
		multireader(NULL),
		sourcetypes(FrameSourceTypes_None),
		frame(NULL),
		running(false)
	{
    for(int i=0; i<JointType_Count; i++)
    {
      for(int j=0; j<JointType_Count; j++)
      {
        Connected[i][j]=-1;
      }
    }
    for(int i=0; i<BoneType_Count; i++)
    {
      Connected[BoneFrom[i]][BoneTo[i]] = i;
      Connected[BoneTo[i]][BoneFrom[i]] = i;
    }
	}

	~KinectSensor()
	{
		SafeRelease(coordinatemapper);
		SafeRelease(multireader);
		SafeRelease(frame);
		running = false;
		if (sensor != NULL) { sensor->Close(); sensor->Release(); sensor = NULL; }
	}

	ICoordinateMapper* getMapper()
	{
		return coordinatemapper;
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
	HRESULT getDepthFrame(IDepthFrame** depth, INT64* time = nullptr)
	{
		HRESULT result;
		IDepthFrameReference* depthref = NULL;
		result = frame->get_DepthFrameReference(&depthref);
		if (SUCCEEDED(result))
		{
			result = depthref->AcquireFrame(depth);
			if (SUCCEEDED(result) && time != nullptr)
			{
				result = depthref->get_RelativeTime(time);
			}
			SafeRelease(depthref);
		}
#if defined (_LJX_DEBUG)
		else
			cout << "ljxDebug: Failed to get DepthReference" << endl;
#endif
		return result;
	}

	HRESULT getBodyIndexFrame(IBodyIndexFrame** bodyindex, INT64* time = nullptr)
	{
		HRESULT result;
		IBodyIndexFrameReference* bodyref = NULL;
		result = frame->get_BodyIndexFrameReference(&bodyref);
		if (SUCCEEDED(result))
		{
			result = bodyref->AcquireFrame(bodyindex);
			if (SUCCEEDED(result) && time != nullptr)
			{
				result = bodyref->get_RelativeTime(time);
			}
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
	HRESULT getColorFrame(IColorFrame** color, INT64* time = nullptr)
	{
		HRESULT result;
		IColorFrameReference* colorref = NULL;
		result = frame->get_ColorFrameReference(&colorref);
		if (SUCCEEDED(result))
		{
			result = colorref->AcquireFrame(color);
			if (SUCCEEDED(result) && time != nullptr)
			{
				result = colorref->get_RelativeTime(time);
			}
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
	HRESULT getBodyFrame(IBodyFrame** body, INT64* time = nullptr)
	{
    if(!body)
    {
      return -1;
    }
		HRESULT result;
		IBodyFrameReference* ref = NULL;
		result = frame->get_BodyFrameReference(&ref);
		if (SUCCEEDED(result))
		{
			result = ref->AcquireFrame(body);
			if (SUCCEEDED(result) && time != nullptr)
			{
				result = ref->get_RelativeTime(time);
			}
			SafeRelease(ref);
		}
#if defined (_LJX_DEBUG)
		else
			cout << "ljxDebug: Failed to get BodyReference" << endl;
#endif
		return result;
	}

	HRESULT getKBodyFrame(KinectBody bodies[])
  {
		HRESULT result;
    IBodyFrame* frame = NULL;
		result = getBodyFrame(&frame);
		if (FAILED(result))
		{
      SafeRelease(frame);
			return result;
		}
    INT64 time = 0;
		result = frame->get_RelativeTime(&time);
		if (FAILED(result))
		{
      SafeRelease(frame);
			return result;
		}
    IBody* body[BODY_COUNT] = { 0 };
    result = frame->GetAndRefreshBodyData(BODY_COUNT, body);
		for (int i = 0; i < BODY_COUNT; i++)
		{
      bodies[i].tracked = FALSE;
			result = body[i]->get_IsTracked(&bodies[i].tracked);
			if (SUCCEEDED(result) && bodies[i].tracked)
			{
//        cout << "body " << i << "is tracked" << endl;
				bodies[i].time = time;
				body[i]->get_HandLeftState(&bodies[i].left);
				body[i]->get_HandRightState(&bodies[i].right);
				result = body[i]->GetJoints(_countof(bodies[i].joints), bodies[i].joints);
			}
      SafeRelease(body[i]);
		}
    SafeRelease(frame);
    return result;
	}

	/// <summary>
	/// Get the infra frame and store it to the pointer passed as parameter
	/// </summary>
	/// <param name="depth">Pointer to a pointer to store the infra frame </param>
	HRESULT getInfraredFrame(IInfraredFrame** infra, INT64* time = nullptr)
	{
		HRESULT result;
		IInfraredFrameReference* ref = NULL;
		result = frame->get_InfraredFrameReference(&ref);
		if (SUCCEEDED(result))
		{
			result = ref->AcquireFrame(infra);
			if (SUCCEEDED(result) && time != nullptr)
			{
				result = ref->get_RelativeTime(time);
			}
			SafeRelease(ref);
		}
#if defined (_LJX_DEBUG)
		else
			cout << "ljxDebug: Failed to get InfrafedReference" << endl;
#endif
		return result;
	}

	/// <summary>
	/// Get the infra frame and store it to the pointer passed as parameter
	/// </summary>
	/// <param name="depth">Pointer to a pointer to store the infra frame </param>
	HRESULT getLongExposureInfraredFrame(ILongExposureInfraredFrame** infra, INT64* time = nullptr)
	{
		HRESULT result;
		ILongExposureInfraredFrameReference* ref = NULL;
		result = frame->get_LongExposureInfraredFrameReference(&ref);
		if (SUCCEEDED(result))
		{
			result = ref->AcquireFrame(infra);
			if (SUCCEEDED(result) && time != nullptr)
			{
				result = ref->get_RelativeTime(time);
			}
			SafeRelease(ref);
		}
#if defined (_LJX_DEBUG)
		else
			cout << "ljxDebug: Failed to get LongExposureInfrafedReference" << endl;
#endif
		return result;
	}

#ifdef _USE_OPENCV
	/// <summary>
	/// Get the depth frame and store it to a CV_16U Mat class
	/// </summary>
	/// <returns>Pointer to a pointer to store the depth frame </returns>
	Mat getDepthMat(INT64* time = nullptr)
	{
		HRESULT result;
		IDepthFrame* getframe = NULL;
		result = getDepthFrame(&getframe, time);
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
	Mat getBodyIndexMat(INT64* time = nullptr)
	{
		HRESULT result;
		IBodyIndexFrame* getframe = NULL;
		result = getBodyIndexFrame(&getframe, time);
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
	Mat getColorMat(INT64* time = nullptr)
	{
		HRESULT result;
		IColorFrame* getframe = NULL;
		result = getColorFrame(&getframe, time);
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
	Mat getInfraredMat(INT64* time = nullptr)
	{
		HRESULT result;
		IInfraredFrame* getframe = NULL;
		result = getInfraredFrame(&getframe, time);
		if (SUCCEEDED(result))
		{
			Mat mat = infra2mat(getframe);
			SafeRelease(getframe);
			return mat;
		}
		return Mat();
	}

	/// <summary>
	/// Get the infrared frame and store it to a CV_16U Mat class
	/// </summary>
	/// <returns>Pointer to a pointer to store the infrared frame </returns>
	Mat getLongExposureInfraredMat(INT64* time = nullptr)
	{
		HRESULT result;
		ILongExposureInfraredFrame* getframe = NULL;
		result = getLongExposureInfraredFrame(&getframe, time);
		if (SUCCEEDED(result))
		{
			Mat mat = longinfra2mat(getframe);
			SafeRelease(getframe);
			return mat;
		}
		return Mat();
	}

#endif // _USE_OPENCV
};

HRESULT IBF2KBody(IBodyFrame* frame, KinectBody bodies[]);


#pragma region KinectFusion

/// <summary>
/// Set Identity in a Matrix4
/// </summary>
/// <param name="mat">The matrix to set to identity</param>
void SetIdentityMatrix(Matrix4 &mat);

class KinectFusion
{

public:
	NUI_FUSION_RECONSTRUCTION_PARAMETERS fusionParameters;
	INuiFusionReconstruction* volume;
	Matrix4* worldToCameraTransform;
	Matrix4 defaultWorldToVolumeTransform;
	UINT16* depthPixelBuffer;
	NUI_FUSION_IMAGE_FRAME* depthFloatImage;
	NUI_FUSION_IMAGE_FRAME* depthSmoothFloatImage;
	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE processorType;
	NUI_FUSION_CAMERA_PARAMETERS cameraParameters;
	WAITABLE_HANDLE coordinateMapChanged;
	DepthSpacePoint* depthDistortMap;
	UINT* depthDistortLT;
	bool cameraParametersValid;
	int sources;

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
		depthSmoothFloatImage(NULL),
		pointCloud(NULL),
		depthDistortLT(NULL),
		depthDistortMap(NULL),
		shadedSurface(NULL),
		sources(sourceCount)
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

		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, NUI_DEPTH_RAW_WIDTH, NUI_DEPTH_RAW_HEIGHT, nullptr, &depthSmoothFloatImage);
		if (FAILED(hr)) return hr;

		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, NUI_DEPTH_RAW_WIDTH, NUI_DEPTH_RAW_HEIGHT, &cameraParameters, &pointCloud);
		if (FAILED(hr)) return hr;

		hr = NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, NUI_DEPTH_RAW_WIDTH, NUI_DEPTH_RAW_HEIGHT, nullptr, &shadedSurface);
		if (FAILED(hr)) return hr;

		depthPixelBuffer = new(std::nothrow) UINT16[NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT];
		depthDistortMap = new(std::nothrow) DepthSpacePoint[NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT];
		depthDistortLT = new(std::nothrow) UINT[NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT];
		for (int i = sources - 1; i >= 0; i--)
		{
			volume->ResetReconstruction(&worldToCameraTransform[i], &defaultWorldToVolumeTransform);
		}
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

		hr = volume->SmoothDepthFloatFrame(depthFloatImage, depthSmoothFloatImage, 3, 0.04f);
		if (FAILED(hr))
		{
			cout << "Failed to smooth depth float image" << endl;
			return hr;
		}

		hr = volume->ProcessFrame(depthSmoothFloatImage, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, nullptr, &worldToCameraTransform[depthSource]);
		//hr = volume->ProcessFrame(depthFloatImage, 1000, 1, nullptr, &worldToCameraTransform[depthSource]);
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

	HRESULT Reset()
	{
		SetIdentityMatrix(*worldToCameraTransform);
		return volume->ResetReconstruction(worldToCameraTransform, nullptr);
	}

	HRESULT CalculateMesh(INuiFusionMesh** mesh)
	{
		return volume->CalculateMesh(1, mesh);
	}

	HRESULT IntegrateFrame(UINT16* depthFrame, int depthSource)
	{
		if (nullptr == depthFrame) return E_POINTER;

		HRESULT hr = S_OK;

		hr = volume->DepthToDepthFloatFrame(depthFrame, NUI_DEPTH_RAW_WIDTH*NUI_DEPTH_RAW_HEIGHT * sizeof(UINT16), depthFloatImage, NUI_FUSION_DEFAULT_MINIMUM_DEPTH, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH, false);
		if (FAILED(hr))
		{
			cout << "Failed to convert depth frame to depthFloatFrame" << endl;
			return hr;
		}

		hr = volume->SmoothDepthFloatFrame(depthFloatImage, depthSmoothFloatImage, 3, 0.04f);
		if (FAILED(hr))
		{
			cout << "Failed to smooth depth float image" << endl;
			return hr;
		}

		float energy = 0.1;
		hr = volume->AlignDepthFloatToReconstruction(depthSmoothFloatImage, 200, nullptr, &energy, &worldToCameraTransform[depthSource]);
		if (FAILED(hr))
		{
			cout << "Failed to align depth to reconstruction" << endl;
			return hr;
		}

		hr = volume->IntegrateFrame(depthSmoothFloatImage, 10, &worldToCameraTransform[depthSource]);
		if (FAILED(hr))
		{
			cout << "Failed to integrate frame" << endl;
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

		//hr = NuiFusionShadePointCloud(pointCloud, &worldToCameraTransform[depthSource], nullptr, shadedSurface, nullptr);

		//if (FAILED(hr))
		//{
		//	cout << "0x" << hex << hr << ": NuiFusionShadePointCloud Failed" << endl;
		//	return hr;
		//}

		return hr;
	}

#ifdef _USE_OPENCV
	Mat GetShadedSurface(int depthSource = 0)
	{
		NuiFusionShadePointCloud(pointCloud, &worldToCameraTransform[depthSource], nullptr, shadedSurface, nullptr);
		return Mat(NUI_DEPTH_RAW_HEIGHT, NUI_DEPTH_RAW_WIDTH, CV_8UC4, shadedSurface->pFrameBuffer->pBits);
	}
#endif
};
#pragma endregion

void PrintMatrix4(Matrix4 mat, int fw = 6, ostream &stream = cout);

#endif //_LJX_EASYKINECT_H
