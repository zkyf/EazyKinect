#include "EasyKinect.h"
#include <opencv2\opencv.hpp>
#define _OPENCV_USED

#ifdef _OPENCV_USED

using namespace cv;

JointType KinectSensor::BoneFrom[KinectSensor::BoneType_Count] =
{
  JointType_Head,              //BoneType_Head_Neck
  JointType_Neck,              //BoneType_Neck_SpineShoulder

  JointType_SpineShoulder,     //BoneType_SpineShoulder_ShoulderRight
  JointType_ShoulderRight,     //BoneType_ShoulderRight_ElbowRight
  JointType_ElbowRight,        //BoneType_ElbowRight_WristRight
  JointType_WristRight,        //BoneType_WristRight_HandRight
  JointType_HandRight,         //BoneType_HandRight_ThumbRight
  JointType_HandRight,         //BoneType_HandRight_HandTipRight

  JointType_SpineShoulder,     //BoneType_SpineShoulder_ShoulderLeft
  JointType_ShoulderLeft,      //BoneType_ShoulderLeft_ElbowLeft
  JointType_ElbowLeft,         //BoneType_ElbowLeft_WristLeft
  JointType_WristLeft,         //BoneType_WristLeft_HandLeft
  JointType_HandLeft,          //BoneType_HandLeft_ThumbLeft
  JointType_HandLeft,          //BoneType_HandLeft_HandTipLeft

  JointType_SpineShoulder,     //BoneType_SpineShoulder_SpineMid
  JointType_SpineMid,          //BoneType_SpineMid_SpineBase

  JointType_SpineBase,         //BoneType_SpineBase_HipRight
  JointType_HipRight,          //BoneType_HipRight_KneeRight
  JointType_KneeRight,         //BoneType_KneeRight_AnkleRight
  JointType_AnkleRight,        //BoneType_AnkleRight_FootRight

  JointType_SpineBase,         //BoneType_SpineBase_HipLeft
  JointType_HipLeft,           //BoneType_HipLeft_KneeLeft
  JointType_KneeLeft,          //BoneType_KneeLeft_AnkleLeft
  JointType_AnkleLeft          //BoneType_AnkleLeft_FootLeft
};

JointType KinectSensor::BoneTo[KinectSensor::BoneType_Count] =
{
  JointType_Neck,              //BoneType_Head_Neck
  JointType_SpineShoulder,     //BoneType_Neck_SpineShoulder

  JointType_ShoulderRight,     //BoneType_SpineShoulder_ShoulderRight
  JointType_ElbowRight,        //BoneType_ShoulderRight_ElbowRight
  JointType_WristRight,        //BoneType_ElbowRight_WristRight
  JointType_HandRight,         //BoneType_WristRight_HandRight
  JointType_ThumbRight,        //BoneType_HandRight_ThumbRight
  JointType_HandTipRight,      //BoneType_HandRight_HandTipRight

  JointType_ShoulderLeft,      //BoneType_SpineShoulder_ShoulderLeft
  JointType_ElbowLeft,         //BoneType_ShoulderLeft_ElbowLeft
  JointType_WristLeft,         //BoneType_ElbowLeft_WristLeft
  JointType_HandLeft,          //BoneType_WristLeft_HandLeft
  JointType_ThumbLeft,         //BoneType_HandLeft_ThumbLeft
  JointType_HandTipLeft,       //BoneType_HandLeft_HandTipLeft

  JointType_SpineMid,          //BoneType_SpineShoulder_SpineMid
  JointType_SpineBase,         //BoneType_SpineMid_SpineBase

  JointType_HipRight,          //BoneType_SpineBase_HipRight
  JointType_KneeRight,         //BoneType_HipRight_KneeRight
  JointType_AnkleRight,        //BoneType_KneeRight_AnkleRight
  JointType_FootRight,         //BoneType_AnkleRight_FootRight

  JointType_HipLeft,           //BoneType_SpineBase_HipLeft
  JointType_KneeLeft,          //BoneType_HipLeft_KneeLeft
  JointType_AnkleLeft,         //BoneType_KneeLeft_AnkleLeft
  JointType_FootLeft           //BoneType_AnkleLeft_FootLeft
};

int KinectSensor::Connected[JointType_Count][JointType_Count] = {-1};

string KinectSensor::JointName[JointType_Count] =
{
  "JointType_SpineBase",
  "JointType_SpineMid",
  "JointType_Neck",
  "JointType_Head",
  "JointType_ShoulderLeft",
  "JointType_ElbowLeft",
  "JointType_WristLeft",
  "JointType_HandLeft",
  "JointType_ShoulderRight",
  "JointType_ElbowRight",
  "JointType_WristRight",
  "JointType_HandRight",
  "JointType_HipLeft",
  "JointType_KneeLeft",
  "JointType_AnkleLeft",
  "JointType_FootLeft",
  "JointType_HipRight",
  "JointType_KneeRight",
  "JointType_AnkleRight",
  "JointType_FootRight",
  "JointType_SpineShoulder",
  "JointType_HandTipLeft",
  "JointType_ThumbLeft",
  "JointType_HandTipRight",
  "JointType_ThumbRight",
};

string KinectSensor::BoneName[KinectSensor::BoneType_Count] =
{
  "BoneType_Head_Neck",
  "BoneType_Neck_SpineShoulder",

  "BoneType_SpineShoulder_ShoulderRight",
  "BoneType_ShoulderRight_ElbowRight",
  "BoneType_ElbowRight_WristRight",
  "BoneType_WristRight_HandRight",
  "BoneType_HandRight_ThumbRight",
  "BoneType_HandRight_HandTipRight",

  "BoneType_SpineShoulder_ShoulderLeft",
  "BoneType_ShoulderLeft_ElbowLeft",
  "BoneType_ElbowLeft_WristLeft",
  "BoneType_WristLeft_HandLeft",
  "BoneType_HandLeft_ThumbLeft",
  "BoneType_HandLeft_HandTipLeft",

  "BoneType_SpineShoulder_SpineMid",
  "BoneType_SpineMid_SpineBase",

  "BoneType_SpineBase_HipRight",
  "BoneType_HipRight_KneeRight",
  "BoneType_KneeRight_AnkleRight",
  "BoneType_AnkleRight_FootRight",

  "BoneType_SpineBase_HipLeft",
  "BoneType_HipLeft_KneeLeft",
  "BoneType_KneeLeft_AnkleLeft",
  "BoneType_AnkleLeft_FootLeft"
};

/// <summary>
/// Convert depthframe in IDepthFrame* to a CV_16U Mat.
/// </summary>
/// <param name="depthframe">The pointer to the obtained depth frame</param>
/// <returns>Returns a Mat in CV_16U containing the depth frame</returns>
Mat depth2mat(IDepthFrame* depthframe)
{
	if (depthframe == NULL)
	{
		return Mat();
	}
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
	if (colorframe == NULL)
	{
		return Mat();
	}
	IFrameDescription* size = NULL;
	colorframe->get_FrameDescription(&size);
	int height = 0, width = 0;
	size->get_Height(&height);
	size->get_Width(&width);
	SafeRelease(size);
	Mat frame(height, width, CV_8UC3, Scalar::all(0));
	static RGBQUAD* colorbuffer = new RGBQUAD[height * width];
	UINT buffersize = height * width * sizeof(RGBQUAD);
	HRESULT hr = colorframe->CopyConvertedFrameDataToArray(buffersize, reinterpret_cast<BYTE*>(colorbuffer), ColorImageFormat_Bgra);
	if (SUCCEEDED(hr))
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
	if (infraframe == NULL)
	{
		return Mat();
	}
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
/// Convert infraredframe in IInfraredFrame* to a CV_16U Mat.
/// </summary>
/// <param name="infraframe">The pointer to the obtained infrared frame</param>
/// <returns>Returns a Mat in CV_16U containing the infrared frame</returns>
Mat longinfra2mat(ILongExposureInfraredFrame* infraframe)
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

#endif

void PrintMatrix4(Matrix4 mat, int fw, ostream &stream)
{
	stream << fixed << setw(fw) << mat.M11 << ", " << mat.M12 << ", " << mat.M13 << ", " << mat.M14 << endl;
	stream << fixed << setw(fw) << mat.M21 << ", " << mat.M22 << ", " << mat.M23 << ", " << mat.M24 << endl;
	stream << fixed << setw(fw) << mat.M31 << ", " << mat.M32 << ", " << mat.M33 << ", " << mat.M34 << endl;
	stream << fixed << setw(fw) << mat.M41 << ", " << mat.M42 << ", " << mat.M43 << ", " << mat.M44 << endl;
}

HRESULT IBF2KBody(IBodyFrame* frame, KinectBody bodies[])
{
	HRESULT result;
	if (frame == NULL)
	{
		return -1;
	}
	INT64 time = 0;
	result = frame->get_RelativeTime(&time);
	if (FAILED(result))
	{
		return result;
	}
	IBody* body[BODY_COUNT] = { 0 };
	result = frame->GetAndRefreshBodyData(BODY_COUNT, body);
	for (int i = 0; i < BODY_COUNT; i++)
	{
		result = body[i]->get_IsTracked(&bodies[i].tracked);
		if (SUCCEEDED(result) && bodies[i].tracked)
		{
			bodies[i].time = time;
			body[i]->get_HandLeftState(&bodies[i].left);
			body[i]->get_HandRightState(&bodies[i].right);
			result = body[i]->GetJoints(_countof(bodies[i].joints), bodies[i].joints);
		}
	}
	return result;
}

void SetIdentityMatrix(Matrix4 &mat)
{
	mat.M11 = 1; mat.M12 = 0; mat.M13 = 0; mat.M14 = 0;
	mat.M21 = 0; mat.M22 = 1; mat.M23 = 0; mat.M24 = 0;
	mat.M31 = 0; mat.M32 = 0; mat.M33 = 1; mat.M34 = 0;
	mat.M41 = 0; mat.M42 = 0; mat.M43 = 0; mat.M44 = 1;
}
