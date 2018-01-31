#pragma once

#ifndef MYKINECTREC_H
#define MYKINECTREC_H

#ifndef _USE_OPENCV
#define _USE_OPENCV
#endif

#include "EasyKinect.h"
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace cv;
using namespace std;

struct MyKinectFrame
{
	Mat depth;
	INT64 depthTime;
	Mat infrared;
	INT64 infraTime;
	KinectBody bodies[BODY_COUNT];
	Point2f jind[BODY_COUNT][JointType_Count];

	MyKinectFrame();
};

class MyKinectRec
{
public:
	enum Mode
	{
		in,
		out
	};

	MyKinectRec();
	MyKinectRec(string fileName, Mode mode);
	bool Open(string fileName, Mode mode);
	MyKinectFrame Read();
	void Write(MyKinectFrame frame);
	void Close();
	void SeekFrame(int index);
	int Length();
	int Size();
	bool Failed();
	bool Eof();
	string FileName();

private:
	fstream file;
	string fileName;
	bool failed;
	Mode iomode;
};

#endif
