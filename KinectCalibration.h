#ifndef _USE_OPENCV
#define _USE_OPENCV
#endif

#ifndef _LJX_DEBUG
#define _LJX_DEBUG
#endif

#pragma once

#ifndef _KINECT_CALIBRATION_H
#define _KINECT_CALIBRATION_H

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

void Calibration(int board_w, int board_h, int n_boards = 20, KinectSensor *sensor = NULL)
{
	HRESULT hr;
	if (sensor == nullptr)
	{
		sensor = new KinectSensor;
		hr = sensor->init((FrameSourceTypes)(FrameSourceTypes_Depth | FrameSourceTypes_Infrared));
		if (FAILED(hr))
		{
			MessageBoxA(NULL, "Failed to initialize Kinect Sensor depth and infrared frame source.", "Calibration error", 0);
			return;
		}
	}
	int board_n = board_w*board_h;
	Size board_size = Size(board_w, board_h);
	vector<Point2d> corners;
	Mat imgp(n_boards*board_n, 2, CV_32F);
	Mat objp(n_boards*board_n, 2, CV_32F);
	Mat counts(n_boards, 1, CV_32S);

	int cornercount = 0;
	int success = 0;
	int frame = 0;

	while (1)
	{
		hr = sensor->update();
		if (FAILED(hr))
		{
			waitKey(10);
			continue;
		}
		Mat depthframe = sensor->getDepthMat();
		Mat infraframe = sensor->getInfraredMat();

		if (depthframe.empty() || infraframe.empty()) continue;

		imshow("depth", depthframe);

		infraframe.convertTo(infraframe, CV_8U, 1 / 256.0);

		bool found = findChessboardCorners(infraframe, board_size, corners);

		cornerSubPix(infraframe, corners, board_size, Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		drawChessboardCorners(infraframe, board_size, corners, found);

		imshow("infrared corners", infraframe);

		char key = waitKey(10);
		if (key == 27) break;
	}

	sensor->close();
}

#endif