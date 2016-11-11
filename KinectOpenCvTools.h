#pragma once

#ifndef _KINECT_OPENCV_TOOLS
#define _KINECT_OPENCV_TOOLS

#include <opencv2\opencv.hpp>
using namespace cv;
#include <iostream>
#include <string>
#include <vector>
using namespace std;

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
			result.at<Vec3b>(i, j)[1] = depthmat.at<unsigned short>(i, j) / 256;
			result.at<Vec3b>(i, j)[2] = depthmat.at<unsigned short>(i, j) % 256;
		}
	}
	return result;
}

/// <summary>
/// Decode the combined mat to CV_16U infrared frame and CV_16U depth frame
/// </summary>
/// <param name="source">Input the combined mat</param>
/// <param name="inframat">Output the Mat structure containing infrared frame</param>
/// <param name="depthmat">Output the Mat structure containing depth frame</param>
void mat2InfraDepth(Mat source, Mat& inframat, Mat& depthmat)
{
	Size size = source.size();
	inframat = Mat(size, CV_16U, Scalar::all(0));
	depthmat = Mat(size, CV_16U, Scalar::all(0));
	for (int i = 0; i < size.height; i++)
	{
		for (int j = 0; j < size.width; j++)
		{
			inframat.at<unsigned short>(i, j) = source.at<Vec3b>(i, j)[0] * 256;
			depthmat.at<unsigned short>(i, j) = source.at<Vec3b>(i, j)[1] * 256 + source.at<Vec3b>(i, j)[2];
		}
	}
}

#endif
