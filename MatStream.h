#pragma once

#ifndef _MATSTREAM_H
#define _MATSTREAM_H

#include <opencv2\opencv.hpp>
using namespace cv;
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

struct MatStreamHeader
{
	int height;
	int width;
	int channels;
	int bytesPerPixel;
	int type;
	int time;
};


class MatStream
{
public:
	enum Op
	{
		in,
		out,
		unknown
	};

private:
	int frameNum;
	MatStreamHeader header;
	fstream file;
	Op mode;

private:
	void ReadHead()
	{
		if (file.fail()) return;
		if (mode == Op::out) return;
		file.seekg(ios::beg, 0);
		file.read((char*)(&frameNum), sizeof(frameNum));
		file.read((char*)(&header), sizeof(header));
	}

	void WriteHead()
	{
		if (file.fail()) return;
		if (mode == Op::in) return;
		file.seekp(ios::beg, 0);
		file.write((char*)(&frameNum), sizeof(frameNum));
		file.write((char*)(&header), sizeof(header));
	}

public:
	MatStream() : mode(Op::unknown), frameNum(0) {}

	void SetHead(MatStreamHeader _header)
	{
		header = _header;
		return;
	}

	MatStreamHeader GetHead()
	{
		return header;
	}

	void Open(string filename, Op op)
	{
		mode = op;
		if (op == Op::in)
		{
			file.open(filename, ios::in | ios::binary);
			ReadHead();
		}
		if (op == Op::out)
		{
			file.open(filename, ios::out | ios::binary);
			WriteHead();
		}
	}

	bool Fail()
	{
		return file.fail();
	}

	bool Write(Mat content)
	{
		if (content.rows != header.height || content.cols != header.width)
			return false;
		if (file.fail()) return false;
		if (mode == Op::in) return false;
		file.write((char*)content.data, header.height*header.width*header.bytesPerPixel*header.channels);
		frameNum++;
		return true;
	}

	Mat Read()
	{
		Mat content(header.height, header.width, header.type, Scalar::all(0));
		if (file.fail()) return Mat();
		if (mode == Op::out) return Mat();
		file.read((char*)(content.data), header.height*header.width*header.bytesPerPixel*header.channels);
		return content;
	}

	void Close()
	{
		if (file.fail()) return;
		if (mode == Op::out)
		{
			file.seekp(ios::beg, 0);
			file.write((char*)(&frameNum), sizeof(frameNum));
		}
		file.close();
	}

	int FrameNum()
	{
		return frameNum;
	}
};

#endif
