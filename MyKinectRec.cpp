#include "MyKinectRec.h"

MyKinectFrame::MyKinectFrame() :
	depth(NUI_DEPTH_RAW_HEIGHT, NUI_DEPTH_RAW_WIDTH, CV_16U, Scalar::all(0)),
	infrared(NUI_DEPTH_RAW_HEIGHT, NUI_DEPTH_RAW_WIDTH, CV_16U, Scalar::all(0))
{
	memset(jind, 0, sizeof(Point2f)*BODY_COUNT*JointType_Count);
}

MyKinectRec::MyKinectRec()
{
	failed = true;
}

MyKinectRec::MyKinectRec(string fileName, Mode mode)
{
	failed = Open(fileName, mode);
}

bool MyKinectRec::Open(string fileName, Mode mode)
{
	if (mode == Mode::in)
	{
		file.open(fileName, ios::in | ios::binary);
	}
	if (mode == Mode::out)
	{
		file.open(fileName, ios::out | ios::binary);
	}
	iomode = mode;
	failed = file.fail();
	if (!failed)
	{
		this->fileName = fileName;
	}
	else
	{
		this->fileName = "";
	}
	return !failed;
}

MyKinectFrame MyKinectRec::Read()
{
	MyKinectFrame frame;
	if (failed || iomode != Mode::in)
	{
		return frame;
	}
	file.read((char*)(frame.depth.data), frame.depth.cols*frame.depth.rows * 2);
	file.read((char*)(&frame.depthTime), sizeof(INT64));
	file.read((char*)(frame.infrared.data), frame.infrared.cols*frame.depth.rows * 2);
	file.read((char*)(&frame.infraTime), sizeof(INT64));
	file.read((char*)(frame.bodies), BODY_COUNT * sizeof(KinectBody));
	file.read((char*)(frame.jind), BODY_COUNT * JointType_Count * sizeof(Point2f));
	if (file.eof()) return frame;
	return frame;
}

void MyKinectRec::Write(MyKinectFrame frame)
{
	if (failed || iomode != Mode::out)
	{
		return;
	}
	file.write((char*)(frame.depth.data), frame.depth.cols*frame.depth.rows * 2);
	file.write((char*)(&frame.depthTime), sizeof(INT64));
	file.write((char*)(frame.infrared.data), frame.infrared.cols*frame.depth.rows * 2);
	file.write((char*)(&frame.infraTime), sizeof(INT64));
	file.write((char*)(frame.bodies), BODY_COUNT * sizeof(KinectBody));
	file.write((char*)(frame.jind), BODY_COUNT * JointType_Count * sizeof(Point2f));
}

void MyKinectRec::Close()
{
	file.close();
	failed = true;
}

void MyKinectRec::SeekFrame(int index)
{
	if (iomode == Mode::in)
	{
		file.seekg(index * sizeof(MyKinectFrame), ios::beg);
	}
	else
	{
		file.seekp(index * sizeof(MyKinectFrame), ios::beg);
	}
}

int MyKinectRec::Length()
{
	return Size() / sizeof(MyKinectFrame);
}

int MyKinectRec::Size()
{
	if (iomode == Mode::in)
	{
		int pos = file.tellg();
		file.seekg(0, ios::end);
		int size = file.tellg();
		file.seekg(pos, ios::beg);
		return size;
	}
	else
	{
		int pos = file.tellp();
		file.seekp(0, ios::end);
		int size = file.tellp();
		file.seekp(pos, ios::beg);
		return size;
	}
}

bool MyKinectRec::Failed() { return failed; }

bool MyKinectRec::Eof() { return file.eof(); }

string MyKinectRec::FileName() { return fileName; }
