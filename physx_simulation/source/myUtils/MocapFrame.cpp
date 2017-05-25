#include "MocapFrame.h"
#include "glog/logging.h"

MocapFrame::MocapFrame()
{

}
MocapFrame::MocapFrame(int nMarkers, int nDofs)
{
	mMarkerPos.resize(nMarkers, Eigen::Vector3d::Zero());
	mMarkerOccluded.resize(nMarkers, 1);
	mMotorAngle.resize(nDofs, 0);
}


vector<MocapFrame> ReadMocapFrames(const string& fileName)
{
	ifstream in(fileName.c_str());
	int nFrames, nMarkers;
	const int nDofs = 22;
	in >> nFrames >> nMarkers;
	vector<MocapFrame> ret;
	ret.resize(nFrames);
	
	for (int i = 0; i < nFrames; ++i)
	{
		ret[i].mMarkerPos.resize(nMarkers);
		ret[i].mMarkerOccluded.resize(nMarkers);
		ret[i].mMotorAngle.resize(nDofs);
		
		in >> ret[i];
	}
	return ret;
}
void SaveMocapFrames(const string& fileName, const vector<MocapFrame>& frames)
{
	ofstream oFile(fileName.c_str());
	int nFrames = static_cast<int>(frames.size());
	if (!nFrames) return;
	int nMarkers = static_cast<int>(frames[0].mMarkerPos.size());
	oFile << nFrames << " " << nMarkers << endl;
	for (int i = 0; i < nFrames; ++i)
	{
		oFile << frames[i] << endl;
	}

}

ifstream& operator>> (ifstream& in, MocapFrame& frame)
{
	in >> frame.mTime;
	int nMarkers = static_cast<int>(frame.mMarkerPos.size());
	int nDofs = static_cast<int>(frame.mMotorAngle.size());
	if (!nMarkers || !nDofs)
	{
		LOG(FATAL) << "Memory should be allocated to read in MocapFrame.";
	}
	for (int j = 0; j < nMarkers; ++j)
	{
		in >> frame.mMarkerPos[j][0] >> frame.mMarkerPos[j][1] >> frame.mMarkerPos[j][2] >> frame.mMarkerOccluded[j];

	}
	for (int j = 0; j < nDofs; ++j)
	{
		in >> frame.mMotorAngle[j];

	}

	return in;
}
ofstream& operator<< (ofstream& out, const MocapFrame& frame)
{
	out << frame.mTime << " ";
	int nMarkers = static_cast<int>(frame.mMarkerPos.size());
	for (int j = 0; j < nMarkers; ++j)
	{
		out << frame.mMarkerPos[j][0] << " " << frame.mMarkerPos[j][1] << " " << frame.mMarkerPos[j][2] << " " << frame.mMarkerOccluded[j] << " ";
	}
	int nDofs = static_cast<int>(frame.mMotorAngle.size());
	for (int j = 0; j < nDofs; ++j)
	{
		out << frame.mMotorAngle[j] << " ";
	}
	out << endl;
	
	return out;
}
