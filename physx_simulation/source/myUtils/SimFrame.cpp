#include "SimFrame.h"
#include <iomanip>
#include <fstream>

vector<SimFrame> ReadSimFrames(const string& fileName)
{
	ifstream inFile(fileName.c_str());
	const int nDofs = 22;
	int numFrames;
	inFile >> numFrames;
	vector<SimFrame> ret(numFrames, SimFrame(nDofs));

	for (int i = 0; i < numFrames; ++i)
	{
		inFile >> ret[i];
	}
	return ret;
}
void SaveSimFrames(const string& fileName, const vector<SimFrame>& frames)
{
	ofstream oFile(fileName.c_str());
	int nFrames = static_cast<int>(frames.size());
	if (!nFrames) return;
	oFile << nFrames << endl;
	for (int i = 0; i < nFrames; ++i)
	{
		oFile << frames[i] << endl;
	}
}



istream& operator>> (istream& in, SimFrame& rhs)
{
	in >> rhs.mTime;
	if (!rhs.mPose.size())
	{
		LOG(FATAL) << "Memory is not allocated when reading SimFrame.";
	}
	int nDofs = static_cast<int>(rhs.mPose.size());
	for (int i = 0; i < nDofs; ++i)
	{
		in >> rhs.mPose[i];
	}
	return in;
}
ostream& operator<< (ostream& out, const SimFrame& rhs)
{
	out << rhs.mTime << " ";
	int nDofs = static_cast<int>(rhs.mPose.size());
	for (int i = 0; i < nDofs; ++i)
	{
		out << setprecision(16) << rhs.mPose[i] << " ";
	}
	return out;
}
