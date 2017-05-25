#ifndef _MOCAP_FRAME_H
#define _MOCAP_FRAME_H

#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/StdVector>
using namespace std;

class MocapFrame
{
public:
	double mTime;
	std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > mMarkerPos;
	std::vector<int> mMarkerOccluded;
	Eigen::VectorXd mMotorAngle;

	MocapFrame();
	MocapFrame(int nMarkers, int nDofs);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

vector<MocapFrame> ReadMocapFrames(const string& fileName);
void SaveMocapFrames(const string& fileName, const vector<MocapFrame>& frames);
ifstream& operator>> (ifstream& in, MocapFrame& frame);
ofstream& operator<< (ofstream& out, const MocapFrame& frame);

#endif