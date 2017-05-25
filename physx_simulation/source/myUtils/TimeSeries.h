#ifndef _TIME_SERIES_H
#define _TIME_SERIES_H

#include <string>
#include <vector>
#include <Eigen/Dense>
using namespace std;

class TimeSeriesSample
{
public:
	double time;
	Eigen::VectorXd value;
};
class TimeSeries
{
public:
	TimeSeries();
	~TimeSeries();
	void ReadFromFile(const string& filename, int valueDim);
	Eigen::VectorXd GetValue(double time);
	TimeSeriesSample GetIthSample(int ithSample);
private:
	vector<TimeSeriesSample> mData;
};

#endif