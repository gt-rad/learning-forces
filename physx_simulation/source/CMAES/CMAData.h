#ifndef _CMA_DATA_H
#define _CMA_DATA_H

#include <iostream>
#include <string>

using namespace std;

class CMAData
{
public:
    CMAData() {}
	virtual ~CMAData();
	virtual void ReadFromFile(const string& filename) = 0;
	virtual int GetNumParameters() const = 0;
	virtual void GetParameterLowerBounds(double* lb) = 0;
	virtual void GetParameterUpperBounds(double* ub) = 0;
	virtual void FromParameterSetting(double* param) = 0;
	virtual void getParameter(double* pm) = 0;
};

#endif