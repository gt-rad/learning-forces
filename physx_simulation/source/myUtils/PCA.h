#ifndef _PCA_H
#define _PCA_H

#include "stdafx.h"

class PCA
{
public:
	PCA();
	~PCA();

	void Analyze(const vector<Eigen::VectorXd>& data, vector<double>& eigenValues, vector<Eigen::VectorXd>& basis);

private:

	void centerData();
	void scaleData();
	void performSVD();
	void checkCenterAndScale();

	int mDim;
	int mNumPoints;
	Eigen::MatrixXd mData;
	vector<double> mEigenValues;
	vector<Eigen::VectorXd> mBasis;
};

#endif