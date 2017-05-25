#ifndef _SEARCHER_H
#define _SEARCHER_H

#include "CMAData.h"

class Searcher
{
public:
	Searcher();
	Searcher(int dim);
	virtual ~Searcher();
	virtual void SetDimension(int dim);
	virtual void SetEvaluatorFunc(double(*Evaluate)(CMAData*, int, double*));
	virtual int Search(CMAData* cData, double* argMin, int maxIterations) = 0;
	virtual int Search(CMAData* cData, double* lower_bound, double* upper_bound, double* argMin, int maxIterations) = 0;
protected:
	int mDim;
	double(*mEvaluator)(CMAData*, int, double*);
};

#endif 