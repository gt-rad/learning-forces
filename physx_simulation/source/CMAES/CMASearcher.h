#ifndef _CMA_SEARCH
#define _CMA_SEARCH

#include "Searcher.h"
#include "CMAData.h"

//double Evaluator(double* params, DecoScene* scene);

class CMASearcher : public Searcher
{
public:
	CMASearcher();
	CMASearcher(int dim);
	virtual ~CMASearcher();
	virtual void SetDimension(int dim);
	virtual int Search(CMAData* cData, double* argMin, int maxIterations);
	virtual int Search(CMAData* cData, double* lower_bound, double* upper_bound, double* argMin, int maxIterations);
	
	void CreateMessageQueue();

protected:
	
	double* mPrevSol;
	double* mStandardDeviation;
	int* mInfeasibleTime;
	int NUM_IPC_PROCESS;
	
	bool isFeasible(double* value, double* lower_bound, double* upper_bound);
	void calculateSearchStandardDeviation(double* lower_bound, double* upper_boun);
	void clear();
	void setInitialGuess(double* lower_bound, double* upper_boun);
	void recoverFromScaling(int n, double* lower_bound, double* upper_bound, double* scaledValue, double* trueValue);
	
};




#endif