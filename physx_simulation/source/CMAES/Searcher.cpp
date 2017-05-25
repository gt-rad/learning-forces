#include "Searcher.h"

Searcher::Searcher() : mDim(-1)
{
	mEvaluator = NULL;

}
Searcher::Searcher(int dim) : mDim(-1)
{
	SetDimension(dim);
	mEvaluator = NULL;
}
Searcher::~Searcher()
{
	
}

void Searcher::SetEvaluatorFunc(double(*Evaluate)(CMAData*, int, double*))
{
	mEvaluator = Evaluate;
}

void Searcher::SetDimension(int dim)
{
	mDim = dim;
}

