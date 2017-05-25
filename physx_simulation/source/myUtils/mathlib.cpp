#include "stdafx.h"
#include "mathlib.h"
#include "polarDecomposition.h"

const static DOUBLE PI = 3.14159265359;

void indexTo2d(int index, int& xIndex, int& yIndex, int numX, int numY)
{
	assert(index >= 0 && index < numX * numY);
	xIndex = index / (numY);
	yIndex = index % numY;
}

int indexTo1d(int xIndex, int yIndex, int numX, int numY)
{
	assert(xIndex >= 0 && xIndex < numX);
	assert(yIndex >= 0 && yIndex < numY);
	return xIndex * numY + yIndex;
}

void indexTo3d (INT index, UINT& xIndex, UINT& yIndex, UINT& zIndex, UINT numX, UINT numY, UINT numZ)
{
	assert (index >= 0 && (UINT)index < numX * numY * numZ);
	xIndex = index / (numY * numZ);
	UINT remainder = index % (numY * numZ);
	yIndex = remainder / numZ;
	zIndex = remainder % numZ;
}

UINT indexTo1d (INT xIndex, INT yIndex, INT zIndex, UINT numX, UINT numY, UINT numZ)
{
	assert(xIndex >= 0 && (UINT)xIndex < numX);
	assert(yIndex >= 0 && (UINT)yIndex < numY);
	assert(zIndex >= 0 && (UINT)zIndex < numZ);
	return xIndex * (numY * numZ) + yIndex * numZ + zIndex;
}

UINT round2Pow2(UINT num)
{
	if (num & (num - 1))
	{
		INT ithBit = 0;
		for (ithBit = 1; ithBit < 8 * sizeof(UINT); ithBit++)
		{
			if ((num >> ithBit) == 1)
				break;
		}
		return 1 << (ithBit + 1);
	}
	else
	{
		return num;
	}
}
double RandDouble(double low, double high)
{
	double temp;

	/* swap low & high around if the user makes no sense */
	if (low > high)
	{
		temp = low;
		low = high;
		high = temp;
	}

	/* calculate the random number & return it */
	temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0))
		* (high - low) + low;
	return temp;
}

float TriangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
{
	Eigen::Vector3f ab = b - a;
	Eigen::Vector3f ac = c - a;
	Eigen::Vector3f cross = ab.cross(ac);
	return 0.5f * cross.norm();
}
Eigen::Vector3f TriangleNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
{
	Eigen::Vector3f ab = b - a;
	Eigen::Vector3f ac = c - a;
	Eigen::Vector3f cross = ab.cross(ac);
	return cross.normalized();
}
void RandomPointInTriangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& na, const Eigen::Vector3f& nb, const Eigen::Vector3f& nc, Eigen::Vector3f* p, Eigen::Vector3f* n)
{
	float r1 = static_cast<float>(RandDouble(0, 1));
	float r2 = static_cast<float>(RandDouble(0, 1));
	float w1 = (1 - sqrt(r1));
	float w2 = (sqrt(r1) * (1 - r2));
	float w3 = (sqrt(r1) * r2);
	*p = w1 * a + w2 * b + w3 * c;
	*n = w1 * na + w2 * nb + w3 * nc;
	n->normalize();
}

Eigen::Matrix3d FormulateSkewSymmetricMatrix(const Eigen::Vector3d& vec)
{
	Eigen::Matrix3d ret;
	ret(0, 0) = 0;
	ret(0, 1) = -vec[2];
	ret(0, 2) = vec[1];
	  
	ret(1, 0) = vec[2];
	ret(1, 1) = 0;
	ret(1, 2) = -vec[0];
	  
	ret(2, 0) = -vec[1];
	ret(2, 1) = vec[0];
	ret(2, 2) = 0;

	return ret;
}

bool NearlyEqual(double a, double b, double threshold)
{
    return (abs(a - b) < threshold);
}

double ComputeWeightFromLWLinearRegression(double dist, double maxDist)
{
	const double tau = 1.0 / 3.0;
	double normalizedDist = dist / maxDist;
	double weight = exp(-normalizedDist * normalizedDist / (2 * tau * tau));
	return weight;
}

vector<int> Index1DToND(int id, const vector<int>& lenPerDim)
{
	int numDim = static_cast<int>(lenPerDim.size());
	vector<int> ret;
	if (numDim == 1)
	{
		ret.push_back(id);
		return ret;
	}
	int divider = 1;
	for (int i = 1; i < numDim; ++i)
	{
		divider *= lenPerDim[i];
	}
	for (int i = 1; i < numDim; ++i)
	{
		int idPerDim = id / divider;
		id %= divider;

		ret.push_back(idPerDim);
		divider /= lenPerDim[i];
	}

	ret.push_back(id);
	return ret;
}
int IndexNDTo1D(const vector<int>& indices, const vector<int>& lenPerDim)
{
	int numDim = static_cast<int>(lenPerDim.size());
	int ret = indices[0];
	for (int i = 1; i < numDim; ++i)
	{
		ret = ret * lenPerDim[i] + indices[i];
	}
	return ret;
}

double sigmod(double x)
{
	return 1.0 / (1.0 + exp(-x));
}

void PCAOnPoints(const vector<Eigen::Vector3d>& points, Eigen::Vector3d& mean, vector<Eigen::Vector3d>& axis)
{
	axis.resize(3);
	int numPoints = static_cast<int>(points.size());
	Eigen::MatrixXd dataMat = Eigen::MatrixXd(numPoints, 3);
	for (int i = 0; i < numPoints; ++i)
	{
		dataMat.row(i) = points[i];
	}
	mean = dataMat.colwise().mean();
	Eigen::MatrixXd centered = dataMat.rowwise() - mean.transpose();
	Eigen::MatrixXd cov = centered.transpose() * centered;

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es;

	es.compute(cov);
	axis[0] = es.eigenvectors().col(0);
	axis[1] = es.eigenvectors().col(1);
	axis[2] = es.eigenvectors().col(2);
}


vector<vector<int> > GetPermutation(int startVal, int num)
{
	std::vector<int> input;
	std::vector<std::vector<int> > ret;
	for (int i = 0; i < num; ++i)
	{
		input.push_back(startVal + i);
	}

	do {
		ret.push_back(input);
	} while (std::next_permutation(input.begin(), input.end()));
	return ret;
}