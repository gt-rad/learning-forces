#ifndef MATHLIB_H
#define MATHLIB_H

#include <algorithm>
#include <vector>
#include "Eigen/Dense"
#include "RenderType.h"
using namespace std;

#define LinearInterpolate(value1,value2,alpha) ( ( 1.0 - alpha ) * value1 + alpha * value2 )


const static DOUBLE EPSILON_FLOAT = 1e-6;
const static DOUBLE MAX_DOUBLE = 1e300;
const static DOUBLE MIN_DOUBLE = -1e300;

const static INT MAX_FACES_BOX = 12;
const static INT NUM_VERTICES_PER_FACE = 3;
const static INT NUM_VERTICES_IN_BOX = 8;
const static INT MAX_FACES_PLANE = 2;
const static INT NUM_VERTICES_IN_PLANE = 4;

//  [4/9/2008 HATEVOL] ADDED start
//const static __int32 INV_SIGN_FLOAT = 0x7fffffff;
//const static __int64 INV_SIGN_DOUBLE = 0x7fffffffffffffff;
//  [4/9/2008 HATEVOL] ADDED end

class ConvexVolume;

//  [4/9/2008 HATEVOL] ADDED start
inline FLOAT signedAbs(const FLOAT& _X)
{
//#ifdef USE_ASM
//	T fOut = 0;
//	__asm
//	{
//		MOV EAX, _X;
//		SHL EAX, 1; //set the sign bit to 0 by shift left then right
//		SHR EAX, 1;
//		MOV fOut, EAX;
//	}
//	return fOut;
//#else
//	unsigned int* temp = (unsigned int*)&_X;
//	unsigned int out = *temp;
//
//	out = out << 1;
//	out = out >> 1;
//
//	return *((FLOAT*)&out);
//#endif
	return abs(_X);
/* #ifdef USE_ASM */
/* 	FLOAT fOut; */
/* 	__asm */
/* 	{ */
/* 		MOV EAX, _X; */
/* 		AND EAX, INV_SIGN_FLOAT; //set the sign bit to 0 by AND */
/* 		MOV fOut, EAX; */
/* 	} */
/* 	return fOut; */
/* #else */
/* 	__int32* temp = (__int32*)&_X; */
/* 	__int32 out = *temp & INV_SIGN_FLOAT; */
/* 	return *((FLOAT*)&out); */
/* #endif */

}

inline DOUBLE signedAbs(const DOUBLE& _X)
{
//#ifdef USE_ASM
//	T fOut = 0;
//	__asm
//	{
//		MOV EAX, _X;
//		SHL EAX, 1; //set the sign bit to 0 by shift left then right
//		SHR EAX, 1;
//		MOV fOut, EAX;
//	}
//	return fOut;
//#else
//	unsigned int* temp = (unsigned int*)&_X;
//	unsigned int out = *temp;
//
//	out = out << 1;
//	out = out >> 1;
//	DOUBLE d = *((DOUBLE*)&out);
//
//	return *((DOUBLE*)&out);
//#endif
	return abs(_X);
/* #ifdef USE_ASM */
/* 	DOUBLE dOut; */
/* 	__asm */
/* 	{ */
/* 		MOV EAX, _X; */
/* 		AND EAX, INV_SIGN_DOUBLE; //set the sign bit to 0 by AND */
/* 		MOV fOut, EAX; */
/* 	} */
/* 	return dOut; */
/* #else */
/* 	__int64* temp = (__int64*)&_X; */
/* 	__int64 out = *temp & INV_SIGN_DOUBLE; */
/* DOUBLE d = *((DOUBLE*)&out); */
/* 	return *((DOUBLE*)&out); */
/* #endif */

}

inline INT signedAbs(INT iNum )
{
	return abs(iNum);
#ifdef USE_ASM
	INT iOut = 0;
	__asm
	{
		MOV EAX, iNum;
		MOV EDX, EAX;
		SAR EDX, 31;   //all of edx's bit are eax's sign bit: 000.. or 111
		XOR EAX, EDX; //this interesting algorithm help to avoid "if else" structure
		SUB EAX, EDX;
		MOV iOut, EAX;
	}
	return iOut;
#else

	INT out = iNum;
	INT temp = iNum;
	temp = temp >> 31;

	out = out ^ temp;
	out = out - temp;

	return out;

#endif
}
//  [4/9/2008 HATEVOL] ADDED end

template <typename T>
T Clamp(const T& n, const T& lower, const T& upper) {
	return std::max(lower, std::min(n, upper));
}
//template <typename T>
//T LinearInterpolate(const T& value1, const T& value2, DOUBLE alpha)
//{
//	assert(alpha >= 0.0 && alpha <= 1.0);
//	//  [4/7/2008 HATEVOL] ANNOTATION start
//	//if (abs(alpha) < EPSILON_FLOAT)
//	//	return value1;
//	//else if (abs(alpha - 1) < EPSILON_FLOAT)
//	//	return value2;
//	//else
//	//	return (1.0 - alpha) * value1 + alpha * value2;
//	//  [4/7/2008 HATEVOL] ANNOTATION end
//
//	//  [4/7/2008 HATEVOL] ADDED start
//	return ( 1.0 - alpha ) * value1 + alpha * value2;
//	//  [4/7/2008 HATEVOL] ADDED end
//}

template <typename T>
T BilinearInterpolate(const T& xValue1, const T& xValue2, const T& yValue1, const T& yValue2, DOUBLE xAlpha, DOUBLE yAlpha)
{
	//assert(xAlpha >= 0.0 && xAlpha <= 1.0 && yAlpha >= 0.0 && yAlpha <= 1.0);
	T xBlend = LinearInterpolate(xValue1, xValue2, xAlpha); //xAlpha * xValue1 + (1.f - xAlpha) * xValue2;
	T yBlend = LinearInterpolate(yValue1, yValue2, xAlpha);//xAlpha * yValue1 + (1.f - xAlpha) * yValue2;
	return LinearInterpolate(xBlend, yBlend, yAlpha);//yAlpha * xBlend + (1.f - yAlpha) * yBlend;
}

template <typename T>
T TrilinearInterpolate(const T& xValue1, const T& xValue2, const T& xValue3, const T& xValue4, 
					   const T& yValue1, const T& yValue2, const T& yValue3, const T& yValue4, 
					   DOUBLE xAlpha, DOUBLE yAlpha, DOUBLE zAlpha)
{
	//assert(xAlpha >= 0.0 && xAlpha <= 1.0 && yAlpha >= 0.0 && yAlpha <= 1.0 && zAlpha >= 0.0 && zAlpha <= 1.0);
	T xBlend = BilinearInterpolate(xValue1, xValue2, xValue3, xValue4, xAlpha, yAlpha);
	T yBlend = BilinearInterpolate(yValue1, yValue2, yValue3, yValue4, xAlpha, yAlpha);
	return LinearInterpolate(xBlend, yBlend, zAlpha);
}



unsigned short CeilPower2(unsigned short x);
void indexTo3d (INT index, UINT& xIndex, UINT& yIndex, UINT& zIndex, UINT numX, UINT numY, UINT numZ);
UINT indexTo1d (INT xIndex, INT yIndex, INT zIndex, UINT numX, UINT numY, UINT numZ);
void indexTo2d(int index, int& xIndex, int& yIndex, int numX, int numY);
int indexTo1d(int xIndex, int yIndex, int numX, int numY);
UINT round2Pow2(UINT num);
double RandDouble(double low, double high);
float TriangleArea(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
Eigen::Vector3f TriangleNormal(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c);
void RandomPointInTriangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, const Eigen::Vector3f& na, const Eigen::Vector3f& nb, const Eigen::Vector3f& nc, Eigen::Vector3f* p, Eigen::Vector3f* n);
Eigen::Matrix3d FormulateSkewSymmetricMatrix(const Eigen::Vector3d& vec);
bool NearlyEqual(double a, double b, double threshold = EPSILON_FLOAT);
double ComputeWeightFromLWLinearRegression(double dist, double maxDist);
vector<int> Index1DToND(int id, const vector<int>& lenPerDim);
int IndexNDTo1D(const vector<int>& indices, const vector<int>& lenPerDim);
double sigmod(double x);
void PCAOnPoints(const vector<Eigen::Vector3d>& points, Eigen::Vector3d& mean, vector<Eigen::Vector3d>& axis);
vector<vector<int> > GetPermutation(int startVal, int num);
#endif
