#include "stdafx.h"
#include <stdio.h>
#include <math.h>
#include "polarDecomposition.h"


// 1-norm of a 3x3 matrix
double PolarDecomposition::norm_one(const double * M)
{
  int i;
  double sum, max;
  max = 0.0;

  for (i=0; i<3; i++) 
  {
    sum = fabs(M[i])+fabs(M[3+i])+fabs(M[6+i]);

    if (max < sum) 
      max = sum;
  }

  return max;
}

// inf-norm of 3x3 matrix
double PolarDecomposition::norm_inf(const double * M)
{
  int i;
  double sum, max;
  max = 0.0;

  for (i=0; i<3; i++) 
  {
    sum = fabs(M[3 * i])+fabs(M[3 * i + 1])+fabs(M[3 * i + 2]);

    if (max < sum) 
      max = sum;
  }

  return max;
}

// M is 3x3 input matrix
// Q is 3x3 rotation output matrix
// S is symmetric 3x3 output matrix
double PolarDecomposition::DoPolarDecomposition(const double * M, double * Q, double * S, double tol)
{
  double Mk[9];
  double MadjTk[9];
  double Ek[9];

  double det, M_one, M_inf, MadjT_one, MadjT_inf, E_one, gamma, g1, g2;

  int i, j;

  // Mk equals transpose of M
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      Mk[3 * i + j] = M[3 * j + i];

  M_one = norm_one(Mk); 
  M_inf = norm_inf(Mk);

  do 
  {
    vcross(&(Mk[3]), &(Mk[6]), &(MadjTk[0])); // cross product of rows 1 and 2
    vcross(&(Mk[6]), &(Mk[0]), &(MadjTk[3]));
    vcross(&(Mk[0]), &(Mk[3]), &(MadjTk[6]));

    det = Mk[0] * MadjTk[0] + Mk[1] * MadjTk[1] + Mk[2] * MadjTk[2];

    if (det==0.0) 
    {
      printf("Warning: zero determinant encountered.\n");
      //do_rank2(Mk, MadjTk, Mk); 

      /*
        // make matrix identity for now
        Q[0] = 1; Q[1] = 0; Q[2] = 0;
        Q[3] = 0; Q[4] = 1; Q[5] = 0;
        Q[6] = 0; Q[7] = 0; Q[8] = 1;
      */

      break;
    }

    MadjT_one = norm_one(MadjTk); 
    MadjT_inf = norm_inf(MadjTk);

    gamma = sqrt(sqrt((MadjT_one*MadjT_inf)/(M_one*M_inf))/fabs(det));

    g1 = gamma*0.5;
    g2 = 0.5/(gamma*det);

    for(i=0; i<9; i++)
    {
      Ek[i] = Mk[i];
      Mk[i] = g1 * Mk[i] + g2 * MadjTk[i];
      Ek[i] -= Mk[i];
    }

    E_one = norm_one(Ek);
 
    M_one = norm_one(Mk);  
    M_inf = norm_inf(Mk);
  } 
  while ( E_one > M_one * tol );

  // Q = Mk^T 
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      Q[3*i+j] = Mk[3*j+i];

  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
    {
      S[3*i+j] = 0;
      for(int k=0; k<3; k++)
        S[3*i+j] += Mk[3*i+k] * M[3*k+j];
    }
    
  // make S symmetric
  for (i=0; i<3; i++) 
    for (j=i; j<3; j++)
      S[3 * i + j] = S[3 * j + i] = 0.5*(S[3 * i + j]+S[3 * j + i]);

  return (det);
}



void PolarDecomposition::DoPolarDecomposition(const Eigen::Matrix3d& M, Eigen::Matrix3d& Q, Eigen::Matrix3d& S, double tol)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::VectorXd singularValues = svd.singularValues();
    Q = U * (V.transpose());
    if (Q.determinant() < 0)
    {
        int minIndex;
        singularValues.minCoeff(&minIndex);
        singularValues(minIndex) *= -1;
        V.col(minIndex) *= -1;
        Q = U * (V.transpose());

    }
    else if (abs(Q.determinant()) < 1e-6)
    {
        
        Q.col(2) = Q.col(0).cross(Q.col(1));
    }
    S = V * (V.transpose());
    int n = singularValues.size();
    for (int i = 0; i < n; ++i)
    {
        S.row(i) *= singularValues[i];
    }
    
}