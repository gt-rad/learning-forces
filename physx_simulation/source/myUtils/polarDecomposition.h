#ifndef _POLARDECOMPOSITION_H_
#define _POLARDECOMPOSITION_H_

/*

* Copyright (c) 2007, Carnegie Mellon University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Carnegie Mellon University, nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY CARNEGIE MELLON UNIVERSITY ``AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  Polar decomposition of a general 3x3 matrix
  Version 1.0

  Author of this code: Jernej Barbic
  This code has been adapted from the polar decomposition implementation provided as a companion to the book "Graphics Gems IV": 
  Decompose.c 
  Ken Shoemake, 1993 
  Polar Decomposition of 3x3 matrix in 4x4, M = QS.  
  The Graphics Gems IV implementation is freely available at: 
  http://tog.acm.org/GraphicsGems/
  (This is the official website for Graphics Gems software. The website states that "All code here (on the GraphicsGems website) can be used without restrictions".)

*/

#include "Eigen/Dense"

class PolarDecomposition
{
public:

  // Computes the Polar Decomposition of a general 3x3 matrix M
  // M = Q * S
  // M is 3x3 input matrix
  // Q is 3x3 orthogonal output matrix 
  // S is 3x3 symmetric output matrix
  // Note: det(Q)=sgn(det(M)); this sign can be 1 or -1, depending on M
  // M is not modified
  // All matrices are row-major
  static double DoPolarDecomposition(const double * M, double * Q, double * S, double tol = 1.0e-6);
  static void DoPolarDecomposition(const Eigen::Matrix3d& M, Eigen::Matrix3d& Q, Eigen::Matrix3d& S, double tol = 1.0e-6);
protected:

  // va, vb, and v are 3-vectors
  inline static void vcross(const double * va, const double * vb, double * v);

  // 1-norm of 3x3 matrix
  static double norm_one(const double * M);

  // inf-norm of 3x3 matrix
  static double norm_inf(const double * M);
};

// cross product: v = va x vb
inline void PolarDecomposition::vcross(const double * va, const double * vb, double * v)
{
  v[0] = va[1]*vb[2] - va[2]*vb[1];
  v[1] = va[2]*vb[0] - va[0]*vb[2];
  v[2] = va[0]*vb[1] - va[1]*vb[0];
}

#endif

