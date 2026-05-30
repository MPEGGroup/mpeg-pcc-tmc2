/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCC_LEAST_SQUARES_H
#define PCC_LEAST_SQUARES_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace pcc {

struct LeastSquaresResult {
  std::vector<double> x;  // n × nrhs, column-major
};

// Solve the linear least-squares problem using Householder QR factorization,
// equivalent to LAPACK dgels_ with trans='N'.
//
// For m >= n (overdetermined): finds x minimizing ||Ax - b||_2
// For m < n  (underdetermined): finds the minimum-norm solution of Ax = b
//
// `a` is the m×n coefficient matrix (column-major, not modified).
// `b` is max(m,n)×nrhs (column-major); only the first m rows are read as
//     the right-hand side (matching the dgels_ convention where ldb = max(m,n)).
// On success, result.x contains the n×nrhs solution (column-major).
// Returns true on success, false if the matrix is numerically rank-deficient.
inline bool leastSquaresSolve( const std::vector<double> &a,
                               std::size_t                m,
                               std::size_t                n,
                               const std::vector<double> &b,
                               std::size_t                nrhs,
                               LeastSquaresResult        &result ) {
  result.x.assign( n * nrhs, 0.0 );
  if ( m == 0 || n == 0 || nrhs == 0 ) return true;

  const std::size_t nb = std::max( m, n );  // leading dimension of b

  if ( m >= n ) {
    // ---- Overdetermined: min ||Ax - b||_2 via QR(A) ----
    std::vector<double> R = a;          // m × n working copy
    std::vector<double> c( m * nrhs );  // transformed RHS
    for ( std::size_t j = 0; j < nrhs; ++j )
      for ( std::size_t i = 0; i < m; ++i ) c[i + j * m] = b[i + j * nb];

    // Store first element of each Householder vector (v0) for the column.
    std::vector<double> tau( n );

    for ( std::size_t k = 0; k < n; ++k ) {
      // Norm of R[k:m, k]
      double norm = 0.0;
      for ( std::size_t i = k; i < m; ++i ) norm += R[i + k * m] * R[i + k * m];
      norm = std::sqrt( norm );
      if ( norm < 1e-300 ) return false;

      // Choose sign of alpha to avoid cancellation
      const double alpha = ( R[k + k * m] >= 0.0 ) ? -norm : norm;
      const double v0    = R[k + k * m] - alpha;
      R[k + k * m]       = alpha;
      tau[k]             = v0;

      double beta = v0 * v0;
      for ( std::size_t i = k + 1; i < m; ++i ) beta += R[i + k * m] * R[i + k * m];
      if ( std::abs( beta ) < 1e-300 ) return false;
      beta = 2.0 / beta;

      // Apply H_k to R[k:m, k+1:n]
      for ( std::size_t j = k + 1; j < n; ++j ) {
        double dot = v0 * R[k + j * m];
        for ( std::size_t i = k + 1; i < m; ++i ) dot += R[i + k * m] * R[i + j * m];
        dot *= beta;
        R[k + j * m] -= v0 * dot;
        for ( std::size_t i = k + 1; i < m; ++i ) R[i + j * m] -= R[i + k * m] * dot;
      }

      // Apply H_k to c[k:m, :]
      for ( std::size_t j = 0; j < nrhs; ++j ) {
        double dot = v0 * c[k + j * m];
        for ( std::size_t i = k + 1; i < m; ++i ) dot += R[i + k * m] * c[i + j * m];
        dot *= beta;
        c[k + j * m] -= v0 * dot;
        for ( std::size_t i = k + 1; i < m; ++i ) c[i + j * m] -= R[i + k * m] * dot;
      }
    }

    // Back-substitution: R[0:n,0:n] is upper triangular
    for ( std::size_t j = 0; j < nrhs; ++j ) {
      for ( std::size_t i = n; i-- > 0; ) {
        double sum = c[i + j * m];
        for ( std::size_t l = i + 1; l < n; ++l ) sum -= R[i + l * m] * result.x[l + j * n];
        if ( std::abs( R[i + i * m] ) < 1e-300 ) return false;
        result.x[i + j * n] = sum / R[i + i * m];
      }
    }
    return true;
  } else {
    // ---- Underdetermined: min-norm ||x|| s.t. Ax = b via QR(A^T) ----
    // A^T = Q R  (Q n×m, R m×m upper triangular)
    // A   = R^T Q^T
    // Solve R^T y = b (forward sub), then x = Q y (apply Householder in reverse)

    // Build A^T (n × m, column-major)
    std::vector<double> AT( n * m );
    for ( std::size_t j = 0; j < m; ++j )
      for ( std::size_t i = 0; i < n; ++i ) AT[i + j * n] = a[j + i * m];

    std::vector<double> rhs( m * nrhs );
    for ( std::size_t j = 0; j < nrhs; ++j )
      for ( std::size_t i = 0; i < m; ++i ) rhs[i + j * m] = b[i + j * nb];

    // Householder QR of A^T (n × m)
    std::vector<double> QR = AT;
    std::vector<double> tau( m );

    for ( std::size_t k = 0; k < m; ++k ) {
      double norm = 0.0;
      for ( std::size_t i = k; i < n; ++i ) norm += QR[i + k * n] * QR[i + k * n];
      norm = std::sqrt( norm );
      if ( norm < 1e-300 ) return false;

      const double alpha = ( QR[k + k * n] >= 0.0 ) ? -norm : norm;
      const double v0    = QR[k + k * n] - alpha;
      QR[k + k * n]      = alpha;
      tau[k]             = v0;

      double beta = v0 * v0;
      for ( std::size_t i = k + 1; i < n; ++i ) beta += QR[i + k * n] * QR[i + k * n];
      if ( std::abs( beta ) < 1e-300 ) return false;
      beta = 2.0 / beta;

      for ( std::size_t j = k + 1; j < m; ++j ) {
        double dot = v0 * QR[k + j * n];
        for ( std::size_t i = k + 1; i < n; ++i ) dot += QR[i + k * n] * QR[i + j * n];
        dot *= beta;
        QR[k + j * n] -= v0 * dot;
        for ( std::size_t i = k + 1; i < n; ++i ) QR[i + j * n] -= QR[i + k * n] * dot;
      }
    }

    // Solve R^T y = rhs by forward substitution
    // R is stored in QR[0:m,0:m]; R^T[i,l] = QR[l + i*n]
    std::vector<double> y( m * nrhs );
    for ( std::size_t j = 0; j < nrhs; ++j ) {
      for ( std::size_t i = 0; i < m; ++i ) {
        double sum = rhs[i + j * m];
        for ( std::size_t l = 0; l < i; ++l ) sum -= QR[l + i * n] * y[l + j * m];
        if ( std::abs( QR[i + i * n] ) < 1e-300 ) return false;
        y[i + j * m] = sum / QR[i + i * n];
      }
    }

    // x = Q y: pad y to n, then apply Householder H_{m-1} ... H_0
    std::vector<double> xw( n * nrhs, 0.0 );
    for ( std::size_t j = 0; j < nrhs; ++j )
      for ( std::size_t i = 0; i < m; ++i ) xw[i + j * n] = y[i + j * m];

    for ( std::size_t k = m; k-- > 0; ) {
      const double v0k  = tau[k];
      double       beta = v0k * v0k;
      for ( std::size_t i = k + 1; i < n; ++i ) beta += QR[i + k * n] * QR[i + k * n];
      if ( std::abs( beta ) < 1e-300 ) continue;
      beta = 2.0 / beta;

      for ( std::size_t j = 0; j < nrhs; ++j ) {
        double dot = v0k * xw[k + j * n];
        for ( std::size_t i = k + 1; i < n; ++i ) dot += QR[i + k * n] * xw[i + j * n];
        dot *= beta;
        xw[k + j * n] -= v0k * dot;
        for ( std::size_t i = k + 1; i < n; ++i ) xw[i + j * n] -= QR[i + k * n] * dot;
      }
    }

    for ( std::size_t j = 0; j < nrhs; ++j )
      for ( std::size_t i = 0; i < n; ++i ) result.x[i + j * n] = xw[i + j * n];

    return true;
  }
}

}  // namespace pcc

#endif  // PCC_LEAST_SQUARES_H
