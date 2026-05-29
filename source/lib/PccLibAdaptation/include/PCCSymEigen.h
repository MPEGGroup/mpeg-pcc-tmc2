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

#ifndef PCC_SYM_EIGEN_H
#define PCC_SYM_EIGEN_H

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace pcc {

struct SymEigenResult {
  std::vector<double> eigenvalues;   // size n, ascending order
  std::vector<double> eigenvectors;  // n*n, column-major; column j is the
                                     // eigenvector for eigenvalues[j]
};

// Cyclic Jacobi eigendecomposition for a real symmetric n x n matrix.
//
// `a` holds the n*n matrix entries; only the lower triangle is read (matching
// the LAPACK UPLO='L' convention) and layout is column-major. On success the
// result contains ascending eigenvalues and the corresponding eigenvectors
// stored as columns of a column-major n*n matrix.
//
// Returns true on success, false if no convergence after `maxSweeps`.
inline bool symEigenDecomposeJacobi( const std::vector<double> &a,
                                     std::size_t                n,
                                     SymEigenResult            &result,
                                     std::size_t                maxSweeps = 100,
                                     double                     tol       = 1e-14 ) {
  result.eigenvalues.assign( n, 0.0 );
  result.eigenvectors.assign( n * n, 0.0 );
  if ( n == 0 ) return true;

  // Symmetrise the lower triangle of `a` into the working matrix `m`
  // (column-major, same layout as `a`).
  std::vector<double> m( n * n, 0.0 );
  for ( std::size_t j = 0; j < n; ++j ) {
    for ( std::size_t i = j; i < n; ++i ) {
      const double x = a[i + j * n];
      m[i + j * n]   = x;
      m[j + i * n]   = x;
    }
  }

  // Eigenvector accumulator, initialised to identity.
  std::vector<double> v( n * n, 0.0 );
  for ( std::size_t i = 0; i < n; ++i ) { v[i + i * n] = 1.0; }

  auto offDiagNorm = [&]() {
    double s = 0.0;
    for ( std::size_t p = 0; p < n; ++p ) {
      for ( std::size_t q = p + 1; q < n; ++q ) {
        const double x = m[p + q * n];
        s += x * x;
      }
    }
    return std::sqrt( s );
  };

  bool converged = false;
  for ( std::size_t sweep = 0; sweep < maxSweeps; ++sweep ) {
    double diagNorm = 0.0;
    for ( std::size_t i = 0; i < n; ++i ) {
      const double d = m[i + i * n];
      diagNorm += d * d;
    }
    diagNorm = std::sqrt( diagNorm );
    if ( offDiagNorm() <= tol * ( diagNorm + tol ) ) {
      converged = true;
      break;
    }

    for ( std::size_t p = 0; p + 1 < n; ++p ) {
      for ( std::size_t q = p + 1; q < n; ++q ) {
        const double apq = m[p + q * n];
        if ( std::abs( apq ) < 1e-300 ) continue;
        const double app = m[p + p * n];
        const double aqq = m[q + q * n];

        const double theta = ( aqq - app ) / ( 2.0 * apq );
        double       t;
        if ( std::abs( theta ) > 1e150 ) {
          t = 0.5 / theta;
        } else {
          const double sign = ( theta >= 0.0 ) ? 1.0 : -1.0;
          t                 = sign / ( std::abs( theta ) + std::sqrt( theta * theta + 1.0 ) );
        }
        const double c = 1.0 / std::sqrt( t * t + 1.0 );
        const double s = t * c;

        m[p + p * n] = app - t * apq;
        m[q + q * n] = aqq + t * apq;
        m[p + q * n] = 0.0;
        m[q + p * n] = 0.0;

        // Rotate the remaining rows and columns of m.
        for ( std::size_t i = 0; i < n; ++i ) {
          if ( i == p || i == q ) continue;
          const double mip = m[i + p * n];
          const double miq = m[i + q * n];
          const double nip = c * mip - s * miq;
          const double niq = s * mip + c * miq;
          m[i + p * n]     = nip;
          m[p + i * n]     = nip;
          m[i + q * n]     = niq;
          m[q + i * n]     = niq;
        }

        // Accumulate the rotation into the eigenvector matrix v.
        for ( std::size_t i = 0; i < n; ++i ) {
          const double vip = v[i + p * n];
          const double viq = v[i + q * n];
          v[i + p * n]     = c * vip - s * viq;
          v[i + q * n]     = s * vip + c * viq;
        }
      }
    }
  }

  if ( !converged ) return false;

  // Extract eigenvalues from m's diagonal and sort ascending to match
  // LAPACK dsyev convention.
  std::vector<double> diag( n );
  for ( std::size_t i = 0; i < n; ++i ) { diag[i] = m[i + i * n]; }

  std::vector<std::size_t> idx( n );
  std::iota( idx.begin(), idx.end(), std::size_t{ 0 } );
  std::sort( idx.begin(), idx.end(), [&]( std::size_t x, std::size_t y ) { return diag[x] < diag[y]; } );

  for ( std::size_t j = 0; j < n; ++j ) {
    const std::size_t src = idx[j];
    result.eigenvalues[j] = diag[src];
    for ( std::size_t i = 0; i < n; ++i ) { result.eigenvectors[i + j * n] = v[i + src * n]; }
  }

  return true;
}

}  // namespace pcc

#endif  // PCC_SYM_EIGEN_H
