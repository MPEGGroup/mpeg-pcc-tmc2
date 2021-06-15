/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ISO/IEC
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
#include "PCCCommon.h"

#include "PCCPointSet.h"
#include "PCCKdTree.h"

#include "KDTreeVectorOfVectorsAdaptor.h"

using namespace pcc;

typedef KDTreeVectorOfVectorsAdaptor<PCCPointSet3, PCCType, float, 3, metric_L2_Simple_2, size_t> KdTreeAdaptor;

PCCKdTree::PCCKdTree() : kdtree_( nullptr ) {}

PCCKdTree::PCCKdTree( const PCCPointSet3& pointCloud ) : kdtree_( nullptr ) { init( pointCloud ); }

PCCKdTree::~PCCKdTree() { clear(); }
void PCCKdTree::clear() {
  if ( kdtree_ != nullptr ) {
    delete ( static_cast<KdTreeAdaptor*>( kdtree_ ) );
    kdtree_ = nullptr;
  }
}

void PCCKdTree::init( const PCCPointSet3& pointCloud ) {
  clear();
  kdtree_ = new KdTreeAdaptor( 3, pointCloud, 10 );
}

void PCCKdTree::search( const PCCPoint3D& point, const size_t num_results, PCCNNResult& results ) const {
  if ( num_results != results.size() ) { results.resize( num_results ); }
  auto retSize = ( static_cast<KdTreeAdaptor*>( kdtree_ ) )
                     ->index->knnSearch( &point[0], num_results, results.indices(), results.dist() );
  assert( retSize == results.size() );
}

void PCCKdTree::searchRadius( const PCCPoint3D& point,
                              const size_t      num_results,
                              const double      radius,
                              PCCNNResult&      results ) const {
  std::vector<std::pair<size_t, double> > ret;
  nanoflann::SearchParams                 params;
  size_t retSize = ( static_cast<KdTreeAdaptor*>( kdtree_ ) )->index->radiusSearch( &point[0], radius, ret, params );
  if ( retSize > num_results ) { retSize = num_results; }
  ret.resize( retSize );
  results.reserve( retSize );
  for ( const auto& result : ret ) { results.pushBack( result ); }
}
