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

#ifndef PCC_ADAPTATION_H
#define PCC_ADAPTATION_H

#include "PCCCommon.h"
#include "PCCPointSet.h"
#include "PCCPointSetGS.h"

#include <array>
#include <string>
#include <vector>

namespace pcc {
/**
 * @struct PCCAdaptationParameters
 * @brief Parameters for GS point cloud adaptation (quantization/dequantization).
 *
 * This structure stores:
 * - Input configuration (bit depths, limits)
 * - Computed parameters during adaptation (actual min/max values used)
 * - Flags for enabling/disabling specific operations
 *
 * Based on gs_pre_process.py and gs_quantize.py logic.
 */
struct PCCAdaptationParameters {
  // ==================== Position Parameters ====================
  /// Bit depth for position quantization (default: 18)
  uint32_t positionBitDepth_ = 18;

  /// Position limits [min, max] for each axis - input configuration
  std::array<double, 3> positionLimitMin_ = { 0.0, 0.0, 0.0 };
  std::array<double, 3> positionLimitMax_ = { 256.0, 256.0, 256.0 };

  /// Auto-compute position limits from point cloud bounding box
  bool autoComputePositionLimits_ = true;

  /// Computed position limits (stored during adaptation)
  std::array<double, 3> computedPositionMin_ = { 0.0, 0.0, 0.0 };
  std::array<double, 3> computedPositionMax_ = { 0.0, 0.0, 0.0 };

  // ==================== Spherical Harmonics Parameters ====================
  /// Bit depth for SH quantization (default: 10)
  uint32_t shBitDepth_ = 10;

  /// Number of SH coefficient sets (1-16, typically 16 for degree 3)
  size_t shCoefficientCount_ = 16;

  /// SH limits per coefficient [16][3] - input configuration
  /// Default: [-4, 4] for all coefficients
  std::array<std::array<double, 3>, 16> shLimitMin_;
  std::array<std::array<double, 3>, 16> shLimitMax_;

  /// Enable RGB to YUV conversion for SH (better compression)
  bool shConvertToYUV_ = true;

  /// Computed SH limits (stored during adaptation)
  std::array<std::array<double, 3>, 16> computedShMin_;
  std::array<std::array<double, 3>, 16> computedShMax_;

  // current CTC sets internal data arrangement to planar
  size_t shMappingMode_ = 1;

  // ==================== SH 2D Transform Parameters ====================
  /// Enable 2D transform for SH (m75038 or m75790)
  bool gsdTransform2dPresentFlag_ = false;

  /// Enable data adaptive SH transform (m75790)
  bool dataAdaptiveShTransformEnabled_ = false;

  // Enable adaptive global/group PCA selection (m76164)
  bool adaptivePcaEnabled_ = false;

  /// Column Transform type
  std::vector<int> shTrTypeCol_;

  /// Row transform groups
  std::vector<std::vector<int>> shTrGrpRow_;

  /// Row Transform type
  std::vector<int> shTrTypeRow_;

  /// KLT flag for each group
  std::vector<int> shKltFlag_;

  /// KLT parameters for each group
  struct PCCKltParameters {
    std::vector<std::vector<double>> eigvec;
    std::vector<double>              mean;
    std::vector<double>              sigma;
  };
  std::vector<PCCKltParameters> shKltParams_;

  // ==================== SH Non-Uniform Quantization ====================
  /// Enable non-uniform quantization for SH (m75025)
  bool enableShNonUniformQuantization_ = false;

  /// Number of regions minus 1 for each SH coefficient and channel [16*3]
  std::vector<int> shNumRegionsMinus1_;

  /// Transition points for multi-region quantization (quantized integer values)
  std::vector<int> shTransitionPoints_;

  /// Step sizes for each region
  std::vector<double> shStepSizes_;

  /// Number of bits used for transition points
  std::vector<int> shNumBitsForTransitionPoint_;

  // ==================== SH Adaptive Quantization =======================
  /// Enable adaptive quantization for SH (m75039)
  bool enableAdaptiveShQuantization_ = false;

  /// Whether to avoid pruning Gaussians for which the adaptive SH bits are set to 0
  bool keepZeroShBitGaussians_ = false;

  /// List of bitshifts for each adaptive quantization group index
  std::vector<int> adaptiveShBitshifts_;

  /// Path to file containing per-gaussian importance scores
  std::string importanceScoresPath_;

  // ==================== Opacity Parameters ====================
  /// Bit depth for opacity quantization (default: 10)
  uint32_t opacityBitDepth_ = 10;

  /// Opacity limits [min, max] - input configuration
  /// Note: opacity in 3DGS is often in logit space, typical range [-7, 18]
  double opacityLimitMin_ = -7.0;
  double opacityLimitMax_ = 18.0;

  /// Auto-compute opacity limits from data
  bool autoComputeOpacityLimits_ = false;

  /// If true, convert logit opacity to linear before quantization
  bool encodeLinearOpacity_ = false;

  /// Computed opacity limits (stored during adaptation)
  double computedOpacityMin_ = 0.0;
  double computedOpacityMax_ = 0.0;

  // ==================== Scale Parameters ====================
  /// Bit depth for scale quantization (default: 10)
  uint32_t scaleBitDepth_ = 10;

  /// Scale limits [min, max] for each axis - input configuration
  /// Note: scale in 3DGS is often in log space, typical range [-26, 4]
  std::array<double, 3> scaleLimitMin_ = { -26.0, -26.0, -26.0 };
  std::array<double, 3> scaleLimitMax_ = { 4.0, 4.0, 4.0 };

  /// Auto-compute scale limits from data
  bool autoComputeScaleLimits_ = false;

  /// Computed scale limits (stored during adaptation)
  std::array<double, 3> computedScaleMin_ = { 0.0, 0.0, 0.0 };
  std::array<double, 3> computedScaleMax_ = { 0.0, 0.0, 0.0 };

  // ==================== Scale Power Transform ====================
  /// Enable power transform + 2-segment PLS for scale (m76151)
  bool enableScalePowerTransform_ = false;

  /// Computed power values per axis (set by encoder via R-D search, read from bitstream by decoder)
  std::array<double, 3> computedScalePower_ = { 1.0, 1.0, 1.0 };

  /// Scaling factor applied after power transform (gsst_scale_scaling, typically 1.0)
  std::array<double, 3> computedScaleScalingFactor_ = { 1.0, 1.0, 1.0 };

  // ==================== Scale Non-Uniform Quantization ====================
  /// Enable non-uniform quantization for Scale
  bool enableScaleNonUniformQuantization_ = false;

  /// Number of regions minus 1 for each Scale channel
  std::vector<int> scaleNumRegionsMinus1_;

  /// Transition points for multi-region quantization (quantized integer values)
  std::vector<int> scaleTransitionPoints_;

  /// Step sizes for each region
  std::vector<double> scaleStepSizes_;

  /// Number of bits used for transition points
  std::vector<int> scaleNumBitsForTransitionPoint_;

  // ==================== Rotation Parameters ====================
  /// Bit depth for rotation quantization (default: 10)
  uint32_t rotationBitDepth_ = 10;

  /// Rotation limits (quaternion components w,x,y,z) - input configuration
  /// Quaternions are normalized, so range is [-1, 1]
  std::array<double, 4> rotationLimitMin_ = { -1.0, -1.0, -1.0, -1.0 };
  std::array<double, 4> rotationLimitMax_ = { 1.0, 1.0, 1.0, 1.0 };

  /// Enable quaternion canonicalization (ensure w >= 0)
  bool enableRotationCanonicalization_ = true;

  /// Computed rotation limits (stored during adaptation)
  std::array<double, 4> computedRotationMin_ = { 0.0, 0.0, 0.0, 0.0 };
  std::array<double, 4> computedRotationMax_ = { 0.0, 0.0, 0.0, 0.0 };

  // ==================== Enable Flags ====================
  bool enablePositionQuantization_ = true;
  bool enableShQuantization_       = true;
  bool enableOpacityQuantization_  = true;
  bool enableScaleQuantization_    = true;
  bool enableRotationQuantization_ = true;

  // ==================== Constructor ====================
  PCCAdaptationParameters() {
    // Initialize SH limits to default [-4, 4]
    for ( size_t i = 0; i < 16; ++i ) {
      for ( size_t c = 0; c < 3; ++c ) {
        shLimitMin_[i][c]    = -4.0;
        shLimitMax_[i][c]    = 4.0;
        computedShMin_[i][c] = 0.0;
        computedShMax_[i][c] = 0.0;
      }
    }
  }

  // ==================== Serialization ====================
  /// Write parameters to JSON config file (for decoder)
  bool writeConfig( const std::string &filename ) const;

  /// Read parameters from JSON config file
  bool readConfig( const std::string &filename );
};

/**
 * @class PCCAdaptation
 * @brief Adapts Gaussian Splatting point clouds from float to quantized integer representation.
 *
 * This class implements the preprocessing pipeline for 3D Gaussian Splatting point clouds:
 * 1. Takes PCCPointSetGS (float) as input
 * 2. Applies quantization to positions, SH, opacity, scale, rotation
 * 3. Returns PCCPointSet3 (int) for encoding
 *
 * Based on gs_pre_process.py and gs_quantize.py algorithms.
 */
class PCCAdaptation {
 public:
  PCCAdaptation();
  ~PCCAdaptation();

  /**
   * @brief Main adaptation function - converts float point cloud to quantized int point cloud.
   *
   * @param inputPointCloud  Input point cloud with float values (PCCPointSetGS)
   * @param outputPointCloud Output point cloud with quantized int values (PCCPointSet3)
   * @param params           Adaptation parameters (input config + computed values stored)
   * @return true on success, false on failure
   */
  bool adapt( const PCCPointSetGS &inputPointCloud, PCCPointSet3 &outputPointCloud, PCCAdaptationParameters &params );

  /**
   * @brief Inverse adaptation - converts quantized int point cloud back to float.
   *
   * @param inputPointCloud  Input point cloud with quantized int values (PCCPointSet3)
   * @param outputPointCloud Output point cloud with reconstructed float values (PCCPointSetGS)
   * @param params           Adaptation parameters (must contain values from adapt())
   * @return true on success, false on failure
   */
  bool inverseAdapt( const PCCPointSet3            &inputPointCloud,
                     PCCPointSetGS                 &outputPointCloud,
                     const PCCAdaptationParameters &params );

  /// Prune gaussians with 0 SH bits and assign quantization groups for adaptive SH quantization.
  bool pruneAndGroupAdaptiveQuantization( const PCCPointSetGS     &inputPointCloud,
                                          PCCPointSetGS           &outputPointCloud,
                                          PCCAdaptationParameters &params,
                                          uint32_t                 frameNumber ) const;

  // ==================== Individual Operations ====================

  /// Quantize positions from float to int
  bool quantizePositions( const PCCPointSetGS &input, PCCPointSet3 &output, PCCAdaptationParameters &params );

  /// Quantize spherical harmonics (with optional RGB->YUV conversion)
  bool quantizeSphericalHarmonics( const PCCPointSetGS &input, PCCPointSet3 &output, PCCAdaptationParameters &params );

  /// Quantize opacity values
  bool quantizeOpacities( const PCCPointSetGS &input, PCCPointSet3 &output, PCCAdaptationParameters &params );

  /// Quantize scale values
  bool quantizeScales( const PCCPointSetGS &input, PCCPointSet3 &output, PCCAdaptationParameters &params );

  /// Quantize rotation quaternions (with canonicalization)
  bool quantizeRotations( const PCCPointSetGS &input, PCCPointSet3 &output, PCCAdaptationParameters &params );

  /// Apply canonical orientation (m75790) - optimizes axes for compression
  bool applyCanonicalOrientation( PCCPointSetGS &pointCloud );

  /// Initialize SH 2D transform parameters
  void initializeShTransformParams( PCCAdaptationParameters &params );
  /// Transform SH coefficients from RGB to YUV
  bool sphericalHarmonicsTransform( PCCPointSetGS &pointCloud, PCCAdaptationParameters &params );

  /// Transform SH coefficients from YUV to RGB
  bool sphericalHarmonicsInvTransform( PCCPointSetGS &pointCloud, const PCCAdaptationParameters &params );

  /// 2D SH transform logic (forward)
  bool sphericalHarmonicsTransform2D( PCCPointSetGS &pointCloud, PCCAdaptationParameters &params );

  /// 2D SH transform logic (inverse)
  bool sphericalHarmonicsInvTransform2D( PCCPointSetGS &pointCloud, const PCCAdaptationParameters &params );

  /// Compute KLT parameters for a group of SH coefficients
  bool computeShKlt( const PCCPointSetGS                       &pointCloud,
                     const std::vector<int>                    &groupidx,
                     PCCAdaptationParameters::PCCKltParameters &klt );

  // Compute mean absolute correlation across default SH groups
  double computeGroupCorr( const PCCPointSetGS &pointCloud );

  /// Extract SH quantization regions for non-uniform quantization
  bool extractSHQuantRegions( const PCCPointSetGS &input, PCCAdaptationParameters &params );

  /// Extract Scale quantization regions for non-uniform quantization
  bool extractScaleQuantRegions( const PCCPointSetGS &input, PCCAdaptationParameters &params );

  // ==================== Inverse Operations ====================

  /// Dequantize positions from int to float
  bool dequantizePositions( const PCCPointSet3 &input, PCCPointSetGS &output, const PCCAdaptationParameters &params );

  /// Dequantize spherical harmonics (with optional YUV->RGB conversion)
  bool dequantizeSphericalHarmonics( const PCCPointSet3            &input,
                                     PCCPointSetGS                 &output,
                                     const PCCAdaptationParameters &params );

  /// Dequantize opacity values
  bool dequantizeOpacities( const PCCPointSet3 &input, PCCPointSetGS &output, const PCCAdaptationParameters &params );

  /// Dequantize scale values
  bool dequantizeScales( const PCCPointSet3 &input, PCCPointSetGS &output, const PCCAdaptationParameters &params );

  /// Dequantize rotation quaternions
  bool dequantizeRotations( const PCCPointSet3 &input, PCCPointSetGS &output, const PCCAdaptationParameters &params );

  // ==================== Spherical Harmonics Reordering ====================

  /**
   * @brief Reorder AC coefficients between interleaved and non-interleaved layouts.
   *
   * Non-interleaved (toInterleaved=false): Partition 1 has R,G,B; Partition 2 has R,G,B, etc.
   *   - sh[0] = DC(R,G,B), sh[1] = AC1(R,G,B), sh[2] = AC2(R,G,B), ...
   *
   * Interleaved (toInterleaved=true): Partitions 1-5 all red, 6-10 all green, 11-15 all blue
   *   - sh[0] = DC(R,G,B), sh[1-5] = AC_R, sh[6-10] = AC_G, sh[11-15] = AC_B
   *
   * @param pointCloud Point cloud to reorder (modified in place)
   * @param toInterleaved If true, converts to interleaved (color-grouped) layout.
   *                      If false, converts to non-interleaved (coefficient-per-partition) layout.
   * @return true on success, false if point cloud has no spherical harmonics
   */
  static bool reorderSphericalHarmonicsAC( PCCPointSetGS &pointCloud, bool toInterleaved );

  /**
   * @brief Reorder AC coefficients between interleaved and non-interleaved layouts.
   *
   * Non-interleaved (toInterleaved=false): Partition 1 has R,G,B; Partition 2 has R,G,B, etc.
   *   - sh[0] = DC(R,G,B), sh[1] = AC1(R,G,B), sh[2] = AC2(R,G,B), ...
   *
   * Interleaved (toInterleaved=true): Partitions 1-5 all red, 6-10 all green, 11-15 all blue
   *   - sh[0] = DC(R,G,B), sh[1-5] = AC_R, sh[6-10] = AC_G, sh[11-15] = AC_B
   *
   * @param pointCloud Point cloud to reorder (modified in place)
   * @param toInterleaved If true, converts to interleaved (color-grouped) layout.
   *                      If false, converts to non-interleaved (coefficient-per-partition) layout.
   * @return true on success, false if point cloud has no spherical harmonics
   */
  static bool reorderSphericalHarmonicsAC( PCCPointSet3 &pointCloud, bool toInterleaved );

 private:
  // ==================== Helper Functions ====================

  /// Normalize value from [min, max] to [0, 1], with clamping
  static double normalizeToUnit( double value, double min, double max );

  /// Denormalize value from [0, 1] to [min, max]
  static double denormalizeFromUnit( double value, double min, double max );

  /// Quantize a normalized [0, 1] value to integer with given bit depth
  static uint32_t quantizeValue( double normalizedValue, uint32_t bitDepth );

  /// Dequantize integer value back to normalized [0, 1]
  static double dequantizeValue( uint32_t quantizedValue, uint32_t bitDepth );

  /// Canonicalize quaternion (ensure w >= 0, normalize)
  static void canonicalizeQuaternion( float &w, float &x, float &y, float &z );

  /// Convert quaternion [w, x, y, z] to 3x3 rotation matrix
  static void quatToRotMat( const GSRotation &q, PCCMatrix3D &R );

  /// Convert 3x3 rotation matrix to quaternion [w, x, y, z]
  static void rotMatToQuat( const PCCMatrix3D &R, GSRotation &q );

  /// Convert RGB to YUV (BT.601)
  static void rgbToYuv( double r, double g, double b, double &y, double &u, double &v );

  /// Convert YUV to RGB (BT.601)
  static void yuvToRgb( double y, double u, double v, double &r, double &g, double &b );

  /// Compute bounding box from double point cloud
  static void computeBoundingBox( const PCCPointSetGS   &pointCloud,
                                  std::array<double, 3> &minPoint,
                                  std::array<double, 3> &maxPoint );

  /// Compute min/max range from double vector
  static void computeRange( const std::vector<double> &values, double &minVal, double &maxVal );

  /// Convert logit opacity to linear opacity in-place for all points
  static void logitToLinearOpacity( PCCPointSetGS &pointCloud );

  /// Convert linear opacity to logit opacity in-place for all points
  static void linearToLogitOpacity( PCCPointSetGS &pointCloud );
};

}  // namespace pcc

#endif  // PCC_ADAPTATION_H
