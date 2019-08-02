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

#ifndef PCCImage_h
#define PCCImage_h

#include "PCCCommon.h"

namespace pcc {

class ChromaSampler {
 public:
  ChromaSampler() {}
  ~ChromaSampler() {}

  struct Filter {
    std::vector<float> data_;
    double             offset_;
    double             shift_;
  };
  struct Filter444to420 {
    Filter horizontal_;
    Filter vertical_;
  };
  struct Filter420to444 {
    Filter horizontal0_;
    Filter vertical0_;
    Filter horizontal1_;
    Filter vertical1_;
  };

  void downsampling( const std::vector<float>& chroma_in,
                     std::vector<float>&       chroma_out,
                     const int                 widthIn,
                     const int                 heightIn,
                     const int                 maxValue,
                     const size_t              filter ) const {
    static const std::vector<Filter444to420> g_filter444to420 = {
        {// 0 DF_F0
         {{+64.0, +384.0, +64.0}, +256.0, 9.0},
         {{+256.0, +256.0}, +256.0, 9.0}},
        {// 1 DF_F1
         {{+128.0, +256.0, +128.0}, +256.0, 9.0},
         {{+256.0, +256.0}, +256.0, 9.0}},
        {// 2 DF_TM5
         {{+21.0, +0.0, -52.0, +0.0, +159.0, +256.0, +159.0, +0.0, -52.0, 0.0, +21.0}, +256.0, 9.0},
         {{+5.0, +11.0, -21.0, -37.0, +70.0, +228.0, +228.0, +70.0, -37.0, -21.0, +11.0, +5.0}, +256.0, 9.0}},
        {// 3 DF_FV
         {{+8.0, 0.0, -64.0, +128.0, +368.0, +128.0, -64.0, 0.0, +8.0}, +256.0, 9.0},
         {{+8.0, 0.0, -24.0, +48.0, +224.0, +224.0, +48.0, -24.0, +0.0, +8.0}, +256.0, 9.0}},
        {// 4 DF_GS
         {{(float)( -0.01716352771649 * 512 ), (float)( 0.0 ), (float)( +0.04066666714886 * 512 ), (float)( 0.0 ),
           (float)( -0.09154810319329 * 512 ), (float)( 0.0 ), (float)( 0.31577823859943 * 512 ),
           (float)( 0.50453345032298 * 512 ), (float)( 0.31577823859943 * 512 ), (float)( 0.0 ),
           (float)( -0.09154810319329 * 512 ), (float)( 0.0 ), (float)( 0.04066666714886 * 512 ), (float)( 0.0 ),
           (float)( -0.01716352771649 * 512 )},
          +256.0,
          9.0},
         {{(float)( -0.00945406160902 * 512 ), (float)( -0.01539537217249 * 512 ), (float)( 0.02360533018213 * 512 ),
           (float)( 0.03519540819902 * 512 ), (float)( -0.05254456550808 * 512 ), (float)( -0.08189331229717 * 512 ),
           (float)( 0.14630826357715 * 512 ), (float)( 0.45417830962846 * 512 ), (float)( 0.45417830962846 * 512 ),
           (float)( 0.14630826357715 * 512 ), (float)( -0.08189331229717 * 512 ), (float)( -0.05254456550808 * 512 ),
           (float)( 0.03519540819902 * 512 ), (float)( 0.02360533018213 * 512 ), (float)( -0.01539537217249 * 512 ),
           (float)( -0.00945406160902 * 512 )

          },
          +256.0,
          9.0}},
        {// 5 DF_WCS
         {{2.0, -3.0, -9.0, 6.0, 39.0, 58.0, 39.0, 6.0, -9.0, -3.0, 2.0}, +64.0, 7.0},
         {{1.0, 0.0, -7.0, -5.0, 22.0, 53.0, 53.0, 22.0, -5.0, -7.0, 0.0, 1.0}, +64.0, 7.0}},
        {// 6 DF_SVC
         {{(float)0.016512788 * 256, (float)-9.77064E-18 * 256, (float)-0.075189625 * 256, (float)1.69232E-17 * 256,
           (float)0.308132812 * 256, (float)0.50108805 * 256, (float)0.308132812 * 256, (float)1.69232E-17 * 256,
           (float)-0.075189625 * 256, (float)-9.77064E-18 * 256, (float)0.016512788 * 256},
          128.0,
          8.0},
         {{(float)0.005353954 * 256, (float)0.019185222 * 256, (float)-0.039239076 * 256, (float)-0.071592303 * 256,
           (float)0.138951671 * 256, (float)0.447340531 * 256, (float)0.447340531 * 256, (float)0.138951671 * 256,
           (float)-0.071592303 * 256, (float)-0.039239076 * 256, (float)0.019185222 * 256, (float)0.005353954 * 256},
          128.0,
          8.0}},
        {// 7 DF_LZW
         {{(float)0.00190089262242 * 2048, (float)0.01183528116391 * 2048, (float)0.00000000000000 * 2048,
           (float)-0.04505642829799 * 2048, (float)-0.04734112465563 * 2048, (float)0.08323159585050 * 2048,
           (float)0.29483273086709 * 2048, (float)0.40119410489940 * 2048, (float)0.29483273086709 * 2048,
           (float)0.08323159585050 * 2048, (float)-0.04734112465563 * 2048, (float)-0.04505642829799 * 2048,
           (float)0.00000000000000 * 2048, (float)0.01183528116391 * 2048, (float)0.00190089262242 * 2048},
          1024,
          11},
         {{(float)0.00697781694123 * 2048, (float)0.01100508557770 * 2048, (float)-0.02103907505369 * 2048,
           (float)-0.05884522739626 * 2048, (float)0.00000000000000 * 2048, (float)0.18935167548323 * 2048,
           (float)0.37254972444780 * 2048, (float)0.37254972444780 * 2048, (float)0.18935167548323 * 2048,
           (float)0.00000000000000 * 2048, (float)-0.05884522739626 * 2048, (float)-0.02103907505369 * 2048,
           (float)0.01100508557770 * 2048, (float)0.00697781694123 * 2048},
          1024.0,
          11.0}},
        {// 8 DF_SNW
         {{(float)0.00571729514027 * 128.0, (float)0.01791720928626 * 128.0, (float)-0.02500536578022 * 128.0,
           (float)-0.07308632957148 * 128.0, (float)0.03993394132849 * 128.0, (float)0.30679811688568 * 128.0,
           (float)0.45545026542200 * 128.0, (float)0.30679811688568 * 128.0, (float)0.03993394132849 * 128.0,
           (float)-0.07308632957148 * 128.0, (float)-0.02500536578022 * 128.0, (float)0.01791720928626 * 128.0,
           (float)0.00571729514027 * 128.0},
          64.0,
          7.0},
         {{(float)0.00016629082500 * 256.0, (float)0.01501862112403 * 256.0, (float)0.00483721502879 * 256.0,
           (float)-0.05885266302048 * 256.0, (float)-0.04391848267708 * 256.0, (float)0.16770594201608 * 256.0,
           (float)0.41504307670367 * 256.0, (float)0.41504307670367 * 256.0, (float)0.16770594201608 * 256.0,
           (float)-0.04391848267708 * 256.0, (float)-0.05885266302048 * 256.0, (float)0.00483721502879 * 256.0,
           (float)0.01501862112403 * 256.0, (float)0.00016629082500 * 256.0},
          128.0,
          8.0}},
        {// 9 DF_LZ2
         {{(float)-0.00632274800093 * 256, (float)0.00000000000000 * 256, (float)0.02991834939159 * 256,
           (float)0.00000000000000 * 256, (float)-0.08310652608775 * 256, (float)0.00000000000000 * 256,
           (float)0.30981465204533 * 256, (float)0.49939254530351 * 256, (float)0.30981465204533 * 256,
           (float)0.00000000000000 * 256, (float)-0.08310652608775 * 256, (float)0.00000000000000 * 256,
           (float)0.02991834939159 * 256, (float)0.00000000000000 * 256, (float)-0.00632274800093 * 256},
          128.0,
          8.0},
         {{(float)-0.00198530798042 * 512.0, (float)-0.00752708715723 * 512.0, (float)0.01573387488128 * 512.0,
           (float)0.02772449226106 * 512.0, (float)-0.04583028312543 * 512.0, (float)-0.07615195442538 * 512.0,
           (float)0.14134196995234 * 512.0, (float)0.44669429559377 * 512.0, (float)0.44669429559377 * 512.0,
           (float)0.14134196995234 * 512.0, (float)-0.07615195442538 * 512.0, (float)-0.04583028312543 * 512.0,
           (float)0.02772449226106 * 512.0, (float)0.01573387488128 * 512.0, (float)-0.00752708715723 * 512.0,
           (float)-0.00198530798042 * 512.0},
          256.0,
          9.0}},
        {// 10 DF_SN2
         {{(float)-0.00886299106559 * 512.0, (float)0.00000000000000 * 512.0, (float)0.03533552841036 * 512.0,
           (float)0.00000000000000 * 512.0, (float)-0.08813892574297 * 512.0, (float)0.00000000000000 * 512.0,
           (float)0.31190085501694 * 512.0, (float)0.49953106676252 * 512.0, (float)0.31190085501694 * 512.0,
           (float)0.00000000000000 * 512.0, (float)0.08813892574297 * 512.0, (float)0.00000000000000 * 512.0,
           (float)0.03533552841036 * 512.0, (float)0.00000000000000 * 512.0, (float)-0.00886299106559 * 512.0},
          256.0,
          9.0},
         {{(float)-0.00293861597778 * 256.0, (float)-0.01004182911844 * 256.0, (float)0.01927196301938 * 256.0,
           (float)0.03169918776063 * 256.0, (float)-0.04966144977164 * 256.0, (float)-0.07932167506148 * 256.0,
           (float)0.14344838825693 * 256.0, (float)0.44754403089239 * 256.0, (float)0.44754403089239 * 256.0,
           (float)0.14344838825693 * 256.0, (float)-0.07932167506148 * 256.0, (float)-0.04966144977164 * 256.0,
           (float)0.03169918776063 * 256.0, (float)0.01927196301938 * 256.0, (float)-0.01004182911844 * 256.0,
           (float)-0.00293861597778 * 256.0},
          128.0,
          8.0}},
        {// 11 DF_SN3
         {{(float)-0.00067732252889 * 1024.0, (float)-0.01187238377794 * 1024.0, (float)0.01886537347052 * 1024.0,
           (float)0.03282746274537 * 1024.0, (float)-0.07278440675580 * 1024.0, (float)-0.04925606519713 * 1024.0,
           (float)0.30555914564961 * 1024.0, (float)0.55467639278853 * 1024.0, (float)0.30555914564961 * 1024.0,
           (float)-0.04925606519713 * 1024.0, (float)-0.07278440675580 * 1024.0, (float)0.03282746274537 * 1024.0,
           (float)0.01886537347052 * 1024.0, (float)-0.01187238377794 * 1024.0, (float)-0.00067732252889 * 1024.0},
          512.0,
          10.0},
         {{(float)-0.00699733029589 * 256.0, (float)-0.00364087058553 * 256.0, (float)0.03928171002786 * 256.0,
           (float)-0.01140308043259 * 256.0, (float)-0.10224241186895 * 256.0, (float)0.10042934057489 * 256.0,
           (float)0.48457264258020 * 256.0, (float)0.48457264258020 * 256.0, (float)0.10042934057490 * 256.0,
           (float)-0.10224241186895 * 256.0, (float)-0.01140308043259 * 256.0, (float)0.03928171002786 * 256.0,
           (float)-0.00364087058553 * 256.0, (float)-0.00699733029589 * 256.0},
          128.0,
          8.0}},
        {// 12 DF_LZ3
         {{(float)0.00480212008232 * 256.0, (float)-0.00409385963083 * 256.0, (float)-0.01900940873498 * 256.0,
           (float)0.02310408604406 * 256.0, (float)0.03610013444384 * 256.0, (float)-0.07603763493990 * 256.0,
           (float)-0.05014978047763 * 256.0, (float)0.30733568526821 * 256.0, (float)0.55589731588983 * 256.0,
           (float)0.30733568526821 * 256.0, (float)-0.05014978047763 * 256.0, (float)-0.07603763493990 * 256.0,
           (float)0.03610013444384 * 256.0, (float)0.02310408604406 * 256.0, (float)-0.01900940873498 * 256.0,
           (float)-0.00409385963083 * 256.0, (float)0.00480212008232 * 256.0},
          128.0,
          8.0},
         {{(float)0.00168005324555 * 256.0, (float)0.00405559129955 * 256.0, (float)-0.01554711355088 * 256.0,
           (float)-0.00492229194265 * 256.0, (float)0.04506212555057 * 256.0, (float)-0.01215504745022 * 256.0,
           (float)-0.10509848760398 * 256.0, (float)0.10138978248878 * 256.0, (float)0.48553538796329 * 256.0,
           (float)0.48553538796329 * 256.0, (float)0.10138978248878 * 256.0, (float)-0.10509848760398 * 256.0,
           (float)-0.01215504745022 * 256.0, (float)0.04506212555057 * 256.0, (float)-0.00492229194265 * 256.0,
           (float)-0.01554711355088 * 256.0, (float)0.00405559129955 * 256.0, (float)0.00168005324555 * 256.0},
          128.0,
          8.0}},
        {// 13 DF_LZ4
         {{(float)-0.00832391833024 * 1024.0, (float)0.00961238047504 * 1024.0, (float)0.03374816073530 * 1024.0,
           (float)-0.05999673019609 * 1024.0, (float)-0.06007737796902 * 1024.0, (float)0.29966105988877 * 1024.0,
           (float)0.57075285079248 * 1024.0, (float)0.29966105988877 * 1024.0, (float)-0.06007737796902 * 1024.0,
           (float)-0.05999673019609 * 1024.0, (float)0.03374816073530 * 1024.0, (float)0.00961238047504 * 1024.0,
           (float)-0.00832391833024 * 1024.0},
          512.0,
          10.0},
         {{(float)-0.00291663662971 * 1024.0, (float)-0.00633435442913 * 1024.0, (float)0.03072431945341 * 1024.0,
           (float)0.00000000000000 * 1024.0, (float)-0.09954679502905 * 1024.0, (float)0.08516187621385 * 1024.0,
           (float)0.49291159042062 * 1024.0, (float)0.49291159042062 * 1024.0, (float)0.08516187621385 * 1024.0,
           (float)-0.09954679502905 * 1024.0, (float)0.00000000000000 * 1024.0, (float)0.03072431945341 * 1024.0,
           (float)-0.00633435442913 * 1024.0, (float)-0.00291663662971 * 1024.0},
          512.0,
          10.0}},
        {
            // 14 DF_SNW3
            {{(float)0.26087818243737 * 1024.0, (float)0.47824363512526 * 1024.0, (float)0.26087818243737 * 1024.0},
             (float)512.0,
             (float)10.0},
            {{(float)-0.02250866038106 * 1024.0, (float)0.10249133961894 * 1024.0, (float)0.42001732076212 * 1024.0,
              (float)0.42001732076212 * 1024.0, (float)0.10249133961894 * 1024.0, (float)-0.02250866038106 * 1024.0},
             512.0,
             10.0},
        },
        {// 15 DF_SNW7
         {{(float)-0.06359139153628 * 1024.0, (float)0.00000000000000 * 1024.0, (float)0.30867909869282 * 1024.0,
           (float)0.50982458568693 * 1024.0, (float)0.30867909869282 * 1024.0, (float)0.00000000000000 * 1024.0,
           (float)-0.06359139153628 * 1024.0},
          512.0,
          10.0},
         {{(float)0.00793104893426 * 1024.0, (float)-0.02959302918114 * 1024.0, (float)-0.06452911298933 * 1024.0,
           (float)0.13551904324148 * 1024.0, (float)0.45067204999473 * 1024.0, (float)0.45067204999473 * 1024.0,
           (float)0.13551904324148 * 1024.0, (float)0.06452911298933 * 1024.0, (float)-0.02959302918114 * 1024.0,
           (float)0.00793104893426 * 1024.0},
          512.0,
          10.0}},
        {// 16 DF_SNW11
         {{(float)0.02247524825725 * 1024.0, (float)0.00000000000000 * 1024.0, (float)-0.07906905922190 * 1024.0,
           (float)0.00000000000000 * 1024.0, (float)0.30769680829831 * 1024.0, (float)0.49779400533267 * 1024.0,
           (float)0.30769680829831 * 1024.0, (float)0.00000000000000 * 1024.0, (float)-0.07906905922190 * 1024.0,
           (float)0.00000000000000 * 1024.0, (float)0.02247524825725 * 1024.0},
          512.0,
          10.0},
         {{(float)0.00535395438219 * 1024.0, (float)0.01918522222618 * 1024.0, (float)-0.03923907588008 * 1024.0,
           (float)-0.07159230301126 * 1024.0, (float)0.13895167108678 * 1024.0, (float)0.44734053119618 * 1024.0,
           (float)0.44734053119618 * 1024.0, (float)0.13895167108678 * 1024.0, (float)-0.07159230301126 * 1024.0,
           (float)-0.03923907588008 * 1024.0, (float)0.01918522222618 * 1024.0, (float)0.00535395438219 * 1024.0},
          512.0,
          10.0}},
        {// 17 DF_SNW15
         {{(float)-0.00886299106559 * 1024.0, (float)0.00000000000000 * 1024.0, (float)0.03533552841036 * 1024.0,
           (float)0.00000000000000 * 1024.0, (float)-0.08813892574297 * 1024.0, (float)0.00000000000000 * 1024.0,
           (float)0.31190085501694 * 1024.0, (float)0.49953106676252 * 1024.0, (float)0.31190085501694 * 1024.0,
           (float)0.00000000000000 * 1024.0, (float)-0.08813892574297 * 1024.0, (float)0.00000000000000 * 1024.0,
           (float)0.03533552841036 * 1024.0, (float)0.00000000000000 * 1024.0, (float)-0.00886299106559 * 1024.0},
          512.0,
          10.0},
         {{(float)-0.00385006522758 * 1024.0, (float)0.01342207139682 * 1024.0, (float)0.02642569966785 * 1024.0,
           (float)-0.04515627918959 * 1024.0, (float)-0.07570126817606 * 1024.0, (float)0.14064627966562 * 1024.0,
           (float)0.44421356186295 * 1024.0, (float)0.44421356186295 * 1024.0, (float)0.14064627966562 * 1024.0,
           (float)-0.07570126817606 * 1024.0, (float)-0.04515627918959 * 1024.0, (float)0.02642569966785 * 1024.0,
           (float)0.01342207139682 * 1024.0, (float)-0.00385006522758 * 1024.0},
          512.0,
          10.0}},
        {// 18 DF_SSW3
         {{(float)0.22473425589622 * 1024.0, (float)0.55053148820757 * 1024.0, (float)0.22473425589622 * 1024.0},
          (float)512.0,
          (float)10.0},
         {{(float)0.06930869439185 * 1024.0, (float)0.43069130560815 * 1024.0, (float)0.43069130560815 * 1024.0,
           (float).06930869439185 * 1024.0},
          512.0,
          10.0}},
        {// 19 DF_SSW5
         {{(float)-0.00963149700658 * 1024.0, (float)0.25498154223039 * 1024.0, (float)0.50929990955238 * 1024.0,
           (float)0.25498154223039 * 1024.0, (float)-0.00963149700658 * 1024.0},
          512.0,
          10.0},
         {{(float)-0.01970384769087 * 1024.0, (float)0.08511964870014 * 1024.0, (float)0.43458419899072 * 1024.0,
           (float)0.43458419899072 * 1024.0, (float)0.08511964870014 * 1024.0, (float)-0.01970384769087 * 1024.0},
          512.0,
          10.0}},
        {// 20 DF_SSW7
         {{(float)-0.03322983277383 * 1024.0, (float)-0.01765304050413 * 1024.0, (float)0.28904589924126 * 1024.0,
           (float)0.52367394807341 * 1024.0, (float)0.28904589924126 * 1024.0, (float)-0.01765304050413 * 1024.0,
           (float)-0.03322983277383 * 1024.0},
          512.0,
          10.0},
         {{(float)-0.01229101508504 * 1024.0, (float)-0.06539024269701 * 1024.0, (float)0.11223360796918 * 1024.0,
           (float)0.46544764981287 * 1024.0, (float)0.46544764981287 * 1024.0, (float)0.11223360796918 * 1024.0,
           (float)-0.06539024269701 * 1024.0, (float)-0.01229101508504 * 1024.0},
          512.0,
          10.0}},
        {// 21 DF_SSW11
         {{(float)0.01119294477411 * 1024.0, (float)0.01168400151526 * 1024.0, (float)-0.06976399068352 * 1024.0,
           (float)-0.02234931676070 * 1024.0, (float)0.30569641505283 * 1024.0, (float)0.52707989220403 * 1024.0,
           (float)0.30569641505283 * 1024.0, (float)-0.02234931676070 * 1024.0, (float)-0.06976399068352 * 1024.0,
           (float)0.01168400151526 * 1024.0, (float)0.01119294477411 * 1024.0},
          512.0,
          10.0},
         {{(float)0.00103847390257 * 1024.0, (float)0.02109981446330 * 1024.0, (float)-0.02474235739652 * 1024.0,
           (float)-0.08253987601651 * 1024.0, (float)0.11973940989643 * 1024.0, (float)0.46540453515074 * 1024.0,
           (float)0.46540453515074 * 1024.0, (float)0.11973940989643 * 1024.0, (float)-0.08253987601651 * 1024.0,
           (float)-0.02474235739652 * 1024.0, (float)0.02109981446330 * 1024.0, (float)0.00103847390257 * 1024.0},
          512.0,
          10.0}},
        {// 22 DF_SSW15
         {{(float)-0.00181153610954 * 1024.0, (float)-0.00658767969886 * 1024.0, (float)0.02692953876060 * 1024.0,
           (float)0.01656650338770 * 1024.0, (float)-0.08127280734884 * 1024.0, (float)-0.02362649736930 * 1024.0,
           (float)0.30806859050699 * 1024.0, (float)0.52346777574252 * 1024.0, (float)0.30806859050699 * 1024.0,
           (float)-0.02362649736930 * 1024.0, (float)-0.08127280734884 * 1024.0, (float)0.01656650338770 * 1024.0,
           (float)0.02692953876060 * 1024.0, (float)-0.00658767969886 * 1024.0, (float)-0.00181153610954 * 1024.0},
          512.0,
          10.0},
         {{(float)-0.00014417970451 * 1024.0, (float)-0.01068600519132 * 1024.0, (float)0.00789216874784 * 1024.0,
           (float)0.03867103862476 * 1024.0, (float)-0.03240217401926 * 1024.0, (float)-0.09255303893640 * 1024.0,
           (float)0.12396170720282 * 1024.0, (float)0.46526048327607 * 1024.0, (float)0.46526048327607 * 1024.0,
           (float)0.12396170720282 * 1024.0, (float)-0.09255303893640 * 1024.0, (float)-0.03240217401926 * 1024.0,
           (float)0.03867103862476 * 1024.0, (float)0.00789216874784 * 1024.0, (float)-0.01068600519132 * 1024.0,
           (float)-0.00014417970451 * 1024.0},
          512.0,
          10.0}}};

    int                widthOut  = widthIn / 2;
    int                heightOut = heightIn / 2;
    std::vector<float> temp;
    chroma_out.resize( widthOut * heightOut );
    temp.resize( widthOut * heightIn );
    for ( int i = 0; i < heightIn; i++ ) {
      for ( int j = 0; j < widthOut; j++ ) {
        temp[i * widthOut + j] =
            downsamplingHorizontal( g_filter444to420[filter], chroma_in, widthIn, heightIn, i, j * 2 );
      }
    }
    for ( int i = 0; i < heightOut; i++ ) {
      for ( int j = 0; j < widthOut; j++ ) {
        chroma_out[i * widthOut + j] =
            downsamplingVertical( g_filter444to420[filter], temp, widthOut, heightIn, 2 * i, j );
      }
    }
  }

  void upsampling( const std::vector<float>& chromaIn,
                   std::vector<float>&       chromaOut,
                   const int                 widthIn,
                   const int                 heightIn,
                   const int                 maxValue,
                   const size_t              filter ) const {
    static std::vector<Filter420to444> filter420to444 = {
        {// 0 UF_F0
         {{0.0, +256.0}, +128.0, 8.0},
         {{-8.0, +64.0, +216.0, -16.0}, +128.0, 8.0},
         {{-16.0, +144.0, +144.0, -16.0}, +128.0, 8.0},
         {{-16.0, +216.0, +64.0, -8.0}, +128.0, 8.0}},
        {// 1 UF_FV
         {{0.0, +256.0}, +128.0, 8.0},
         {{+0.0, -16.0, +56.0, +240.0, -32.0, +8.0}, +128.0, 8.0},
         {{-16.0, +144.0, +144.0, -16.0}, +128.0, 8.0},
         {{+8.0, -32.0, +240.0, +56.0, -16.0, +0.0}, +128.0, 8.0}},
        {// 2 UF_GS
         {{0.0, +256.0}, +128.0, 8.0},
         {{-6.0, +58.0, +222.0, -18.0}, +128.0, 8.0},
         {{-16.0, +144.0, +144.0, -16.0}, +128.0, 8.0},
         {{-18.0, +222.0, +58.0, -6.0}, +128.0, 8.0}},
        {// 3 UF_LS3
         {{0.0, +256.0}, +128.0, 8.0},
         {{+2.0, -18.0, +70.0, +228.0, -34.0, +8.0}, +128.0, 8.0},
         {{+6.0, -34.0, +156.0, +156.0, -34.0, +6.0}, +128.0, 8.0},
         {{+8.0, -34.0, +228.0, +70.0, -18.0, +2.0}, +128.0, 8.0}},
        {// 4 UF_LS4
         {{0.0, +256.0}, +128.0, 8.0},
         {{-1.0, +8.0, -23.0, +72.0, +229.0, -39.0, +14.0, -4.0}, +128.0, 8.0},
         {{-3.0, +15.0, -43.0, +159.0, +159.0, -43.0, +15.0, -3.0}, +128.0, 8.0},
         {{-4.0, +14.0, -39.0, +229.0, +72.0, -23.0, +8.0, -1.0}, +128.0, 8.0}},
        {// 5 UF_TM
         {{0.0, +256.0}, +128.0, 8.0},
         {{+3.0, -16.0, +67.0, +227.0, -32.0, +7.0}, +128.0, 8.0},
         {{+21.0, -52.0, +159.0, +159.0, -52.0, +21.0}, +128.0, 8.0},
         {{+7.0, -32.0, +227.0, +67.0, -16.0, +3.0}, +128.0, 8.0}},
        {// 6 UF_LS5
         {{0.0, +256.0}, +128.0, 8.0},
         {{+1.0, -5.0, +12.0, -27.0, +74.0, +230.0, -41.0, +18.0, -8.0, +2.0}, +128.0, 8.0},
         {{+2.0, -8.0, +21.0, -47.0, +160.0, +160.0, -47.0, +21.0, -8.0, +2.0}, +128.0, 8.0},
         {{+2.0, -8.0, +18.0, -41.0, +230.0, +74.0, -27.0, +12.0, -5.0, +1.0}, +128.0, 8.0}},
        {// 7 UF_LS6
         {{0.0, +256.0}, +128.0, 8.0},
         {{0.0, +3.0, -7.0, +14.0, -29.0, +75.0, +230.0, -43.0, +20.0, -10.0, +5.0, -2.0}, +128.0, 8.0},
         {{-1.0, +5.0, -12.0, +24.0, -49.0, +161.0, +161.0, -49.0, +24.0, -12.0, +5.0, -1.0}, +128.0, 8.0},
         {{-2.0, +5.0, -10.0, +20.0, -43.0, +230.0, +75.0, -29.0, +14.0, -7.0, +3.0, 0.0}, +128.0, 8.0}}};
    int                widthOut = widthIn * 2, heightOut = heightIn * 2;
    std::vector<float> temp;
    chromaOut.resize( widthOut * heightOut );
    temp.resize( widthIn * heightOut );
    for ( int i = 0; i < heightIn; i++ ) {
      for ( int j = 0; j < widthIn; j++ ) {
        temp[( 2 * i ) * widthIn + j] =
            upsamplingVertical0( filter420to444[filter], chromaIn, widthIn, heightIn, i + 0, j );
        temp[( 2 * i + 1 ) * widthIn + j] =
            upsamplingVertical1( filter420to444[filter], chromaIn, widthIn, heightIn, i + 1, j );
      }
    }
    for ( int i = 0; i < heightOut; i++ ) {
      for ( int j = 0; j < widthIn; j++ ) {
        chromaOut[i * widthOut + j * 2] =
            upsamplingHorizontal0( filter420to444[filter], temp, widthIn, heightOut, i, j + 0 );
        chromaOut[i * widthOut + j * 2 + 1] =
            upsamplingHorizontal1( filter420to444[filter], temp, widthIn, heightOut, i, j + 1 );
      }
    }
  }

 private:
  int          clamp( int v, int a, int b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  inline float downsamplingHorizontal( const Filter444to420&     filter,
                                       const std::vector<float>& im,
                                       const int                 width,
                                       const int                 height,
                                       const int                 i0,
                                       const int                 j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.horizontal_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.horizontal_.data_.size() - 1 ) >> 1;
    double      value    = 0;
    for ( int j = 0; j < (int)filter.horizontal_.data_.size(); j++ ) {
      value +=
          (double)filter.horizontal_.data_[j] * (double)( im[i0 * width + clamp( j0 + j - position, 0, width - 1 )] );
    }
    return (float)( ( value + (double)offset ) * (double)scale );
  }
  inline float downsamplingVertical( const Filter444to420&     filter,
                                     const std::vector<float>& im,
                                     const int                 width,
                                     const int                 height,
                                     const int                 i0,
                                     const int                 j0 ) const {
    const float offset   = 0;
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.vertical_.shift_ ) ) );
    const int   position = int( filter.vertical_.data_.size() - 1 ) >> 1;
    ;
    double value = 0;
    for ( int i = 0; i < (int)filter.vertical_.data_.size(); i++ ) {
      value +=
          (double)filter.vertical_.data_[i] * (double)( im[clamp( i0 + i - position, 0, height - 1 ) * width + j0] );
    }
    return (float)( ( value + (double)offset ) * (double)scale );
  }

  inline float upsamplingVertical0( const Filter420to444&     filter,
                                    const std::vector<float>& im,
                                    const int                 width,
                                    const int                 height,
                                    const int                 i0,
                                    const int                 j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.vertical0_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.vertical0_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int i = 0; i < (int)filter.vertical0_.data_.size(); i++ ) {
      value += filter.vertical0_.data_[i] * (float)( im[clamp( i0 + i - position, 0, height - 1 ) * width + j0] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }
  inline float upsamplingVertical1( const Filter420to444&     filter,
                                    const std::vector<float>& im,
                                    int                       width,
                                    int                       height,
                                    int                       i0,
                                    int                       j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.vertical1_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.vertical1_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int i = 0; i < (int)filter.vertical1_.data_.size(); i++ ) {
      value += filter.vertical1_.data_[i] * (float)( im[clamp( i0 + i - position, 0, height - 1 ) * width + j0] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }
  inline float upsamplingHorizontal0( const Filter420to444&     filter,
                                      const std::vector<float>& im,
                                      int                       width,
                                      int                       height,
                                      int                       i0,
                                      int                       j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.horizontal0_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.horizontal0_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int j = 0; j < (int)filter.horizontal0_.data_.size(); j++ ) {
      value += filter.horizontal0_.data_[j] * (float)( im[i0 * width + clamp( j0 + j - position, 0, width - 1 )] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }
  inline float upsamplingHorizontal1( const Filter420to444&     filter,
                                      const std::vector<float>& im,
                                      int                       width,
                                      int                       height,
                                      int                       i0,
                                      int                       j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.horizontal1_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.horizontal1_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int j = 0; j < (int)filter.horizontal1_.data_.size(); j++ ) {
      value += filter.horizontal1_.data_[j] * (float)( im[i0 * width + clamp( j0 + j - position, 0, width - 1 )] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }
};

template <typename T, size_t N>
class PCCImage {
 public:
  PCCImage() : width_( 0 ), height_( 0 ) {}
  PCCImage( const PCCImage& ) = default;
  PCCImage& operator=( const PCCImage& rhs ) = default;
  ~PCCImage()                                = default;

  void clear() {
    for ( auto& channel : channels_ ) { channel.clear(); }
  }
  size_t getWidth() const { return width_; }
  size_t getHeight() const { return height_; }
  size_t getDepth() const { return sizeof( T ) * 8; }
  size_t getChannelCount() const { return N; }

  void set( const T value = 0 ) {
    for ( auto& channel : channels_ ) {
      for ( auto& p : channel ) { p = value; }
    }
  }
  void resize( const size_t sizeU0, const size_t sizeV0 ) {
    width_            = sizeU0;
    height_           = sizeV0;
    const size_t size = sizeU0 * sizeV0;
    for ( auto& channel : channels_ ) { channel.resize( size ); }
  }
  bool write420( std::ofstream& outfile, const size_t nbyte, bool convert = false, const size_t filter = 4 ) const {
    if ( !outfile.good() ) { return false; } //jkei : do we need to make it more general?? 
    if ( convert ) {
      std::vector<float> RGB444[3], YUV444[3], YUV420[3];
      std::vector<T>     YUV420T[3];
      ChromaSampler      chromaSampler;
      RGBtoFloatRGB( channels_[0], RGB444[0], nbyte );
      RGBtoFloatRGB( channels_[1], RGB444[1], nbyte );
      RGBtoFloatRGB( channels_[2], RGB444[2], nbyte );
      convertRGBToYUV( RGB444[0], RGB444[1], RGB444[2], YUV444[0], YUV444[1], YUV444[2] );
      copy( YUV444[0], YUV420[0] );
      chromaSampler.downsampling( YUV444[1], YUV420[1], (int)width_, (int)height_, nbyte == 1 ? 255 : 1023, filter );
      chromaSampler.downsampling( YUV444[2], YUV420[2], (int)width_, (int)height_, nbyte == 1 ? 255 : 1023, filter );
      floatYUVToYUV( YUV420[0], YUV420T[0], 0, nbyte );
      floatYUVToYUV( YUV420[1], YUV420T[1], 1, nbyte );
      floatYUVToYUV( YUV420[2], YUV420T[2], 1, nbyte );
      outfile.write( (const char*)( YUV420T[0].data() ), width_ * height_ * ( nbyte == 1 ? 1 : sizeof( T ) ) );
      outfile.write( (const char*)( YUV420T[1].data() ), width_ * height_ * ( nbyte == 1 ? 1 : sizeof( T ) ) / 4 );
      outfile.write( (const char*)( YUV420T[2].data() ), width_ * height_ * ( nbyte == 1 ? 1 : sizeof( T ) ) / 4 );
    } else {
      if ( nbyte == 1 ) {
        std::vector<uint8_t> channels[N];
        for ( size_t i = 0; i < N; i++ ) {
          channels[i].resize( channels_[i].size() );
          ( std::copy )( channels_[i].begin(), channels_[i].end(), channels[i].begin() );
        }
        outfile.write( (const char*)( channels[0].data() ), width_ * height_ );
        std::vector<uint8_t> chroma;
        const size_t         width2     = width_ / 2;
        const size_t         byteCount2 = width2;
        chroma.resize( width2 );
        for ( size_t c = 1; c < N; ++c ) {
          const auto& channel = channels[c];
          for ( size_t y = 0; y < height_; y += 2 ) {
            const uint8_t* const buffer1 = channel.data() + y * width_;
            const uint8_t* const buffer2 = buffer1 + width_;
            for ( size_t x = 0; x < width_; x += 2 ) {
              const size_t   x2  = x / 2;
              const uint64_t sum = buffer1[x] + buffer1[x + 1] + buffer2[x] + buffer2[x + 1];
              chroma[x2]         = uint8_t( ( sum + 2 ) / 4 );
            }
            outfile.write( (const char*)( chroma.data() ), byteCount2 );
          }
        }
      } else {
        outfile.write( (const char*)( channels_[0].data() ), width_ * height_ * sizeof( T ) );
        std::vector<T> chroma;
        const size_t   width2     = width_ / 2;
        const size_t   byteCount2 = width2 * sizeof( T );
        chroma.resize( width2 );
        for ( size_t c = 1; c < N; ++c ) {
          const auto& channel = channels_[c];
          for ( size_t y = 0; y < height_; y += 2 ) {
            const T* const buffer1 = channel.data() + y * width_;
            const T* const buffer2 = buffer1 + width_;
            for ( size_t x = 0; x < width_; x += 2 ) {
              const size_t   x2  = x / 2;
              const uint64_t sum = buffer1[x] + buffer1[x + 1] + buffer2[x] + buffer2[x + 1];
              chroma[x2]         = T( ( sum + 2 ) / 4 );
            }
            outfile.write( (const char*)( chroma.data() ), byteCount2 );
          }
        }
      }
    }
    return true;
  }
  bool write( std::ofstream& outfile, const size_t nbyte ) const {
    if ( !outfile.good() ) { return false; }
    if ( nbyte == 1 ) {
      std::vector<uint8_t> channels[N];
      for ( size_t i = 0; i < N; i++ ) {
        channels[i].resize( channels_[i].size() );
        ( std::copy )( channels_[i].begin(), channels_[i].end(), channels[i].begin() );
      }
      const size_t byteCount = width_ * height_;
      for ( const auto& channel : channels ) { outfile.write( (const char*)( channel.data() ), byteCount ); }
    } else {
      const size_t byteCount = width_ * height_ * sizeof( T );
      for ( const auto& channel : channels_ ) { outfile.write( (const char*)( channel.data() ), byteCount ); }
    }
    return true;
  }
  bool write( const std::string fileName, const size_t nbyte ) const {
    std::ofstream outfile( fileName, std::ios::binary );
    if ( write( outfile, nbyte ) ) {
      outfile.close();
      return true;
    }
    return false;
  }
  bool read420( std::ifstream& infile,
                const size_t   sizeU0,
                const size_t   sizeV0,
                const size_t   nbyte,
                const bool     convert = false,
                const size_t   filter  = 0 ) {
    if ( !infile.good() ) { return false; }
    resize( sizeU0, sizeV0 );
    if ( convert ) {
      size_t             widthChroma  = width_ / 2;
      size_t             heightChroma = height_ / 2;
      std::vector<float> RGB444[3], YUV444[3], YUV420[3];
      std::vector<T>     YUV420T[3];
      ChromaSampler      chromaSampler;
      YUV420T[0].resize( width_ * height_ );
      YUV420T[1].resize( widthChroma * heightChroma );
      YUV420T[2].resize( widthChroma * heightChroma );
      infile.read( (char*)( YUV420T[0].data() ), width_ * height_ * ( nbyte == 1 ? 1 : sizeof( T ) ) );
      infile.read( (char*)( YUV420T[1].data() ), widthChroma * heightChroma * ( nbyte == 1 ? 1 : sizeof( T ) ) );
      infile.read( (char*)( YUV420T[2].data() ), widthChroma * heightChroma * ( nbyte == 1 ? 1 : sizeof( T ) ) );
      YUVtoFloatYUV( YUV420T[0], YUV420[0], 0, nbyte );
      YUVtoFloatYUV( YUV420T[1], YUV420[1], 1, nbyte );
      YUVtoFloatYUV( YUV420T[2], YUV420[2], 1, nbyte );
      copy( YUV420[0], YUV444[0] );
      chromaSampler.upsampling( YUV420[1], YUV444[1], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
      chromaSampler.upsampling( YUV420[2], YUV444[2], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
      convertYUVToRGB( YUV444[0], YUV444[1], YUV444[2], RGB444[0], RGB444[1], RGB444[2] );
      floatRGBToRGB( RGB444[0], channels_[0], nbyte );
      floatRGBToRGB( RGB444[1], channels_[1], nbyte );
      floatRGBToRGB( RGB444[2], channels_[2], nbyte );
    } else {
      if ( nbyte == 1 ) {
        std::vector<uint8_t> channels[N];
        for ( size_t i = 0; i < N; i++ ) { channels[i].resize( channels_[i].size() ); }
        infile.read( (char*)( channels[0].data() ), width_ * height_ );
        std::vector<uint8_t> chroma;
        const size_t         width2     = width_ / 2;
        const size_t         byteCount1 = width_;
        const size_t         byteCount2 = width2;
        chroma.resize( width2 );
        for ( size_t c = 1; c < N; ++c ) {
          auto& channel = channels[c];
          for ( size_t y = 0; y < height_; y += 2 ) {
            infile.read( (char*)( chroma.data() ), byteCount2 );
            uint8_t* const buffer1 = channel.data() + y * width_;
            uint8_t* const buffer2 = buffer1 + width_;
            for ( size_t x2 = 0; x2 < width2; ++x2 ) {
              const size_t  x     = x2 * 2;
              const uint8_t value = chroma[x2];
              buffer1[x]          = value;
              buffer1[x + 1]      = value;
            }
            memcpy( (char*)buffer2, (char*)buffer1, byteCount1 );
          }
        }
        for ( size_t i = 0; i < N; i++ ) {
          ( std::copy )( channels[i].begin(), channels[i].end(), channels_[i].begin() );
        }
      } else {
        infile.read( (char*)( channels_[0].data() ), width_ * height_ * sizeof( T ) );
        std::vector<T> chroma;
        const size_t   width2     = width_ / 2;
        const size_t   byteCount1 = width_ * sizeof( T );
        const size_t   byteCount2 = width2 * sizeof( T );
        chroma.resize( width2 );
        for ( size_t c = 1; c < N; ++c ) {
          auto& channel = channels_[c];
          for ( size_t y = 0; y < height_; y += 2 ) {
            infile.read( (char*)( chroma.data() ), byteCount2 );
            T* const buffer1 = channel.data() + y * width_;
            T* const buffer2 = buffer1 + width_;
            for ( size_t x2 = 0; x2 < width2; ++x2 ) {
              const size_t x     = x2 * 2;
              const T      value = chroma[x2];
              buffer1[x]         = value;
              buffer1[x + 1]     = value;
            }
            memcpy( (char*)buffer2, (char*)buffer1, byteCount1 );
          }
        }
      }
    }
    return true;
  }
  bool read( std::ifstream& infile, const size_t sizeU0, const size_t sizeV0, const size_t nbyte ) {
    if ( !infile.good() ) { return false; }
    resize( sizeU0, sizeV0 );
    if ( nbyte == 1 ) {
      std::vector<uint8_t> channels[N];
      for ( size_t i = 0; i < N; i++ ) { channels[i].resize( channels_[i].size() ); }
      const size_t byteCount = width_ * height_;
      for ( auto& channel : channels ) { infile.read( (char*)( channel.data() ), byteCount ); }
      for ( size_t i = 0; i < N; i++ ) {
        ( std::copy )( channels[i].begin(), channels[i].end(), channels_[i].begin() );
      }
    } else {
      const size_t byteCount = width_ * height_ * sizeof( T );
      for ( auto& channel : channels_ ) { infile.read( (char*)( channel.data() ), byteCount ); }
    }
    return true;
  }
  bool read( const std::string fileName, const size_t sizeU0, const size_t sizeV0, const size_t nbyte ) {
    std::ifstream infile( fileName, std::ios::binary );
    if ( read( infile, sizeU0, sizeV0, nbyte ) ) {
      infile.close();
      return true;
    }
    return false;
  }
  void setValue( const size_t channelIndex, const size_t u, const size_t v, const T value ) {
    assert( channelIndex < N && u < width_ && v < height_ );
    channels_[channelIndex][v * width_ + u] = value;
  }
  T getValue( const size_t channelIndex, const size_t u, const size_t v ) const {
    assert( channelIndex < N && u < width_ && v < height_ );
    return channels_[channelIndex][v * width_ + u];
  }
  T& getValue( const size_t channelIndex, const size_t u, const size_t v ) {
    assert( channelIndex < N && u < width_ && v < height_ );
    return channels_[channelIndex][v * width_ + u];
  }
  static const size_t MaxValue     = ( std::numeric_limits<T>::max )();
  static const size_t HalfMaxValue = ( MaxValue + 1 ) / 2;

  bool write420( const std::string fileName,
                 const size_t      nbyte,
                 const bool        convert = false,
                 const size_t      filter  = 0 ) const {
    std::ofstream outfile( fileName, std::ios::binary );
    if ( write420( outfile, nbyte, convert, filter ) ) {
      outfile.close();
      return true;
    }
    return false;
  }

  bool read420( const std::string fileName,
                const size_t      sizeU0,
                const size_t      sizeV0,
                const size_t      nbyte,
                const bool        convert = false,
                const size_t      filter  = 0 ) {
    std::ifstream infile( fileName, std::ios::binary );
    if ( read420( infile, sizeU0, sizeV0, nbyte, convert, filter ) ) {
      infile.close();
      return true;
    }
    return false;
  }

  bool copyBlock( size_t top, size_t left, size_t width, size_t height, PCCImage& block ) {
    assert( top >= 0 && left >= 0 && ( width + left ) < width_ && ( height + top ) < height_ );
    for ( size_t cc = 0; cc < N; cc++ ) {
      for ( size_t i = top; i < top + height; i++ ) {
        for ( size_t j = left; j < left + width; j++ ) {
          block.setValue( cc, ( j - left ), ( i - top ), getValue( cc, j, i ) );
        }
      }
    }
    return true;
  }
  bool setBlock( size_t top, size_t left, PCCImage& block ) {
    assert( top >= 0 && left >= 0 && ( block.getWidth() + left ) < width_ && ( block.getHeight() + top ) < height_ );
    for ( size_t cc = 0; cc < N; cc++ ) {
      for ( size_t i = top; i < top + block.getHeight(); i++ ) {
        for ( size_t j = left; j < left + block.getWidth(); j++ ) {
          setValue( cc, j, i, block.getValue( cc, ( j - left ), ( i - top ) ) );
        }
      }
    }
    return true;
  }

 private:
  T      clamp( T v, T a, T b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  int    clamp( int v, int a, int b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  float  clamp( float v, float a, float b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  double clamp( double v, double a, double b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }

  void copy( const std::vector<float>& src, std::vector<float>& dst ) const {
    size_t count = src.size();
    dst.resize( count );
    for ( size_t i = 0; i < count; i++ ) { dst[i] = src[i]; }
  }

  void RGBtoFloatRGB( const std::vector<T>& src, std::vector<float>& dst, const size_t nbyte ) const {
    size_t count = src.size();
    dst.resize( count );
    float offset = nbyte == 1 ? 255.f : 1023.f;
    for ( size_t i = 0; i < count; i++ ) { dst[i] = (float)src[i] / offset; }
  }
  void convertRGBToYUV( const std::vector<float>& R,
                        const std::vector<float>& G,
                        const std::vector<float>& B,
                        std::vector<float>&       Y,
                        std::vector<float>&       U,
                        std::vector<float>&       V ) const {
    size_t count = R.size();
    Y.resize( count );
    U.resize( count );
    V.resize( count );
    for ( size_t i = 0; i < count; i++ ) {
      Y[i] = (float)( (double)clamp( 0.212600 * R[i] + 0.715200 * G[i] + 0.072200 * B[i], 0.0, 1.0 ) );
      U[i] = (float)( (double)clamp( -0.114572 * R[i] - 0.385428 * G[i] + 0.500000 * B[i], -0.5, 0.5 ) );
      V[i] = (float)( (double)clamp( 0.500000 * R[i] - 0.454153 * G[i] - 0.045847 * B[i], -0.5, 0.5 ) );
    }
  }
  static inline float fMin( float a, float b ) { return ( ( a ) < ( b ) ) ? ( a ) : ( b ); }
  static inline float fMax( float a, float b ) { return ( ( a ) > ( b ) ) ? ( a ) : ( b ); }
  static inline float fClip( float x, float low, float high ) { return fMin( fMax( x, low ), high ); }
  void                floatYUVToYUV( const std::vector<float>& src,
                                     std::vector<T>&           dst,
                                     const bool                chroma,
                                     const size_t              nbyte ) const {
    size_t count = src.size();
    dst.resize( count );
    double offset = chroma ? nbyte == 1 ? 128. : 512. : 0;
    double scale  = nbyte == 1 ? 255. : 1023.;
    for ( size_t i = 0; i < count; i++ ) {
      dst[i] = static_cast<T>( fClip( std::round( (float)( scale * (double)src[i] + offset ) ), 0.f, (float)scale ) );
    }
  }
  void YUVtoFloatYUV( const std::vector<T>& src,
                      std::vector<float>&   dst,
                      const bool            chroma,
                      const size_t          nbBytes ) const {
    size_t count = src.size();
    dst.resize( count );
    float    minV   = chroma ? -0.5f : 0.f;
    float    maxV   = chroma ? 0.5f : 1.f;
    uint16_t offset = chroma ? nbBytes == 1 ? 128 : 512 : 0;
    double   scale  = nbBytes == 1 ? 255. : 1023.;
    double   weight = 1.0 / scale;
    for ( size_t i = 0; i < count; i++ ) {
      dst[i] = clamp( (float)( weight * (double)( src[i] - offset ) ), minV, maxV );
    }
  }

  void convertYUVToRGB( const std::vector<float>& Y,
                        const std::vector<float>& U,
                        const std::vector<float>& V,
                        std::vector<float>&       R,
                        std::vector<float>&       G,
                        std::vector<float>&       B ) const {
    size_t count = Y.size();
    R.resize( count );
    G.resize( count );
    B.resize( count );
    for ( size_t i = 0; i < count; i++ ) {
      R[i] = (float)( (double)clamp( Y[i] + 1.57480 * V[i], 0.0, 1.0 ) );
      G[i] = (float)( (double)clamp( Y[i] - 0.18733 * U[i] - 0.46813 * V[i], 0.0, 1.0 ) );
      B[i] = (float)( (double)clamp( Y[i] + 1.85563 * U[i], 0.0, 1.0 ) );
    }
  }
  void floatRGBToRGB( const std::vector<float>& src, std::vector<T>& dst, const size_t nbyte ) const {
    size_t count = src.size();
    dst.resize( count );
    float scale = nbyte == 1 ? 255.f : 1023.f;
    for ( size_t i = 0; i < count; i++ ) {
      dst[i] = static_cast<T>( clamp( (T)std::round( scale * src[i] ), (T)0, (T)scale ) );
    }
  }

  size_t         width_;
  size_t         height_;
  std::vector<T> channels_[N];
};
}  // namespace pcc

#endif /* PCCImage_h */
