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
#include "PCCInternalColorConverter.h"

using namespace pcc;

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
       (float)( -0.00945406160902 * 512 )},
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

static std::vector<Filter420to444> g_filter420to444 = {
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

template <typename T>
PCCInternalColorConverter<T>::PCCInternalColorConverter() {}

template <typename T>
PCCInternalColorConverter<T>::~PCCInternalColorConverter() {}

template <typename T>
void PCCInternalColorConverter<T>::convert( std::string        configuration,
                                            PCCVideo<T, 3>&    videoSrc,
                                            PCCVideo<T, 3>&    videoDst,
                                            const std::string& externalPath,
                                            const std::string& fileName ) {
  std::string config   = "";
  int32_t     bitdepth = -1;
  int32_t     filter   = -1;
  extractParameters( configuration, config, bitdepth, filter );
  if ( config.empty() || bitdepth == -1 ) {
    printf( "ColorConverter configuration is not correct ( %s %d %d ) \n", config.c_str(), bitdepth, filter );
    exit( -1 );
  }
  printf( "ColorConverter configuration : %s bitdepth = %d filter = %d videoSrc %zu frames \n", config.c_str(),
          bitdepth, filter, videoSrc.getFrameCount() );
  videoDst.clear();
  if ( config == "YUV420ToYUV444" ) {
    convertYUV420ToYUV444( videoSrc, videoDst, bitdepth == 8 ? 1 : 2, filter );
  } else if ( config == "YUV420ToRGB444" ) {
    convertYUV420ToRGB444( videoSrc, videoDst, bitdepth == 8 ? 1 : 2, filter );
  } else if ( config == "RGB444ToYUV420" ) {
    convertRGB44ToYUV420( videoSrc, videoDst, bitdepth == 8 ? 1 : 2, filter );
  } else if ( config == "RGB444ToYUV444" ) {
    convertRGB44ToYUV444( videoSrc, videoDst, bitdepth == 8 ? 1 : 2, filter );
  } else if ( config == "YUV444ToRGB444" ) {
    convertYUV444ToRGB444( videoSrc, videoDst, bitdepth == 8 ? 1 : 2, filter );
  } else {
    printf( "PCCInternalColorConverter convert format not supported: %s \n", config.c_str() );
    exit( -1 );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::extractParameters( std::string& configuration,
                                                      std::string& config,
                                                      int32_t&     bitdepth,
                                                      int32_t&     filter ) {
  size_t pos = 0;
  if ( ( pos = configuration.find( "_" ) ) != std::string::npos ) {
    config = configuration.substr( 0, pos );
    configuration.erase( 0, pos + 1 );
    if ( ( pos = configuration.find( "_" ) ) != std::string::npos ) {
      bitdepth = std::stoi( configuration.substr( 0, pos ) );
      configuration.erase( 0, pos + 1 );
      filter = std::stoi( configuration.substr( 0, std::string::npos ) );
    }
  }
}

template <typename T>
void PCCInternalColorConverter<T>::convertRGB44ToYUV420( PCCVideo<T, 3>& videoSrc,
                                                         PCCVideo<T, 3>& videoDst,
                                                         size_t          nbyte,
                                                         size_t          filter ) {
  videoDst.resize( videoSrc.getFrameCount() );
  for ( size_t i = 0; i < videoSrc.getFrameCount(); i++ ) {
    convertRGB44ToYUV420( videoSrc[i], videoDst[i], nbyte, filter );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::convertRGB44ToYUV420( PCCImage<T, 3>& imageSrc,
                                                         PCCImage<T, 3>& imageDst,
                                                         size_t          nbyte,
                                                         size_t          filter ) {
  int width  = (int)imageSrc.getWidth();
  int height = (int)imageSrc.getHeight();
  imageDst.resize( width, height, pcc::PCCCOLORFORMAT::YUV420 );
  std::vector<float> RGB444[3], YUV444[3], YUV420[3];
  RGBtoFloatRGB( imageSrc[0], RGB444[0], nbyte );
  RGBtoFloatRGB( imageSrc[1], RGB444[1], nbyte );
  RGBtoFloatRGB( imageSrc[2], RGB444[2], nbyte );
  convertRGBToYUV( RGB444[0], RGB444[1], RGB444[2], YUV444[0], YUV444[1], YUV444[2] );
  downsampling( YUV444[1], YUV420[1], width, height, nbyte == 1 ? 255 : 1023, filter );
  downsampling( YUV444[2], YUV420[2], width, height, nbyte == 1 ? 255 : 1023, filter );
  floatYUVToYUV( YUV444[0], imageDst[0], 0, nbyte );
  floatYUVToYUV( YUV420[1], imageDst[1], 1, nbyte );
  floatYUVToYUV( YUV420[2], imageDst[2], 1, nbyte );
}

template <typename T>
void PCCInternalColorConverter<T>::convertRGB44ToYUV444( PCCVideo<T, 3>& videoSrc,
                                                         PCCVideo<T, 3>& videoDst,
                                                         size_t          nbyte,
                                                         size_t          filter ) {
  videoDst.resize( videoSrc.getFrameCount() );
  for ( size_t i = 0; i < videoSrc.getFrameCount(); i++ ) {
    convertRGB44ToYUV444( videoSrc[i], videoDst[i], nbyte, filter );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::convertRGB44ToYUV444( PCCImage<T, 3>& imageSrc,
                                                         PCCImage<T, 3>& imageDst,
                                                         size_t          nbyte,
                                                         size_t          filter ) {
  int width  = (int)imageSrc.getWidth();
  int height = (int)imageSrc.getHeight();
  imageDst.resize( width, height, pcc::PCCCOLORFORMAT::YUV444 );
  std::vector<float> RGB444[3], YUV444[3];
  RGBtoFloatRGB( imageSrc[0], RGB444[0], nbyte );
  RGBtoFloatRGB( imageSrc[1], RGB444[1], nbyte );
  RGBtoFloatRGB( imageSrc[2], RGB444[2], nbyte );
  convertRGBToYUV( RGB444[0], RGB444[1], RGB444[2], YUV444[0], YUV444[1], YUV444[2] );
  floatYUVToYUV( YUV444[0], imageDst[0], 0, nbyte );
  floatYUVToYUV( YUV444[1], imageDst[1], 1, nbyte );
  floatYUVToYUV( YUV444[2], imageDst[2], 1, nbyte );
}

template <typename T>
void PCCInternalColorConverter<T>::convertYUV420ToYUV444( PCCVideo<T, 3>& videoSrc,
                                                          PCCVideo<T, 3>& videoDst,
                                                          size_t          nbyte,
                                                          size_t          filter ) {
  videoDst.resize( videoSrc.getFrameCount() );
  for ( size_t i = 0; i < videoSrc.getFrameCount(); i++ ) {
    convertYUV420ToYUV444( videoSrc[i], videoDst[i], nbyte, filter );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::convertYUV420ToYUV444( PCCImage<T, 3>& imageSrc,
                                                          PCCImage<T, 3>& imageDst,
                                                          size_t          nbyte,
                                                          size_t          filter ) {
  int width  = (int)imageSrc.getWidth();
  int height = (int)imageSrc.getHeight();
  imageDst.resize( width, height, pcc::PCCCOLORFORMAT::YUV444 );
  size_t             widthChroma  = width / 2;
  size_t             heightChroma = height / 2;
  std::vector<float> YUV444[3], YUV420[3];
  YUVtoFloatYUV( imageSrc[0], YUV420[0], 0, nbyte );
  YUVtoFloatYUV( imageSrc[1], YUV420[1], 1, nbyte );
  YUVtoFloatYUV( imageSrc[2], YUV420[2], 1, nbyte );
  upsampling( YUV420[1], YUV444[1], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
  upsampling( YUV420[2], YUV444[2], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
  floatYUVToYUV( YUV420[0], imageDst[0], 0, 2 );
  floatYUVToYUV( YUV444[1], imageDst[1], 1, 2 );
  floatYUVToYUV( YUV444[2], imageDst[2], 1, 2 );
}

template <typename T>
void PCCInternalColorConverter<T>::convertYUV420ToRGB444( PCCVideo<T, 3>& videoSrc,
                                                          PCCVideo<T, 3>& videoDst,
                                                          size_t          nbyte,
                                                          size_t          filter ) {
  videoDst.resize( videoSrc.getFrameCount() );
  for ( size_t i = 0; i < videoSrc.getFrameCount(); i++ ) {
    convertYUV420ToRGB444( videoSrc[i], videoDst[i], nbyte, filter );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::convertYUV420ToRGB444( PCCImage<T, 3>& imageSrc,
                                                          PCCImage<T, 3>& imageDst,
                                                          size_t          nbyte,
                                                          size_t          filter ) {
  printf( "convertYUV420ToRGB444 \n" );
  fflush( stdout );
  int width  = (int)imageSrc.getWidth();
  int height = (int)imageSrc.getHeight();
  imageDst.resize( width, height, pcc::PCCCOLORFORMAT::RGB444 );
  size_t             widthChroma  = width / 2;
  size_t             heightChroma = height / 2;
  std::vector<float> YUV444[3], YUV420[3], RGB444[3];
  YUVtoFloatYUV( imageSrc[0], YUV420[0], 0, nbyte );
  YUVtoFloatYUV( imageSrc[1], YUV420[1], 1, nbyte );
  YUVtoFloatYUV( imageSrc[2], YUV420[2], 1, nbyte );
  upsampling( YUV420[1], YUV444[1], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
  upsampling( YUV420[2], YUV444[2], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
  convertYUVToRGB( YUV420[0], YUV444[1], YUV444[2], RGB444[0], RGB444[1], RGB444[2] );
  floatRGBToRGB( RGB444[0], imageDst[0], nbyte );
  floatRGBToRGB( RGB444[1], imageDst[1], nbyte );
  floatRGBToRGB( RGB444[2], imageDst[2], nbyte );
}

template <typename T>
void PCCInternalColorConverter<T>::convertYUV444ToRGB444( PCCVideo<T, 3>& videoSrc,
                                                          PCCVideo<T, 3>& videoDst,
                                                          size_t          nbyte,
                                                          size_t          filter ) {
  videoDst.resize( videoSrc.getFrameCount() );
  for ( size_t i = 0; i < videoSrc.getFrameCount(); i++ ) {
    convertYUV444ToRGB444( videoSrc[i], videoDst[i], nbyte, filter );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::convertYUV444ToRGB444( PCCImage<T, 3>& imageSrc,
                                                          PCCImage<T, 3>& imageDst,
                                                          size_t          nbyte,
                                                          size_t          filter ) {
  printf( "convertYUV444ToRGB444 \n" );
  fflush( stdout );
  int width  = (int)imageSrc.getWidth();
  int height = (int)imageSrc.getHeight();
  imageDst.resize( width, height, pcc::PCCCOLORFORMAT::RGB444 );
  std::vector<float> YUV444[3], RGB444[3];
  YUVtoFloatYUV( imageSrc[0], YUV444[0], 0, nbyte );
  YUVtoFloatYUV( imageSrc[1], YUV444[1], 1, nbyte );
  YUVtoFloatYUV( imageSrc[2], YUV444[2], 1, nbyte );
  convertYUVToRGB( YUV444[0], YUV444[1], YUV444[2], RGB444[0], RGB444[1], RGB444[2] );
  floatRGBToRGB( RGB444[0], imageDst[0], nbyte );
  floatRGBToRGB( RGB444[1], imageDst[1], nbyte );
  floatRGBToRGB( RGB444[2], imageDst[2], nbyte );
}

template <typename T>
void PCCInternalColorConverter<T>::RGBtoFloatRGB( const std::vector<T>& src,
                                                  std::vector<float>&   dst,
                                                  const size_t          nbyte ) const {
  size_t count = src.size();
  dst.resize( count );
  float offset = nbyte == 1 ? 255.f : 1023.f;
  for ( size_t i = 0; i < count; i++ ) { dst[i] = (float)src[i] / offset; }
}

template <typename T>
void PCCInternalColorConverter<T>::convertRGBToYUV( const std::vector<float>& R,
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

template <typename T>
void PCCInternalColorConverter<T>::floatYUVToYUV( const std::vector<float>& src,
                                                  std::vector<T>&           dst,
                                                  const bool                chroma,
                                                  const size_t              nbyte ) const {
  size_t count = src.size();
  dst.resize( count );
  double offset = chroma ? nbyte == 1 ? 128. : 32768. : 0;
  double scale  = nbyte == 1 ? 255. : 65535.;
  for ( size_t i = 0; i < count; i++ ) {
    dst[i] = static_cast<T>( fClip( std::round( (float)( scale * (double)src[i] + offset ) ), 0.f, (float)scale ) );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::YUVtoFloatYUV( const std::vector<T>& src,
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

template <typename T>
void PCCInternalColorConverter<T>::convertYUVToRGB( const std::vector<float>& Y,
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

template <typename T>
void PCCInternalColorConverter<T>::floatRGBToRGB( const std::vector<float>& src,
                                                  std::vector<T>&           dst,
                                                  const size_t              nbyte ) const {
  size_t count = src.size();
  dst.resize( count );
  float scale = nbyte == 1 ? 255.f : 1023.f;
  for ( size_t i = 0; i < count; i++ ) {
    dst[i] = static_cast<T>( clamp( (T)std::round( scale * src[i] ), (T)0, (T)scale ) );
  }
}

template <typename T>
void PCCInternalColorConverter<T>::downsampling( const std::vector<float>& chroma_in,
                                                 std::vector<float>&       chroma_out,
                                                 const int                 widthIn,
                                                 const int                 heightIn,
                                                 const int                 maxValue,
                                                 const size_t              filter ) const {
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

template <typename T>
void PCCInternalColorConverter<T>::upsampling( const std::vector<float>& chromaIn,
                                               std::vector<float>&       chromaOut,
                                               const int                 widthIn,
                                               const int                 heightIn,
                                               const int                 maxValue,
                                               const size_t              filter ) const {
  int                widthOut = widthIn * 2, heightOut = heightIn * 2;
  std::vector<float> temp;
  chromaOut.resize( widthOut * heightOut );
  temp.resize( widthIn * heightOut );
  for ( int i = 0; i < heightIn; i++ ) {
    for ( int j = 0; j < widthIn; j++ ) {
      temp[( 2 * i ) * widthIn + j] =
          upsamplingVertical0( g_filter420to444[filter], chromaIn, widthIn, heightIn, i + 0, j );
      temp[( 2 * i + 1 ) * widthIn + j] =
          upsamplingVertical1( g_filter420to444[filter], chromaIn, widthIn, heightIn, i + 1, j );
    }
  }
  for ( int i = 0; i < heightOut; i++ ) {
    for ( int j = 0; j < widthIn; j++ ) {
      chromaOut[i * widthOut + j * 2] =
          upsamplingHorizontal0( g_filter420to444[filter], temp, widthIn, heightOut, i, j + 0 );
      chromaOut[i * widthOut + j * 2 + 1] =
          upsamplingHorizontal1( g_filter420to444[filter], temp, widthIn, heightOut, i, j + 1 );
    }
  }
}

template <typename T>
void PCCInternalColorConverter<T>::upsample( PCCVideo<T, 3>& video, size_t rate, size_t nbyte, size_t filter ) {
  for ( auto& image : video ) { upsample( image, rate, nbyte, filter ); }
}

template <typename T>
void PCCInternalColorConverter<T>::upsample( PCCImage<T, 3>& image, size_t rate, size_t nbyte, size_t filter ) {
  for ( size_t i = rate; i > 1; i /= 2 ) {
    int                width        = (int)image.getWidth();
    int                height       = (int)image.getHeight();
    int                widthChroma  = image.getColorFormat() == YUV420 ? width / 2 : width;
    int                heightChroma = image.getColorFormat() == YUV420 ? height / 2 : height;
    std::vector<float> src[3], up[3];
    YUVtoFloatYUV( image[0], src[0], 0, nbyte );
    YUVtoFloatYUV( image[1], src[1], 1, nbyte );
    YUVtoFloatYUV( image[2], src[2], 1, nbyte );
    upsampling( src[0], up[0], width, height, nbyte == 1 ? 255 : 1023, filter );
    upsampling( src[1], up[1], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
    upsampling( src[2], up[2], widthChroma, heightChroma, nbyte == 1 ? 255 : 1023, filter );
    image.resize( width * 2, height * 2, image.getColorFormat() );
    floatYUVToYUV( up[0], image[0], 0, nbyte );
    floatYUVToYUV( up[1], image[1], 1, nbyte );
    floatYUVToYUV( up[2], image[2], 1, nbyte );
  }
}

template class pcc::PCCInternalColorConverter<uint8_t>;
template class pcc::PCCInternalColorConverter<uint16_t>;
