General options
---------------

The next tables shows the parameters of the encoder, decoder and metrics programs.

### Encoder parameters

\begin{longtable}{p{7cm}p{7cm}}
\hline
{\bf Parameter=Value} & {\bf Usage} \\ \hline
--help=0                                      & This help text                             \\ \hline

{\bf Global }                                 &                                            \\ \hline\hline
-c,--config=...                               & Configuration file name                    \\ \hline
--configurationFolder=""                      & Folder where the configuration files       \\ 
                                              & are stored, use for cfg relative           \\ 
                                              & paths.                                     \\ \hline
--uncompressedDataFolder=""                   & Folder where the uncompress input data     \\ 
                                              & are stored, use for cfg relative           \\ 
                                              & paths.                                     \\ \hline
--uncompressedDataPath=""                     & Input pointcloud to encode.                \\ 
                                              & Multi-frame sequences may be               \\ 
                                              & represented by \%04i                       \\ \hline
--compressedStreamPath=""                     & Output                                     \\ 
                                              & compressed bitstream                       \\ \hline
--reconstructedDataPath=""                    & Output decoded pointcloud. Multi-frame     \\ 
                                              & sequences may be represented by \%04i      \\ \hline
--startFrameNumber=0                          & First frame number in sequence to          \\ 
                                              & encode/decode                              \\ \hline
--frameCount=300                              & Number of frames to encode                 \\ \hline
--groupOfFramesSize=32                        & Random access period                       \\ \hline
--colorTransform=1                            & The colour transform to be applied:        \\ 
                                              &   0: none                                  \\ 
                                              &   1: RGB to YCbCr (Rec.709)                \\ \hline
--colorSpaceConversionPath=""                 & Path to the HDRConvert. If unset, an       \\ 
                                              & internal color space conversion is         \\ 
                                              & used                                       \\ \hline
--colorSpaceConversionConfig=""               & HDRConvert configuration file used for     \\ 
                                              & RGB444 to YUV420 conversion                \\ \hline
--inverseColorSpaceConversionConfig=""        & HDRConvert configuration file used for     \\ 
                                              & YUV420 to RGB444 conversion                \\ \hline
--videoEncoderPath=""                         & HM video encoder executable                \\ \hline
--videoEncoderOccupancyMapPath=""             & HM lossless video encoder executable       \\ 
                                              & for occupancy map                          \\ \hline
--nbThread=1                                  & Number of thread used for parallel         \\ 
                                              & processing                                 \\ \hline
--keepIntermediateFiles=0                     & Keep intermediate files: RGB, YUV and      \\ 
                                              & bin                                        \\ \hline\hline

{\bf Encoder }                                &                                            \\ \hline\hline
--nnNormalEstimation=16                       & Number of points used for normal           \\ 
                                              & estimation                                 \\ \hline
--maxNNCountRefineSegmentation=256            & Number of nearest neighbors used           \\ 
                                              & during segmentation refinement             \\ \hline
--iterationCountRefineSegmentation=100        & Number of iterations performed during      \\ 
                                              & segmentation refinement                    \\ \hline
--occupancyResolution=16                      & Resolution T of the occupancy map          \\ \hline
--minPointCountPerCCPatch                     &  Minimum number of points for a            \\ 
     \ \ \ \ \ \ Segmentation=16              & connected component to be retained as      \\ 
                                              & a patch                                    \\ \hline
--maxNNCountPatchSegmentation=16              & Number of nearest neighbors used           \\ 
                                              & during connected components                \\ \hline
                                              & extraction                                 \\ \hline
--surfaceThickness=4                          & Surface thickness                          \\ \hline
--maxAllowedDepth=255                         & Maximum depth per patch                    \\ \hline
--maxAllowedDist2MissedPointsDetection=9      & Maximum distance for a point to be         \\ 
                                              & ignored during missed point detection      \\ \hline
--maxAllowedDist2MissedPointsSelection=1      & Maximum distance for a point to be         \\ 
                                              & ignored during  missed points              \\ 
                                              & selection                                  \\ \hline
--lambdaRefineSegmentation=3                  & Controls the smoothness of the patch       \\ 
                                              & boundaries  during segmentation            \\ 
                                              & refinement                                 \\ \hline
--minimumImageWidth=1280                      & Minimum width of packed patch frame        \\ \hline
--minimumImageHeight=1280                     & Minimum height of packed patch frame       \\ \hline
--maxCandidateCount=4                         & Maximum nuber of candidates in list L      \\ \hline
--occupancyPrecision=4                        & Occupancy map B0 precision                 \\ \hline
--occupancyMapVideoEncoderConfig=""           & Occupancy map encoder config file          \\ \hline
--occupancyMapQP=8                            & QP for compression of occupancy map        \\ 
                                              & video                                      \\ \hline
--useOccupancyMapVideo=1                      & compress occupancy map with video          \\ 
                                              & codec                                      \\ \hline
--neighborCountSmoothing=64                   & todo(kmammou)                              \\ \hline
--radius2Smoothing=64                         & todo(kmammou)                              \\ \hline
--radius2BoundaryDetection=64                 & todo(kmammou)                              \\ \hline
--thresholdSmoothing=64                       & todo(kmammou)                              \\ \hline
--gridSmoothing=1                             & Enable grid smoothing                      \\ \hline
--thresholdColorSmoothing=10                  & Threshold of color smoothing               \\ \hline
--thresholdLocalEntropy=4.5                   & Threshold of local entropy                 \\ \hline
--radius2ColorSmoothing=64                    & Redius of color smoothing                  \\ \hline
--neighborCountColorSmoothing=64              & Neighbor count for color smoothing         \\ \hline
--flagColorSmoothing=0                        & Enable color smoothing                     \\ \hline
--thresholdColorPreSmoothing=10               & Threshold of color pre-smoothing           \\ \hline
--thresholdColorPreSmoothing                  & Threshold of color pre-smoothing local     \\ 
     \ \ \ \ \ \ LocalEntropy=4.5             & entropy                                    \\ \hline
--radius2ColorPreSmoothing=64                 & Redius of color pre-smoothing              \\ \hline
--neighborCountColorPreSmoothing=64           & Neighbor count for color                   \\ 
                                              & pre-smoothing                              \\ \hline
--flagColorPreSmoothing=1                     & Enable color pre-smoothing                 \\ \hline
--bestColorSearchRange=0                      & todo(kmammou)                              \\ \hline
--geometryQP=28                               & QP for compression of geometry video       \\ \hline
--textureQP=43                                & QP for compression of texture video        \\ \hline
--geometryConfig=""                           & HM configuration file for geometry         \\ 
                                              & compression                                \\ \hline
--geometryD0Config=""                         & HM configuration file for geometry D0      \\ 
                                              & compression                                \\ \hline
--geometryD1Config=""                         & HM configuration file for geometry D1      \\ 
                                              & compression                                \\ \hline
--textureConfig=""                            & HM configuration file for texture          \\ 
                                              & compression                                \\ \hline
--losslessGeo=0                               & Enable lossless encoding of geometry       \\ \hline
--losslessTexture=0                           & Enable lossless encoding of texture        \\ \hline
--noAttributes=0                              & Disable encoding of attributes             \\ \hline
--losslessGeo444=0                            & Use 4444 format for lossless geometry      \\ \hline
--useMissedPointsSeparateVideo=0              & compress missed point with video           \\ 
                                              & codec                                      \\ \hline
--geometryMPConfig=""                         & HM configuration file for missed           \\ 
                                              & points geometry compression                \\ \hline
--textureMPConfig=""                          & HM configuration file for missed           \\ 
                                              & points texture compression                 \\ \hline
--absoluteD1=1                                & Absolute D1                                \\ \hline
--constrainedPack=1                           & Temporally consistent patch packing        \\ \hline
--binArithCoding=1                            & Binary arithmetic coding                   \\ \hline
--testLevelOfDetailSignaling=0                & Test the patch resolution signaling        \\ 
                                              & with pseudo-random values                  \\ \hline
--groupDilation=1                             & Group Dilation                             \\ \hline
--textureDilationOffLossless=1                & Group Dilation                             \\ \hline
--enhancedDeltaDepthCode=0                    & Use enhanced-delta-depth code              \\ \hline
--patchColorSubsampling=0                     & Enable per patch color sub-sampling        \\ \hline
--deltaCoding=1                               & Delta meta-data coding                     \\ \hline
--projectionMode=0                            & projectionMode 0:min, 1:max, 2:adaptive    \\ 
                                              & frame and patch, 3:adaptive                \\ 
                                              & patch (all frames)                         \\ \hline
--oneLayerMode=0                              & Use one layer mode                         \\ \hline
--singleLayerPixelInterleaving=0              & Use single layer pixel interleaving        \\ \hline
--removeDuplicatePoints=1                     & Remove duplicate points                    \\ \hline
--sixDirectionMode=1                          & Use Six Direction Projection mode          \\ \hline
--surfaceSeparation=0                         & surface separation                         \\ \hline    
--packingStrategy=1                           & Patches packing strategy                   \\ 
                                              & (0: anchor packing, 1(default): flexible   \\ 
                                              & packing, 2: tetris packing)                \\ \hline
--useEightOrientations=0                      & Allow either 2 orientations (0(default):   \\ 
                                              & NULL AND SWAP), or 8 orientation (1)       \\ \hline
--safeGuardDistance=0                         & Number of empty blocks that must exist     \\ 
                                              & between the patches (default=1)            \\ \hline
--textureBGFill=1                             & Selects the background filling operation   \\ 
                                              & for texture only (0: patch-edge extension, \\ 
                                              & 1(default): smoothed push-pull algorithm), \\ 
                                              & 2: harmonic background filling             \\ \hline
--lossyMissedPointsPatch=0                    & Lossy missed points patch(0: no lossy      \\ 
                                              & missed points patch, 1: enable lossy       \\ 
                                              & missed points  patch (default=0)           \\ \hline
--minNormSumOfInvDist4MP                      & Minimum normalized sum of inverse distance \\ 
     \ \ \ \ \ \ Selection=0.35               & for missed points selection: double value  \\ 
                                              & between 0.0 and 1.0 (default=0.35)         \\ \hline
--lossyMppGeoQP=4                             & QP value for geometry in lossy missed      \\ 
                                              & points patch (default=4)                   \\ \hline
--globalPatchAllocation=0                     & Global temporally consistent patch         \\ 
                                              & allocation.                                \\ 
                                              & (0: anchor's packing method(default),      \\ 
                                              & 1: gpa algorithm)                          \\ \hline
--apply3dMotionCompensation=1                 & Use auxilliary information for 3d motion   \\ 
                                              & compensation.(0: conventional video coding,\\ 
                                              & 1: 3D motion compensated)                  \\ \hline
--geometry3dCoordinatesBitdepth=10            & Bit depth of geomtery 3D coordinates       \\ \hline
--geometryNominal2dBitdepth=8                 & Bit depth of geometry 2D                   \\ \hline
--nbPlrmMode=0                                & Number of PLR mode                         \\ \hline
--patchSize=0                                 & Size of Patch for PLR                      \\ \hline
--enhancedProjectionPlane=1                   & Use enhanced Projection Plane              \\ 
                                              & (0: OFF, 1: ON)                            \\ \hline
--minWeightEPP=0.6                            & Minimum value                              \\ \hline
--additionalProjectionPlaneMode=0             & additiona Projection Plane Mode 0:none     \\ 
                                              & 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis      \\
                                              & 5:apply to portion                         \\ \hline
--partialAdditionalProjectionPlane=0          & The value determines the partial point     \\ 
                                              & cloud. It's available with only            \\ 
                                              & additionalProjectionPlaneMode(5)           \\ \hline\hline

{\bf Metrics }                                &                                            \\ \hline\hline
--computeChecksum=1                           & Compute checksum                           \\ \hline
--computeMetrics=1                            & Compute metrics                            \\ \hline
--normalDataPath=""                           & Input pointcloud to encode.                \\ 
                                              & Multi-frame sequences may be               \\ 
                                              & represented by \%04i                       \\ \hline
--resolution=1023                             & Specify the intrinsic resolution           \\ \hline
--dropdups=2                                  & 0(detect), 1(drop), 2(average)             \\ 
                                              & subsequent points with same                \\ 
                                              & coordinates                                \\ \hline
--neighborsProc=1                             & 0(undefined), 1(average), 2(weighted       \\ 
                                              & average), 3(min), 4(max) neighbors         \\ 
                                              & with same geometric distance               \\ \hline
\end{longtable}

       
### Decoder parameters       

\begin{longtable}{p{7cm}p{7cm}}
\hline
{\bf Parameter=Value} & {\bf Usage} \\ \hline
--help=0                               & This help text                             \\ \hline\hline
{\bf Global }                          &                                            \\ \hline\hline
-c,--config=...                        & Configuration file name                    \\ \hline
--compressedStreamPath=""              & Input                                      \\ 
                                       & compressed bitstream                       \\ \hline
--reconstructedDataPath=""             & Output decoded pointcloud. Multi-frame     \\ 
                                       & sequences may be represented by \%04i      \\ \hline
--startFrameNumber=0                   & Fist frame number in sequence to           \\ 
                                       & encode/decode                              \\ \hline
--colorTransform=1                     & The colour transform to be applied:        \\ 
                                       &   0: none                                  \\ 
                                       &   1: RGB to YCbCr (Rec.709)                \\ \hline
--colorSpaceConversionPath=""          & Path to the HDRConvert. If unset, an       \\ 
                                       & internal color space conversion is         \\ 
                                       & used                                       \\ \hline
--inverseColorSpaceConversionConfig="" & HDRConvert configuration file used for     \\ 
                                       & YUV420 to RGB444 conversion                \\ 
--videoDecoderPath=""                  & HM video decoder executable                \\ \hline
--videoDecoderOccupancyMapPath=""      & HM lossless video decoder executable       \\ 
                                       & for occupancy map                          \\ \hline
--nbThread=1                           & Number of thread used for parallel         \\ 
                                       & processing                                 \\ \hline
--keepIntermediateFiles=0              & Keep intermediate files: RGB, YUV and      \\ 
                                       & bin                                        \\ \hline\hline
{\bf Metrics }                         &                                            \\ \hline\hline
--testLevelOfDetailSignaling=0         & Disable patch sampling resolution          \\ 
                                       & scaling; use in conjunction with same      \\ 
                                       & parameter in encoder                       \\ \hline
--patchColorSubsampling=0              & Enable per-patch color up-sampling         \\ \hline\hline
{\bf Metrics }                         &                                            \\ \hline\hline
--computeChecksum=1                    & Compute checksum                           \\ \hline
--computeMetrics=1                     & Compute metrics                            \\ \hline
--uncompressedDataFolder=""            & Folder where the uncompress input data     \\ 
                                       & are stored, use for cfg relative           \\ 
                                       & paths.                                     \\ \hline
--startFrameNumber=0                   & Fist frame number in sequence to           \\ 
                                       & encode/decode                              \\ \hline
--frameCount=0                         & Number of frames to encode                 \\ \hline
--groupOfFramesSize=32                 & Random access period                       \\ \hline
--uncompressedDataPath=""              & Input pointcloud to encode.                \\ 
                                       & Multi-frame sequences may be               \\ 
                                       & represented by \%04i                       \\ \hline
--reconstructedDataPath=""             & Output decoded pointcloud. Multi-frame     \\ 
                                       & sequences may be represented by \%04i      \\ \hline
--normalDataPath=""                    & Input pointcloud to encode.                \\ 
                                       & Multi-frame sequences may be               \\ 
                                       & represented by \%04i                       \\ \hline
--resolution=1023                      & Specify the intrinsic resolution           \\ \hline
--dropdups=2                           & 0(detect), 1(drop), 2(average)             \\ 
                                       & subsequent points with same                \\ 
                                       & coordinates                                \\ \hline
--neighborsProc=1                      & 0(undefined), 1(average), 2(weighted       \\ 
                                       & average), 3(min), 4(max) neighbors         \\ 
                                       & with same geometric distance               \\ \hline
--nbThread=0                           & Number of thread used for parallel         \\ 
                                       & processing                                 \\ \hline
--minimumImageHeight=0                 & Ignore parameter                           \\ \hline
--flagColorPreSmoothing=0              & Ignore parameter                           \\ \hline
--surfaceSeparation=0                  & Ignore parameter                           \\ \hline
\end{longtable}                      


### Metrics parameters       

\begin{longtable}{p{7cm}p{7cm}}
\hline
{\bf Parameter=Value} & {\bf Usage} \\ \hline
--help=0                               & This help text                             \\ \hline 
--computeChecksum=1                    & Compute checksum                           \\ \hline 
--computeMetrics=1                     & Compute metrics                            \\ \hline 
--startFrameNumber=0                   & Fist frame number in sequence to           \\ \hline 
                                       & encode/decode                              \\ \hline 
--frameCount=0                         & Number of frames to encode                 \\ \hline 
--uncompressedDataPath=""              & Input pointcloud to encode. Multi-frame    \\ \hline 
                                       & sequences may be represented by \%04i      \\ \hline 
--reconstructedDataPath=""             & Output decoded pointcloud. Multi-frame     \\ \hline 
                                       & sequences may be represented by \%04i      \\ \hline 
--normalDataPath=""                    & Input pointcloud to encode. Multi-frame    \\ \hline 
                                       & sequences may be represented by \%04i      \\ \hline 
--resolution=1023                      & Specify the intrinsic resolution           \\ \hline 
--dropdups=2                           & 0(detect), 1(drop), 2(average) subsequent  \\ \hline 
                                       & points with same coordinates               \\ \hline 
--neighborsProc=1                      & 0(undefined), 1(average), 2(weighted       \\ \hline 
                                       & average), 3(min), 4(max) neighbors with    \\ \hline 
                                       & same geometric distance                    \\ \hline 
--nbThread=0                           & Number of thread used for parallel         \\ \hline 
                                       & processing                                 \\ \hline 
--minimumImageHeight=0                 & Ignore parameter                           \\ \hline 
--flagColorPreSmoothing=0              & Ignore parameter                           \\ \hline 
--surfaceSeparation=0                  & Ignore parameter                           \\ \hline 
\end{longtable}

