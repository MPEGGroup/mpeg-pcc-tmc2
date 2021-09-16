General options
---------------

The next tables shows the parameters of the encoder, decoder and metrics programs.

### Encoder parameters

\begin{longtable}{p{6cm}p{8cm}}
\hline
{\bf Parameter=Value}             & {\bf Usage}                                         \\ \hline
help                              &  This help text                                     \\ \hline\hline

{\bf Global }                     &                                                     \\ \hline\hline
c|config                          &  Configuration file name                            \\ \hline
configurationFolder               &  Folder where the configuration files are           \\ 
                                  &  stored,use for cfg relative paths.                 \\ \hline
uncompressedDataFolder            &  Folder where the uncompress input data are         \\ 
                                  &  stored, use for cfg relative paths.                \\ \hline
uncompressedDataPath              &  Input pointcloud to encode. Multi-frame            \\ 
                                  &  sequences may be represented by \%04i               \\ \hline
compressedStreamPath              &  Output(encoder)/Input(decoder) compressed          \\ 
                                  &  bitstream                                          \\ \hline
reconstructedDataPath             &  Output decoded pointcloud. Multi-frame             \\ 
                                  &  sequences may be represented by \%04i               \\ \hline
startFrameNumber                  &  First frame number in sequence to                  \\ 
                                  &  encode/decode                                      \\ \hline
frameCount                        &  Number of frames to encode                         \\ \hline
groupOfFramesSize                 &  Random access period                               \\ \hline
colorTransform                    &  The colour transform to be applied: 0: none        \\ 
                                  &  1: RGB to YCbCr (Rec.709)                          \\ \hline
colorSpaceConversionPath          &  Path to the HDRConvert. If unset, an internal      \\ 
                                  &  color space conversion is used                     \\ \hline
colorSpaceConversionConfig        &  HDRConvert configuration file used for RGB444      \\ 
                                  &  to YUV420 conversion                               \\ \hline
inverseColorSpaceConversion       &  HDRConvert configuration file used for YUV420      \\ 
  \ \ \ \ \ \ Config              &  to RGB444 conversion                               \\ \hline             
videoEncoderPath                  &  HM video encoder executable                        \\ \hline
videoEncoderAuxPath               &  HM video encoder executable                        \\ \hline
videoEncoderOccupancyMapPath      &  HM lossless video encoder executable for           \\ 
                                  &  occupancy map                                      \\ \hline
nbThread                          &  Number of thread used for parallel processing      \\ \hline
keepIntermediateFiles             &  Keep intermediate files: RGB, YUV and bin          \\ \hline \hline

{\bf Encoder }                    &                                                     \\ \hline\hline
nnNormalEstimation                &  Number of points used for normal estimation        \\ \hline
gridBasedRefineSegmentation       &  Use grid-based approach for segmentation           \\ 
                                  &  refinement                                         \\ \hline
maxNNCountRefineSegmentation      &  Number of nearest neighbors used during            \\ 
                                  &  segmentation refinement                            \\ \hline
iterationCountRefine              &  Number of iterations performed during              \\ 
  \ \ \ \ \ \ Segmentation        &  segmentation refinement                            \\ \hline
voxelDimensionRefine              &  Voxel dimension for segmentation refinement        \\ 
  \ \ \ \ \ \ Segmentation        &  (must be a power of 2)                             \\ \hline
searchRadiusRefineSegmentation    &  Search radius for segmentation refinement          \\ \hline
occupancyResolution               &  Resolution of packing block(a block contain        \\ 
                                  &  only one patch)                                    \\ \hline
enablePatchSplitting              &  Enable patch splitting                             \\ \hline
maxPatchSize                      &  Maximum patch size for segmentation                \\ \hline
log2QuantizerSizeX                &  log2 of Quantization step for patch size X:        \\ 
                                  &  0. pixel precision 4.16 as before                  \\ \hline
log2QuantizerSizeY                &  log2 of Quantization step for patch size Y:        \\ 
                                  &  0. pixel precision 4.16 as before                  \\ \hline
minPointCountPerCCPatch           &  Minimum number of points for a connected           \\ 
  \ \ \ \ \ \ Segmentation        &  component to be retained as a patch                \\ \hline
maxNNCountPatchSegmentation       &  Number of nearest neighbors used during            \\ 
                                  &  connected components extraction                    \\ \hline
surfaceThickness                  &  Surface thickness                                  \\ \hline
depthQuantizationStep             &  minimum level for patches                          \\ \hline
maxAllowedDist2RawPoints          &  Maximum distance for a point to be ignored         \\ 
  \ \ \ \ \ \ Detection           &  during raw points detection                        \\ \hline
maxAllowedDist2RawPoints          &  Maximum distance for a point to be ignored         \\ 
  \ \ \ \ \ \ Selection           &  during  raw points  selection                      \\ \hline
lambdaRefineSegmentation          &  Controls the smoothness of the patch               \\ 
                                  &  boundaries  during segmentation  refinement        \\ \hline
minimumImageWidth                 &  Minimum width of packed patch frame                \\ \hline
minimumImageHeight                &  Minimum height of packed patch frame               \\ \hline
maxCandidateCount                 &  Maximum nuber of candidates in list L              \\ \hline
occupancyPrecision                &  Occupancy map B0 precision                         \\ \hline
occupancyMapConfig    &  Occupancy map encoder config file                  \\ \hline
occupancyMapQP                    &  QP for compression of occupancy map video          \\ \hline
enhancedOccupancyMapCode          &  Use enhanced-delta-depth code                      \\ \hline
EOMFixBitCount                    &  enhanced occupancy map fixed bit count             \\ \hline
occupancyMapRefinement            &  Use occupancy map refinement                       \\ \hline
postprocessSmoothingFilterType    &  Exclude geometry smoothing from attribute transfer \\ \hline
flagGeometrySmoothing             &  Enable geometry smoothing                          \\ \hline
neighborCountSmoothing            &  Neighbor count smoothing                           \\ \hline
radius2Smoothing                  &  Radius to smoothing                                \\ \hline
radius2BoundaryDetection          &  Radius to boundary detection                       \\ \hline
thresholdSmoothing                &  Threshold smoothing                                \\ \hline
patchExpansion                    &  Use occupancy map refinement                       \\ \hline
gridSmoothing                     &  Enable grid smoothing                              \\ \hline
gridSize                          &  grid size for the smoothing                        \\ \hline
thresholdColorSmoothing           &  Threshold of color smoothing                       \\ \hline
cgridSize                         &  grid size for the color smoothing                  \\ \hline
thresholdColorDifference          &  Threshold of color difference between cells        \\ \hline
thresholdColorVariation           &  Threshold of color variation in cells              \\ \hline
flagColorSmoothing                &  Enable color smoothing                             \\ \hline
thresholdColorPreSmoothing        &  Threshold of color pre-smoothing                   \\ \hline
thresholdColorPreSmoothing        &  Threshold of color pre-smoothing local             \\ 
     LocalEntropy                 &  entropy                                            \\ \hline
radius2ColorPreSmoothing          &  Redius of color pre-smoothing                      \\ \hline
neighborCountColorPreSmoothing    &  Neighbor count for color pre-smoothing             \\ \hline
flagColorPreSmoothing             &  Enable color pre-smoothing                         \\ \hline
bestColorSearchRange              &  Best color search range                            \\ \hline
numNeighborsColorTransferFwd      &  Number of neighbors creating Fwd list              \\ \hline
numNeighborsColorTransferBwd      &  Number of neighbors creating Bwd list              \\ \hline
useDistWeightedAverageFwd         &  Distance weighted average for Fwd list             \\ \hline
useDistWeightedAverageBwd         &  Distance weighted average for Bwd list             \\ \hline
skipAvgIfIdenticalSourcePoint     &  Skip avgeraging if target is identical to a        \\ 
   \ \ \ \ \ \ PresentFwd         &  Fwd point                                          \\ \hline
skipAvgIfIdenticalSourcePoint     &  Skip avgeraging if target is identical to a        \\ 
   \ \ \ \ \ \ PresentBwd         &  Bwd point                                          \\ \hline
distOffsetFwd                     &  Distance offset to avoid infinite weight           \\ \hline
distOffsetBwd                     &  Distance offset to avoid infinite weight           \\ \hline
maxGeometryDist2Fwd               &  Maximum allowed distance for a Fwd point           \\ \hline
maxGeometryDist2Bwd               &  Maximum allowed distance for a Bwd point           \\ \hline
maxColorDist2Fwd                  &  Maximum allowed pari-wise color distance for       \\ 
                                  &  Fwd list                                           \\ \hline
maxColorDist2Bwd                  &  Maximum allowed pari-wise color distance for       \\ 
                                  &  Bwd list                                           \\ \hline
excludeColorOutlier               &  Exclude color outliers from the NN set             \\ \hline
thresholdColorOutlierDist         &  Threshold of color distance to exclude             \\ 
                                  &  outliers from the NN set                           \\ \hline									   
geometryQP                        &  QP for compression of geometry video               \\ \hline
attributeQP                         &  QP for compression of attribute video                \\ \hline
geometryConfig                    &  HM configuration file for geometry                 \\ 
                                  &  compression                                        \\ \hline
geometry0Config                  &  HM configuration file for geometry D0              \\ 
                                  &  compression                                        \\ \hline
geometry1Config                  &  HM configuration file for geometry D1              \\ 
                                  &  compression                                        \\ \hline
attributeConfig                     &  HM configuration file for attribute                  \\ 
                                  &  compression                                        \\ \hline
attribute0Config                   &  HM configuration file for attribute D0               \\ 
                                  &  compression                                        \\ \hline
attribute1Config                   &  HM configuration file for attribute D1               \\ 
                                  &  compression                                        \\ \hline
rawPointsPatch                    &  Enable raw points patch                            \\ \hline     
noAttributes                      &  Disable encoding of attributes                     \\ \hline
attributeVideo444                 &  Use 444 format for attribute videos                \\ \hline
useRawPointsSeparateVideo         &  Compress raw points with video codec               \\ \hline
attributeRawSeparateVideoWidth      &  Width of the MP's attribute in separate video        \\ \hline
geometryMPConfig                  &  HM configuration file for raw points geometry      \\ 
                                  &  compression                                        \\ \hline
attributeMPConfig                   &  HM configuration file for raw points attribute       \\ 
                                  &  compression                                        \\ \hline
absoluteD1                        &  Absolute D1                                        \\ \hline
absoluteT1                        &  Absolute T1                                        \\ \hline
multipleStreams                   &  number of video(geometry and attribute)            \\ 
                                  &  streams                                            \\ \hline
qpT1                              &  qp adjustment for T1 0, +3, -3...                  \\ \hline
qpD1                              &  qp adjustment for D1 : 0, +3, -3...                \\ \hline
constrainedPack                   &  Temporally consistent patch packing                \\ \hline
levelOfDetailX                    &  levelOfDetail : X axis in 2D space (should be      \\ 
                                  &  greater than 1)                                    \\ \hline
levelOfDetailY                    &  levelOfDetail : Y axis in 2D space (should be      \\ 
                                  &  greater than 1)                                    \\ \hline
groupDilation                     &  Group Dilation                                     \\ \hline
offsetLossyOM                     &  Value to be assigned to non-zero occupancy         \\
                                  &  map positions                                      \\ \hline
thresholdLossyOM                  &  Threshold for converting non-binary occupancy      \\ 
                                  &  map to binary                                      \\ \hline
prefilterLossyOM                  &  Selects whether the occupany map is prefiltered    \\ 
                                  &  before lossy compression (default=false)           \\ \hline
patchColorSubsampling             &  Enable per patch color sub-sampling                \\ \hline
maxNumRefAtalsList                &  maximum Number of Reference Patch list,            \\ 
                                  &  default: 1                                         \\ \hline
maxNumRefAtlasFrame               &  maximum Number of Reference Atlas Frame per        \\ 
                                  &  list, default: 1                                   \\ \hline
pointLocalReconstruction          &  Use point local reconstruction                     \\ \hline
mapCountMinus1                    &  Numbers of layers (rename to maps?)                \\ \hline
singleMapPixelInterleaving        &  Use single layer pixel interleaving                \\ \hline
removeDuplicatePoints             &  Remove duplicate points(                           \\ \hline
surfaceSeparation                 &  surface separation                                 \\ \hline
highGradientSeparation            &  Separate high gradient points from a patch         \\ \hline
minGradient                       &  Minimun gradient for a point to be separated       \\ \hline
minNumHighGradientPoints          &  Minimum number of connected high gradient          \\ 
                                  &  points to be separated from a patch                \\ \hline
packingStrategy                   &  Patches packing strategy(0: anchor packing,        \\ 
                                  &  1(default): flexible packing, 2: tetris            \\ 
                                  &  packing)                                           \\ \hline
useEightOrientations              &  Allow either 2 orientations (0(default): NULL      \\ 
                                  &  AND SWAP), or 8 orientation (1)                    \\ \hline
safeGuardDistance                 &  Number of empty blocks that must exist between     \\ 
                                  &  the patches (default=1)                            \\ \hline
attributeBGFill                     &  Selects the background filling operation for       \\ 
                                  &  attribute only (0: patch-edge extension,             \\ 
                                  &  1(default): smoothed push-pull algorithm), 2:      \\ 
                                  &  harmonic background filling                        \\ \hline
lossyRawPointsPatch               &  Lossy raw points patch(0: no lossy raw points      \\ 
                                  &  patch, 1: enable lossy raw points patch            \\ 
                                  &  (default=0)                                        \\ \hline
minNormSumOfInvDist4MPSelection   &  Minimum normalized sum of inverse distance         \\ 
                                  &  for raw points selection: double value             \\
                                  &  between 0.0 and 1.0 (default=0.35)                 \\ \hline
lossyRawPointPatchGeoQP           &  QP value for geometry in lossy raw points          \\ 
                                  &  patch (default=4)                                  \\ \hline
globalPatchAllocation             &  Global temporally consistent patch                 \\ 
                                  &  allocation.(0: anchor's packing                    \\ 
                                  &  method(default), 1: gpa algorithm, 2: gtp          \\ 
                                  &  algorithm)                                         \\ \hline
globalPackingStrategyGOF          &  Number of frames to pack globally (0:(entire       \\ 
                                  &  GOF))                                              \\ \hline
globalPackingStrategyReset        &  Remove the reference to the previous frame         \\ 
                                  &  (0(default), 1)                                    \\ \hline
globalPackingStrategyThreshold    &  matched patches area ratio threshold (decides      \\ 
                                  &  if connections are valid or not, 0(default))       \\ \hline
patchPrecedenceOrder              &  Order of patches                                   \\ \hline
lowDelayEncoding                  &  Low Delay encoding (0(default): do nothing,        \\ 
                                  &  1: does not allow overlap of patches bounding      \\ 
                                  &  boxes for low delay encoding)                      \\ \hline
geometryPadding                   &  Selects the background filling operation for       \\ 
                                  &  geometry (0: anchor, 1(default): 3D geometry       \\ 
                                  &  padding)                                           \\ \hline
apply3dMotionCompensation         &  Use auxilliary information for 3d motion           \\ 
                                  &  compensation.(0: conventional video coding,        \\ 
                                  &  1: 3D motion compensated)                          \\ \hline
geometry3dCoordinatesBitdepth     &  Bit depth of geomtery 3D coordinates               \\ \hline
geometryNominal2dBitdepth         &  Bit depth of geometry 2D                           \\ \hline
nbPlrmMode                        &  Number of PLR mode                                 \\ \hline
patchSize                         &  Size of Patch for PLR                              \\ \hline
enhancedProjectionPlaneUse        &  Enhanced Projection Plane(0: OFF, 1: ON)           \\ \hline
minWeightEPP                      &  Minimum value                                      \\ \hline
additionalProjectionPlaneMode     &  additiona Projection Plane Mode 0:none             \\ 
                                  &  1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply      \\ 
                                  &  to portion                                         \\ \hline
partialAdditionalProjectionPlane  &  The value determines the partial point cloud.      \\ 
                                  &  It's available with only                           \\ 
                                  &  additionalProjectionPlaneMode(5)                   \\ \hline
enablePointCloudPartitioning      &                                                     \\ \hline
roiBoundingBoxMinX                &                                                     \\ \hline
roiBoundingBoxMaxX                &                                                     \\ \hline
roiBoundingBoxMinY                &                                                     \\ \hline
roiBoundingBoxMaxY                &                                                     \\ \hline
roiBoundingBoxMinZ                &                                                     \\ \hline
roiBoundingBoxMaxZ                &                                                     \\ \hline
numTilesHor                       &                                                     \\ \hline
tileHeightToWidthRatio            &                                                     \\ \hline
numCutsAlong1stLongestAxis        &                                                     \\ \hline
numCutsAlong2ndLongestAxis        &                                                     \\ \hline
numCutsAlong3rdLongestAxis        &                                                     \\ \hline
mortonOrderSortRawPoints          &                                                     \\ \hline
pbfEnableFlag                     &  enable patch block filtering                       \\ \hline
pbfFilterSize                     &  pbfFilterSize                                      \\ \hline
pbfPassesCount                    &  pbfPassesCount                                     \\ \hline
pbfLog2Threshold                  &  pbfLog2Threshold                                   \\ \hline\hline

{\bf Metrics }                    &                                                     \\ \hline\hline
computeChecksum                   & Compute checksum                                    \\ \hline
computeMetrics                    & Compute metrics                                     \\ \hline
normalDataPath                    & Input pointcloud to encode.                         \\ 
                                  & Multi-frame sequences may be                        \\ 
                                  & represented by \%04i                                \\ \hline
resolution                        & Specify the intrinsic resolution                    \\ \hline
dropdups                          & 0(detect), 1(drop), 2(average)                      \\ 
                                  & subsequent points with same                         \\ 
                                  & coordinates                                         \\ \hline
neighborsProc                     & 0(undefined), 1(average), 2(weighted                \\ 
                                  & average), 3(min), 4(max) neighbors                  \\ 
                                  & with same geometric distance                        \\ \hline
\end{longtable}

       
### Decoder parameters       

\begin{longtable}{p{6cm}p{8cm}}
\hline
{\bf Parameter=Value} & {\bf Usage} \\ \hline
help                           & This help text                             \\ \hline\hline
{\bf Global }                  &                                            \\ \hline\hline
c,config                       & Configuration file name                    \\ \hline
compressedStreamPath           & Input                                      \\ 
                               & compressed bitstream                       \\ \hline
reconstructedDataPath          & Output decoded pointcloud. Multi-frame     \\ 
                               & sequences may be represented by \%04i      \\ \hline
startFrameNumber               & Fist frame number in sequence to           \\ 
                               & encode/decode                              \\ \hline
colorTransform                 & The colour transform to be applied:        \\ 
                               &   0: none                                  \\ 
                               &   1: RGB to YCbCr (Rec.709)                \\ \hline
colorSpaceConversion           & Path to the HDRConvert. If unset, an       \\ 
   \ \ \ \ \ \ Path            & internal color space conversion is         \\ 
                               & used                                       \\ \hline
inverseColorSpaceConversion    & HDRConvert configuration file used for     \\ 
   \ \ \ \ \ \ Config          & YUV420 to RGB444 conversion                \\ 
videoDecoderPath=              & HM video decoder executable                \\ \hline
videoDecoderOccupancyMap       & HM lossless video decoder executable       \\ 
   \ \ \ \ \ \ Path            & for occupancy map                          \\ \hline
nbThread                       & Number of thread used for parallel         \\ 
                               & processing                                 \\ \hline
keepIntermediateFiles          & Keep intermediate files: RGB, YUV and      \\ 
                               & bin                                        \\ \hline\hline
testLevelOfDetail              & Disable patch sampling resolution          \\ 
     \ \ \ \ \ \ Signaling     & scaling; use in conjunction with same      \\ 
                               & parameter in encoder                       \\ \hline
patchColorSubsampling          & Enable per-patch color up-sampling         \\ \hline\hline
{\bf Metrics }                 &                                            \\ \hline\hline
computeChecksum=1              & Compute checksum                           \\ \hline
computeMetrics=1               & Compute metrics                            \\ \hline
uncompressedDataFolder         & Folder where the uncompress input data     \\ 
                               & are stored, use for cfg relative           \\ 
                               & paths.                                     \\ \hline
startFrameNumber               & Fist frame number in sequence to           \\ 
                               & encode/decode                              \\ \hline
frameCount                     & Number of frames to encode                 \\ \hline
groupOfFramesSize              & Random access period                       \\ \hline
uncompressedDataPath           & Input pointcloud to encode.                \\ 
                               & Multi-frame sequences may be               \\ 
                               & represented by \%04i                       \\ \hline
reconstructedDataPath          & Output decoded pointcloud. Multi-frame     \\ 
                               & sequences may be represented by \%04i      \\ \hline
normalDataPath                 & Input pointcloud to encode.                \\ 
                               & Multi-frame sequences may be               \\ 
                               & represented by \%04i                       \\ \hline
resolution                     & Specify the intrinsic resolution           \\ \hline
dropdups                       & 0(detect), 1(drop), 2(average)             \\ 
                               & subsequent points with same                \\ 
                               & coordinates                                \\ \hline
neighborsProc                  & 0(undefined), 1(average), 2(weighted       \\ 
                               & average), 3(min), 4(max) neighbors         \\ 
                               & with same geometric distance               \\ \hline
nbThread                       & Number of thread used for parallel         \\ 
                               & processing                                 \\ \hline
minimumImageHeight             & Ignore parameter                           \\ \hline
flagColorPreSmoothing          & Ignore parameter                           \\ \hline
surfaceSeparation              & Ignore parameter                           \\ \hline\hline
{\bf Conformance}              &                                            \\ \hline\hline
checkConformance               & Check conformance                          \\ \hline
path                           & Root directory of conformance files and    \\ 
                               & prefix:   S26C03R03_                       \\ \hline
level                          & Level indice                               \\ \hline
fps                            & Frame per second                           \\ \hline
\end{longtable}                      


### Metrics parameters       

\begin{longtable}{p{6cm}p{8cm}}
\hline
{\bf Parameter=Value} & {\bf Usage} \\ \hline
help                    & This help text                             \\ \hline 
computeChecksum         & Compute checksum                           \\ \hline 
computeMetrics          & Compute metrics                            \\ \hline 
startFrameNumber        & Fist frame number in sequence to           \\ \hline 
                        & encode/decode                              \\ \hline 
frameCount              & Number of frames to encode                 \\ \hline 
uncompressedDataPath    & Input pointcloud to encode. Multi-frame    \\ \hline 
                        & sequences may be represented by \%04i      \\ \hline 
reconstructedDataPath   & Output decoded pointcloud. Multi-frame     \\ \hline 
                        & sequences may be represented by \%04i      \\ \hline 
normalDataPath          & Input pointcloud to encode. Multi-frame    \\ \hline 
                        & sequences may be represented by \%04i      \\ \hline 
resolution              & Specify the intrinsic resolution           \\ \hline 
dropdups                & 0(detect), 1(drop), 2(average) subsequent  \\ \hline 
                        & points with same coordinates               \\ \hline 
neighborsProc           & 0(undefined), 1(average), 2(weighted       \\ \hline 
                        & average), 3(min), 4(max) neighbors with    \\ \hline 
                        & same geometric distance                    \\ \hline 
nbThread                & Number of thread used for parallel         \\ \hline 
                        & processing                                 \\ \hline 
minimumImageHeight      & Ignore parameter                           \\ \hline 
flagColorPreSmoothing   & Ignore parameter                           \\ \hline 
surfaceSeparation       & Ignore parameter                           \\ \hline 
\end{longtable}

