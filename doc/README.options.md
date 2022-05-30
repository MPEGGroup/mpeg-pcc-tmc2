General options
---------------

The next tables shows the parameters of the encoder, decoder and metrics programs.

### Encoder parameters

\begin{longtable}{p{6cm}p{8cm}}
\hline
{\bf Parameter=Value}             & {\bf Usage}                                         \\ \hline
help                              &  This help text                                     \\ \hline\hline
{\bf Global }                     &                                                     \\ \hline\hline
config                            & Configuration file name                             \\ \hline
configurationFolder               & Folder where the configuration files are            \\ 
                                  & stored,use for cfg relative paths.                  \\ \hline
uncompressedDataFolder            & Folder where the uncompress input data are          \\ 
                                  & stored, use for cfg relative paths.                 \\ \hline
uncompressedDataPath              & Input pointcloud to encode. Multi-frame             \\ 
                                  & sequences may be represented by \%04i               \\ \hline
compressedStreamPath              & Output(encoder)/Input(decoder) compressed           \\ 
                                  & bitstream                                           \\ \hline
reconstructedDataPath             & Output decoded pointcloud. Multi-frame              \\ 
                                  & sequences may be represented by \%04i               \\ \hline
forcedSsvhUnitSizePrecisionBytes  & forced SSVH unit size precision bytes               \\ \hline
startFrameNumber                  & First frame number in sequence to                   \\ 
                                  & encode/decode                                       \\ \hline
frameCount                        & Number of frames to encode                          \\ \hline
groupOfFramesSize                 & Random access period                                \\ \hline
colorTransform                    & The colour transform to be applied:                 \\
                                  &   0: none                                           \\ 
                                  &   1: RGB to YCbCr (Rec.709)                         \\ \hline
colorSpaceConversionPath          & Path to the HDRConvert. If unset, an internal       \\ 
                                  & color space conversion is used                      \\ \hline
colorSpaceConversionConfig        & HDRConvert configuration file used for RGB444       \\ 
                                  & to YUV420 conversion                                \\ \hline
inverseColorSpaceConversionConfig & HDRConvert configuration file used for YUV420       \\ 
                                  & to RGB444 conversion                                \\ \hline
gridBasedSegmentation             & Voxel dimension for grid-based segmentation (GBS)   \\ \hline
voxelDimensionGridBased           & Voxel dimension for grid-based segmentation (GBS)   \\ 
  \ \ \ \ \ \ Segmentation        &                                                     \\ \hline
nnNormalEstimation                & Number of points used for normal estimation         \\ \hline
normalOrientation                 & Normal orientation: 0: None 1: spanning tree,       \\
                                  & 2:view point, 3:cubemap projection                  \\ \hline
gridBasedRefineSegmentation       & Use grid-based approach for segmentation            \\
                                  & refinement                                          \\ \hline
maxNNCountRefineSegmentation      & Number of nearest neighbors used during             \\
                                  & segmentation refinement                             \\ \hline
iterationCountRefineSegmentation  & Number of iterations performed during               \\
                                  & segmentation refinement                             \\ \hline
voxelDimensionRefineSegmentation  & Voxel dimension for segmentation refinement         \\
                                  & (must be a power of 2)                              \\ \hline
searchRadiusRefineSegmentation    & Search radius for segmentation refinement           \\ \hline
occupancyResolution               & Resolution of packing block(a block contain         \\ 
                                  & only one patch)                                     \\ \hline
enablePatchSplitting              & Enable patch splitting                              \\ \hline
maxPatchSize                      & Maximum patch size for segmentation                 \\ \hline
log2QuantizerSizeX                & log2 of Quantization step for patch size X:         \\ 
                                  & 0. pixel precision 4.16 as before                   \\ \hline
log2QuantizerSizeY                & log2 of Quantization step for patch size Y:         \\ 
                                  & 0. pixel precision 4.16 as before                   \\ \hline
minPointCountPerCCPatch           & Minimum number of points for a connected            \\ 
  \ \ \ \ \ \ Segmentation        & component to be retained as a patch                 \\ \hline 
maxNNCountPatchSegmentation       & Number of nearest neighbors used during             \\ 
                                  & connected components extraction                     \\ \hline 
surfaceThickness                  & Surface thickness                                   \\ \hline 
depthQuantizationStep             & minimum level for patches                           \\ \hline 
maxAllowedDist2RawPointsDetection & Maximum distance for a point to be ignored          \\ 
                                  & during raw points detection                         \\ \hline 
maxAllowedDist2RawPointsSelection & Maximum distance for a point to be ignored          \\ 
                                  & during  raw points  selection                       \\ \hline 
lambdaRefineSegmentation          & Controls the smoothness of the patch                \\ 
                                  & boundaries  during segmentation  refinement         \\ \hline 
minimumImageWidth                 & Minimum width of packed patch frame                 \\ \hline 
minimumImageHeight                & Minimum height of packed patch frame                \\ \hline 
maxCandidateCount                 & Maximum nuber of candidates in list L               \\ \hline 
occupancyPrecision                & Occupancy map B0 precision                          \\ \hline 
occupancyMapConfig                & Occupancy map encoder config file                   \\ \hline 
occupancyMapQP                    & QP for compression of occupancy map video           \\ \hline 
enhancedOccupancyMapCode          & Use enhanced-delta-depth code                       \\ \hline 
EOMFixBitCount                    & enhanced occupancy map fixed bit count              \\ \hline 
occupancyMapRefinement            & Use occupancy map refinement                        \\ \hline 
decodedAtlasInformationHash       & Enable decoded atlas information hash: 0. disable   \\ 
                                  & 1.MD5 2.CRC 3.Checksum                              \\ \hline 
attributeTransferFilterType       & Exclude geometry smoothing from attribute transfer  \\ \hline
flagGeometrySmoothing             & Enable geometry smoothing                           \\ \hline 
neighborCountSmoothing            & Neighbor count smoothing                            \\ \hline 
radius2Smoothing                  & Radius to smoothing                                 \\ \hline 
radius2BoundaryDetection          & Radius to boundary detection                        \\ \hline 
thresholdSmoothing                & Threshold smoothing                                 \\ \hline 
patchExpansion                    & Use occupancy map refinement                        \\ \hline 
gridSmoothing                     & Enable grid smoothing                               \\ \hline 
gridSize                          & grid size for the smoothing                         \\ \hline 
thresholdColorSmoothing           & Threshold of color smoothing                        \\ \hline 
cgridSize                         & grid size for the color smoothing                   \\ \hline 
thresholdColorDifference          & Threshold of color difference between cells         \\ \hline 
thresholdColorVariation           & Threshold of color variation in cells               \\ \hline 
flagColorSmoothing                & Enable color smoothing                              \\ \hline 
thresholdColorPreSmoothing        & Threshold of color pre-smoothing                    \\ \hline 
thresholdColorPreSmoothingLocal   & Threshold of color pre-smoothing local              \\ 
  \ \ \ \ \ \ Entropy             & entropy                                             \\ \hline 
radius2ColorPreSmoothing          & Radius of color pre-smoothing                       \\ \hline 
neighborCountColorPreSmoothing    & Neighbor count for color pre-smoothing              \\ \hline 
flagColorPreSmoothing             & Enable color pre-smoothing                          \\ \hline 
bestColorSearchRange              & Best color search range                             \\ \hline 
numNeighborsColorTransferFwd      & Number of neighbors creating Fwd list               \\ \hline 
numNeighborsColorTransferBwd      & Number of neighbors creating Bwd list               \\ \hline 
useDistWeightedAverageFwd         & Distance weighted average for Fwd list              \\ \hline 
useDistWeightedAverageBwd         & Distance weighted average for Bwd list              \\ \hline 
skipAvgIfIdenticalSourcePoint     & Skip avgeraging if target is identical to a         \\ 
  \ \ \ \ \ \ PresentFwd          & Fwd point                                           \\ \hline 
skipAvgIfIdenticalSourcePoint     & Skip avgeraging if target is identical to a         \\ 
  \ \ \ \ \ \  PresentBwd         & Bwd point                                           \\ \hline 
distOffsetFwd                     & Distance offset to avoid infinite weight            \\ \hline 
distOffsetBwd                     & Distance offset to avoid infinite weight            \\ \hline 
maxGeometryDist2Fwd               & Maximum allowed distance for a Fwd point            \\ \hline 
maxGeometryDist2Bwd               & Maximum allowed distance for a Bwd point            \\ \hline 
maxColorDist2Fwd                  & Maximum allowed pari-wise color distance for        \\ 
                                  & Fwd list                                            \\ \hline 
maxColorDist2Bwd                  & Maximum allowed pari-wise color distance for        \\ 
                                  & Bwd list                                            \\ \hline 
excludeColorOutlier               & Exclude color outliers from the NN set              \\ \hline 
thresholdColorOutlierDist         & Threshold of color distance to exclude              \\ 
                                  & outliers from the NN set                            \\ \hline 
videoEncoderOccupancyPath         & Occupancy video encoder executable path             \\ \hline 
videoEncoderGeometryPath          & Geometry video encoder executable path              \\ \hline 
videoEncoderAttributePath         & Attribute video encoder executable path             \\ \hline 
videoEncoderOccupancyCodecId      & Occupancy video encoder codec id                    \\ \hline 
videoEncoderGeometryCodecId       & Geometry video encoder codec id                     \\ \hline 
videoEncoderAttributeCodecId      & Attribute video encoder codec id                    \\ \hline 
videoEncoderInternalBitdepth      & Video encoder internal bitdepth                     \\ \hline 
byteStreamVideoEncoderOccupancy   & Attribute video encoder outputs byteStream          \\ \hline 
byteStreamVideoEncoderGeometry    & Attribute video encoder outputs byteStream          \\ \hline 
byteStreamVideoEncoderAttribute   & Attribute video encoder outputs byteStream          \\ \hline 
geometryQP                        & QP for compression of geometry video                \\ \hline 
attributeQP                       & QP for compression of attribute video               \\ \hline 
auxGeometryQP                     & QP for compression of auxiliary geometry            \\ 
                                  & video : default=4 for lossy raw points,             \\ 
                                  & geometryQP for lossless raw points                  \\ \hline 
auxAttributeQP                    & QP for compression of auxiliary attribute video     \\ \hline 
geometryConfig                    & HM configuration file for geometry compression      \\ \hline 
geometry0Config                   & HM configuration file for geometry 0 compression    \\ \hline 
geometry1Config                   & HM configuration file for geometry 1 compression    \\ \hline 
attributeConfig                   & HM configuration file for attribute compression     \\ \hline 
attribute0Config                  & HM configuration file for attribute 0 compression   \\ \hline 
attribute1Config                  & HM configuration file for attribute 1 compression   \\ \hline 
rawPointsPatch                    & Enable raw points patch                             \\ \hline 
noAttributes                      & Disable encoding of attributes                      \\ \hline 
attributeVideo444                 & Use 444 format for attribute video                  \\ \hline 
useRawPointsSeparateVideo         & Compress raw points with video codec                \\ \hline 
attributeRawSeparateVideoWidth    & Width of the MP's attribute in separate video       \\ \hline 
geometryMPConfig                  & HM configuration file for raw points geometry       \\ 
                                  & compression                                         \\ \hline 
attributeMPConfig                 & HM configuration file for raw points                \\ 
                                  & attribute compression                               \\ \hline 
nbThread                          & Number of thread used for parallel processing       \\ \hline 
keepIntermediateFiles             & Keep intermediate files: RGB, YUV and bin           \\ \hline 
absoluteD1                        & Absolute D1                                         \\ \hline 
absoluteT1                        & Absolute T1                                         \\ \hline 
multipleStreams                   & number of video(geometry and attribute) streams     \\ \hline 
deltaQPD0                         & qp adjustment for geometry0 video: 0, +3, -3...     \\ \hline 
deltaQPD1                         & qp adjustment for geometry1 video: 0, +3, -3...     \\ \hline 
deltaQPT0                         & qp adjustment for attribute0 video: 0, +3, -3...    \\ \hline 
deltaQPT1                         & qp adjustment for attribute1 video: 0, +3, -3...    \\ \hline 
constrainedPack                   & Temporally consistent patch packing                 \\ \hline 
levelOfDetailX                    & levelOfDetail : X axis in 2D space (should be       \\ 
                                  & greater than 1)                                     \\ \hline 
levelOfDetailY                    & levelOfDetail : Y axis in 2D space (should be       \\ 
                                  & greater than 1)                                     \\ \hline 
groupDilation                     & Group Dilation                                      \\ \hline 
offsetLossyOM                     & Value to be assigned to non-zero occupancy map      \\ 
                                  & positions (default=0)                               \\ \hline 
thresholdLossyOM                  & Threshold for converting non-binary occupancy       \\ 
                                  & map to binary (default=0)                           \\ \hline 
prefilterLossyOM                  & Selects whether the occupany map is prefiltered     \\ 
                                  & before lossy compression (default=false)            \\ \hline 
shvcLayerIndex                    & Decode Layer ID number using SHVC codec             \\ \hline 
shvcRateX                         & SHVCRateX: reduce rate of each SHVC layer X         \\ 
                                  & axis in 2D space (should be greater than 1)         \\ \hline 
shvcRateY                         & SHVCRateY: reduce rate of each SHVC layer Y         \\ 
                                  & axis in 2D space (should be greater than 1)         \\ \hline 
patchColorSubsampling             & Enable per patch color sub-sampling                 \\ \hline 
maxNumRefAtalsList                & maximum Number of Reference Atlas Frame list,       \\ 
                                  & default: 1                                          \\ \hline 
maxNumRefAtlasFrame               & maximum Number of Reference Atlas Frame per         \\ 
                                  & list, default: 1                                    \\ \hline 
pointLocalReconstruction          & Use point local reconstruction                      \\ \hline 
mapCountMinus1                    & Numbers of layers (rename to maps?)                 \\ \hline 
singleMapPixelInterleaving        & Use single layer pixel interleaving                 \\ \hline 
removeDuplicatePoints             & Remove duplicate points(                            \\ \hline 
surfaceSeparation                 & surface separation                                  \\ \hline 
highGradientSeparation            & Separate high gradient points from a patch          \\ \hline 
minGradient                       & Minimun gradient for a point to be separated        \\ \hline 
minNumHighGradientPoints          & Minimum number of connected high gradient           \\ 
                                  & points to be separated from a patch                 \\ \hline 
packingStrategy                   & Patches packing strategy(0: anchor packing,         \\ 
                                  & 1(default): flexible packing, 2: tetris             \\ 
                                  & packing)                                            \\ \hline 
useEightOrientations              & Allow either 2 orientations (0(default): NULL       \\ 
                                  & AND SWAP), or 8 orientation (1))                    \\ \hline 
safeGuardDistance                 & Number of empty blocks that must exist between      \\ 
                                  & the patches (default=1)                             \\ \hline 
attributeBGFill                   & Selects the background filling operation for        \\ 
                                  & attribute only (0: patch-edge extension,            \\ 
                                  & 1(default): smoothed push-pull algorithm), 2:       \\ 
                                  & harmonic background filling                         \\ \hline 
lossyRawPointsPatch               & Lossy raw points patch(0: no lossy raw points       \\ 
                                  & patch, 1: enable lossy raw points patch             \\ 
                                  & (default=0)                                         \\ \hline 
minNormSumOfInvDist4MPSelection   & Minimum normalized sum of inverse distance          \\ 
                                  & for raw points selection: double value              \\ 
                                  & between 0.0 and 1.0 (default=0.35)                  \\ \hline 
globalPatchAllocation             & Global temporally consistent patch                  \\ 
                                  & allocation.(0: anchor's packing                     \\ 
                                  & method(default), 1: gpa algorithm, 2: gtp           \\ 
                                  & algorithm)                                          \\ \hline 
globalPackingStrategyGOF          & Number of frames to pack globally (0:(entire        \\ 
                                  & GOF))                                               \\ \hline 
globalPackingStrategyReset        & Remove the reference to the previous frame          \\ 
                                  & (0(default), 1)                                     \\ \hline 
globalPackingStrategyThreshold    & Matched patches area ratio threshold (decides       \\ 
                                  & if connections are valid or not, 0(default))        \\ \hline 
patchPrecedenceOrder              & Order of patches                                    \\ \hline 
lowDelayEncoding                  & Low Delay encoding (0(default): do nothing,         \\ 
                                  & 1: does not allow overlap of patches bounding       \\ 
                                  & boxes for low delay encoding)                       \\ \hline 
geometryPadding                   & Selects the background filling operation for        \\ 
                                  & geometry (0: anchor, 1(default): 3D geometry        \\ 
                                  & padding)                                            \\ \hline 
apply3dMotionCompensation         & Use auxilliary information for 3d motion            \\ 
                                  & compensation.(0: conventional video coding,         \\ 
                                  & 1: 3D motion compensated)                           \\ \hline 
usePccRDO                         & Use HEVC PCC RDO optimization                       \\ \hline 
geometry3dCoordinatesBitdepth     & Bit depth of geomtery 3D coordinates                \\ \hline 
geometryNominal2dBitdepth         & Bit depth of geometry 2D                            \\ \hline 
nbPlrmMode                        & Number of PLR mode                                  \\ \hline 
patchSize                         & Size of Patch for PLR                               \\ \hline 
enhancedProjectionPlane           & Use enhanced Projection Plane(0: OFF, 1: ON)        \\ \hline 
minWeightEPP                      & Minimum value                                       \\ \hline 
additionalProjectionPlaneMode     & additional projection plane mode: 0:none            \\ 
                                  & 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply       \\ 
                                  & to portion                                          \\ \hline 
partialAdditionalProjectionPlane  & The value determines the partial point cloud.       \\ 
                                  & It's available with only                            \\ 
                                  & additionalProjectionPlaneMode(5)                    \\ \hline 
numMaxTilePerFrame                & number of maximum tiles in a frame                  \\ \hline 
uniformPartitionSpacing           & indictation of uniform partitioning                 \\ \hline 
tilePartitionWidth                & uniform partition width in the unit of 64 pixels    \\ \hline 
tilePartitionHeight               & uniform partition height in the unit of 64 pixels   \\ \hline 
tilePartitionWidthList            & non uniform partition width in the unit of 64 pixels\\ \hline 
tilePartitionHeightListnon        & uniform partition height in the unit of 64 pixels   \\ \hline 
tileSegmentationType              & tile segmentaton method : 0.no tile partition       \\ \hline 
                                  & 1. 3D ROI based 2.2D Patch size based               \\ \hline 
enablePointCloudPartitioning      & enablePointCloudPartitioning                        \\ \hline 
roiBoundingBoxMinX                & roiBoundingBoxMinX                                  \\ \hline
roiBoundingBoxMaxX                & roiBoundingBoxMaxX                                  \\ \hline
roiBoundingBoxMinY                & roiBoundingBoxMinY                                  \\ \hline
roiBoundingBoxMaxY                & roiBoundingBoxMaxY                                  \\ \hline
roiBoundingBoxMinZ                & roiBoundingBoxMinZ                                  \\ \hline
roiBoundingBoxMaxZ                & roiBoundingBoxMaxZ                                  \\ \hline
numTilesHor                       & numTilesHor                                         \\ \hline
tileHeightToWidthRatio            & tileHeightToWidthRatio                              \\ \hline
numCutsAlong1stLongestAxis        & numCutsAlong1stLongestAxis                          \\ \hline 
numCutsAlong2ndLongestAxis        & numCutsAlong2ndLongestAxis                          \\ \hline 
numCutsAlong3rdLongestAxis        & numCutsAlong3rdLongestAxis                          \\ \hline 
mortonOrderSortRawPoints          & mortonOrderSortRawPoints                            \\ \hline 
pbfEnableFlag                     & Enable patch block filtering                        \\ \hline 
pbfFilterSize                     & pbfFilterSize                                       \\ \hline 
pbfPassesCount                    & pbfPassesCount                                      \\ \hline 
pbfLog2Threshold                  & pbfLog2Threshold                                    \\ \hline 
computeChecksum                   & Compute checksum                                    \\ \hline 
computeMetrics                    & Compute metrics                                     \\ \hline 
normalDataPath                    & Input pointcloud to encode. Multi-frame             \\ 
                                  & sequences may be represented by \%04i               \\ \hline 
resolution                        & Specify the intrinsic resolution                    \\ \hline 
dropdups                          & 0(detect), 1(drop), 2(average) subsequent           \\ 
                                  & points with same coordinates                        \\ \hline 
neighborsProc                     & 0(undefined), 1(average), 2(weighted                \\ 
                                  & average), 3(min), 4(max) neighbors with same        \\ 
                                  & geometric distance                                  \\ \hline 
tierFlag                          & Tier Flag                                           \\ \hline 
profileCodecGroupIdc              & Profile Codec Group Idc                             \\ \hline 
profileToolsetIdc                 & Profile Toolset Idc                                 \\ \hline 
profileReconstructionIdc          & Profile Reconstruction Idc                          \\ \hline 
levelIdc                          & Level Idc                                           \\ \hline 
avcCodecIdIndex                   & Index for avc codec                                 \\ \hline 
hevcCodecIdIndex                  & Index for hevc codec                                \\ \hline 
shvcCodecIdIndex                  & Index for shvc codec                                \\ \hline 
vvcCodecIdIndex                   & Index for vvc codec                                 \\ \hline 
oneV3CFrameOnlyFlag               & One V3C Frame Only Flag                             \\ \hline 
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
   \ \ \ \ \ \ Config          & YUV420 to RGB444 conversion                \\ \hline
videoDecoderOccupancyPath      & Occupancy video decoder executable         \\ \hline
videoDecoderGeometryPath       & Geometry video decoder executable          \\ \hline
videoDecoderAttributePath      & Attribute video decoder executable         \\ \hline
byteStreamVideoCoderOccupancy  & Occupancy video decoder inputs byteStream  \\ \hline
byteStreamVideoCoderGeometry   & Geometry video decoder inputs byteStream   \\ \hline
byteStreamVideoCoderAttribute  & Attribute video decoder inputs byteStream  \\ \hline
nbThread                       & Number of thread used for parallel         \\ \hline
                               & processing                                 \\ \hline
attributeTransferFilterType    & Exclude geometry smoothing from attribute  \\ \hline
                               & transfer                                   \\ \hline
keepIntermediateFiles          & Keep intermediate files: RGB, YUV and bin  \\ \hline
shvcLayerIndex                 & Decode Layer ID number using SHVC codec    \\ \hline
patchColorSubsampling          & Enable per-patch color up-sampling         \\ \hline\hline
{\bf Metrics }                 &                                            \\ \hline\hline
computeChecksum                & Compute checksum                           \\ \hline
computeMetrics                 & Compute metrics                            \\ \hline
uncompressedDataFolder         & Folder where the uncompress input data are \\ \hline
                               & stored, use for cfg relative  paths.       \\ \hline
startFrameNumber               & Fist frame number in sequence to           \\ \hline
                               & encode/decode                              \\ \hline
frameCount                     & Number of frames to encode                 \\ \hline
groupOfFramesSize              & Random access period                       \\ \hline
uncompressedDataPath           & Input pointcloud to encode. Multi-frame    \\ 
                               & sequences may be represented by \%04i      \\ \hline
reconstructedDataPath          & Output decoded pointcloud. Multi-frame     \\ 
                               & sequences may be represented by \%04i      \\ \hline
normalDataPath                 & Input pointcloud to encode. Multi-frame    \\ 
                               & sequences may be represented by \%04i      \\ \hline
resolution                     & Specify the intrinsic resolution           \\ \hline
dropdups                       & 0(detect), 1(drop), 2(average) subsequent  \\ 
                               & points with same coordinates               \\ \hline
neighborsProc                  & 0(undefined), 1(average), 2(weighted       \\ 
                               & average), 3(min), 4(max) neighbors with    \\
                               & same geometric distance                    \\ \hline
nbThread                       & Number of thread used for parallel         \\ \hline
                               & processing                                 \\ \hline
minimumImageHeight             & Ignore parameter                           \\ \hline
flagColorPreSmoothing          & Ignore parameter                           \\ \hline
surfaceSeparation              & Ignore parameter                           \\ \hline\hline
{\bf Conformance}              &                                            \\ \hline\hline
checkConformance               & Check conformance                          \\ \hline
path                           & Root directory of conformance files +      \\
                               & prefix: S26C03R03_                         \\ \hline
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

