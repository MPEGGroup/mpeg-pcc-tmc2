Using the codec
===============


```
  ../bin/PccAppEncoder [--help] [-c config.cfg] [--parameter=value]
  ../bin/PccAppDecoder [--help] [--parameter=value]
  ../bin/PccAppMetrics [--help] [--parameter=value]
```

Principle
---------------

The encoder takes as input a PLY file describing a point cloud with
integer positions and, optionally, per-point integer colour attributes.

The output of the encoder is a binary bitstream encapsulated using the
V3C annex-B format.

Conversely, the decoder takes as input a compressed bitstream file in
V3C annex-B format and produces a reconstructed PLY file with position
and any present attribute values.

The software may be configured using either command line arguments or from
a configuration file specified using the `-c|--config=` option.

Sample configuration files are provided in the cfg/ directory.

Parameters are set by the last value encountered on the command line.
Therefore if a setting is set via a configuration file, and then a
subsequent command line parameter changes that same setting, the command
line parameter value will be used.



Common test condition configurations
---------------

Configuration files are provided in the cfg directory to aid configuring the encoder.  The general pattern of usage is illustrated below, where
multiple configuration files control different aspects of the test conditions.

NB: parameters set in one configuration file override the same parameter in earlier files.  ie. order matters.

Further help text describing option usage is available using "./bin/PccAppEncoder --help" or "./bin/PccAppDecoder --help".

Examples
---------------

### Encoder

The next command line encodes one streams:

```
./bin/PccAppEncoder \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --colorTransform=0 \
  --configurationFolder=./cfg/ \
  --uncompressedDataFolder=./People/ \
  --colorSpaceConversionPath=HDRConvert \
  --videoEncoderPath=TAppEncoderHighBitDepthStatic \
  --videoEncoderOccupancyMapPath=TAppEncoderHighBitDepthStatic \
  --compressedStreamPath=./S22C2AI_queen/S22C2AIR01_queen.bin \
  --frameCount=32
```

To compute the metrics in the encode, the normal of the source point cloud
must be given to the encoder. The next parameter must be added to the previous 
command:
```
  --normalDataPath=./People/Technicolor/queen_n/frame_%04d_n.ply 
```


### Decoder

The next command line decodes one streams:

```
./bin/PccAppDecoder \
  --startFrameNumber=0000 \
  --compressedStreamPath=./S22C2AI_queen/S22C2AIR01_queen.bin \
  --reconstructedDataPath=./S22C2AI_queen/S22C2AIR01_queen_dec_%04d.ply \
  --videoDecoderPath=TAppDecoderHighBitDepthStatic \
  --videoDecoderOccupancyMapPath=TAppDecoderHighBitDepthStatic \
  --colorSpaceConversionPath=./external/HDRTools/bin/HDRConvert \
  --inverseColorSpaceConversionConfig=./cfg/hdrconvert/yuv420torgb444.cfg \
  --nbThread=1 \
  --colorTransform=0 \
```

To compute the metrics in the decoder, the normal of the source point cloud and
the source PLY must be given to the decoder. The next parameter must be added 
to the previous command:
   
```
  --config=./cfg/sequence/queen.cfg \
  --uncompressedDataFolder=./People/ \
  --normalDataPath=./People/Technicolor/queen_n/frame_%04d_n.ply 
```


### Metrics

PccAppMetrics could be used to test the PccLibMetrics. For CTC experiments, 
it's sugested to used mpeg-pcc-dmetrics:
http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-dmetric.git. 


For example, mpeg-pcc-dmetric and PccAppMetric could be used with the next 
command line:

```
../bin/PccAppMetrics \
  --uncompressedDataPath=longdress_vox10_1051.ply \
  --reconstructedDataPath=./S26C2AIR01_longdress_dec_1051.ply \
  --normalDataPath=./People/8i/longdress_n/longdress_vox10_1051_n.ply \
  --resolution=1023 \
  --frameCount=1
  
./mpeg-pcc-demetric/test/pc_error \
  --fileA=./People/8i/8iVFBv2/longdress/Ply/longdress_vox10_1051.ply  \
  --fileB=S26C2AIR01_longdress_dec_1051.ply \
  --inputNorm=./People/8i/longdress_n/longdress_vox10_1051_n.ply \ 
  --color=1 \
  --resolution= 1023  
``` 

The two softwares give the same results.


### Scripts

More examples of running could be found in ./test/runme_linux.sh. 

These examples can be start based on your system with the following scripts:
- ./test/runme_linux.sh
- ./test/runme_windows.bat
- ./test/runme_osx.sh

The V3C common test condition (CTC) command lines could be found in ./test/ctc_command_line.sh. 