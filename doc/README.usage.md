Using the codec
===============


```console
  $ ../bin/PccAppEncoder [--help] [-c config.cfg] [--parameter=value]
  $ ../bin/PccAppDecoder [--help] [--parameter=value]
  $ ../bin/PccAppMetrics [--help] [--parameter=value]
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

Further help text describing option usage is available using `./bin/PccAppEncoder --help` or `./bin/PccAppDecoder --help`.

Examples
---------------

### Encoder

The next command line encodes one streams:

```console 
$ ./bin/PccAppEncoder \
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
```console 
--normalDataPath=./People/Technicolor/queen_n/frame_%04d_n.ply 
```


### Decoder

The next command line decodes one streams:

```console 
$ ./bin/PccAppDecoder \
  --startFrameNumber=0000 \
  --compressedStreamPath=./S22C2AIR01_queen.bin \
  --reconstructedDataPath=./S22C2AIR01_queen_dec_%04d.ply \
  --videoDecoderPath=TAppDecoderHighBitDepthStatic \
  --videoDecoderOccupancyMapPath=TAppDecoderHighBitDepthStatic \
  --colorSpaceConversionPath=./external/HDRTools/bin/HDRConvert \
  --inverseColorSpaceConversionConfig=\\
    ./cfg/hdrconvert/yuv420torgb444.cfg \
  --nbThread=1 \
  --colorTransform=0 \
```

To compute the metrics in the decoder, the normal of the source point cloud and
the source PLY must be given to the decoder. The next parameter must be added 
to the previous command:
   
```console
--config=./cfg/sequence/queen.cfg \
--uncompressedDataFolder=./People/ \
--normalDataPath=\\
  ./People/Technicolor/queen_n/frame_%04d_n.ply
```


### Metrics

PccAppMetrics could be used to test the PccLibMetrics. For CTC experiments, 
it's sugested to used mpeg-pcc-dmetrics:
http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-dmetric.git. 


For example, mpeg-pcc-dmetric and PccAppMetric could be used with the next 
command line:

```console 
$ ../bin/PccAppMetrics \
  --uncompressedDataPath=longdress_vox10_1051.ply \
  --reconstructedDataPath=./S26C2AIR01_longdress_dec_1051.ply \
  --normalDataPath=\\
    ./People/8i/longdress_n/longdress_vox10_1051_n.ply \
  --resolution=1023 \
  --frameCount=1
  
$ ./mpeg-pcc-demetric/test/pc_error \
  --fileA=\\
    ./People/8i/8iVFBv2/longdress/Ply/longdress_vox10_1051.ply  \
  --fileB=S26C2AIR01_longdress_dec_1051.ply \
  --inputNorm=\\
    ./People/8i/longdress_n/longdress_vox10_1051_n.ply \ 
  --color=1 \
  --resolution= 1023  
``` 

The two softwares give the same results.


### Scripts

More examples of running could be found in ./test/runme_linux.sh. 

These examples can be start based on your system with the following scripts:
* ./test/runme_linux.sh
* ./test/runme_windows.bat
* ./test/runme_osx.sh

The V3C common test condition (CTC) command lines could be found in ./test/ctc_command_line.sh. 


### SHVC Information

The SHVC software used in the program can be obtained from the link below:

	https://hevc.hhi.fraunhofer.de/svn/svn_SHVCSoftware/tags/SHM-12.4/

The additional Enhanced Layer will be used by SHVC codec according to the number of entries entered in the SHVCLayer in the encoder.
SHVCRateX and SHVCRateY refer to the width and height resolution reduction rate of 2D images of additional layers.
The decoder uses the video corresponding to the layer entered into the SHVCLayerID. LID 0 has the lowest density, and if 3 layers are used, LID 2 has the same density as V-PCC TMC2 output.
Occupancy Map video encode/decode using same version of HM encooder/decoder.


#### SHVC Running 3layer PccAppEncoder

```console 
$ ./bin/PccAppEncoder \
	--configurationFolder=cfg/ \
	--config=cfg/common/ctc-common.cfg \
	--config=cfg/condition/ctc-random-access-svc-3L.cfg \
	--config=cfg/sequence/longdress_vox10.cfg \
	--config=cfg/rate/ctc-r3.cfg \
	--uncompressedDataFolder=Dynamic_Objects/People/ \
	--frameCount=1 \
	--videoEncoderGeometryPath=..\bin\TAppEncoder.exe \
	--videoEncoderAttributePath=..\bin\TAppEncoder.exe \
 	--videoEncoderOccupancyPath=..\bin\occupancy\TAppEncoder.exe \
	--colorSpaceConversionPath=../external/HDRTools/bin/HDRConvert \
	--reconstructedDataPath=S26C03R03_rec_%04d.ply \
	--compressedStreamPath=S26C03R03.bin \
	--SHVCLayer=2 \
	--SHVCRateX=2 \
	--SHVCRateY=2
```

#### SHVC Running 3layer PccAppDecoder

```console 
$ ./bin/PccAppDecoder \
	--compressedStreamPath=S26C03R03.bin \
	--videoDecoderGeometryPath=..\bin\TAppDecoder.exe \
	--videoDecoderAttributePath=..\bin\TAppDecoder.exe \
	--videoDecoderOccupancyPath=..\bin\occupancy\TAppDecoder.exe \
	--colorSpaceConversionPath=../external/HDRTools/bin/HDRConvert \ 
	--inverseColorSpaceConversionConfig=\\
    cfg/hdrconvert/yuv420torgb444.cfg \
	--reconstructedDataPath=S26C03R03_dec_%04d.ply \
	--SHVCLayerID=2 
```