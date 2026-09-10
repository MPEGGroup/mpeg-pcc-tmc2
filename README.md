# Video Point Cloud Compression - VPCC - mpeg-pcc-tmc2 test model candidate software

## Building

Bash scripts can be use to build mpeg-pcc-tmc2 project: 
- build.sh: build solutions.  
- clear.sh: clear solututions ( ./clear.sh all: to clear dependencies)
 
### OSX
- mkdir build
- cd build
- cmake .. 
- cmake --build . --config Release --parallel 8

### Linux
- mkdir build
- cd build
- cmake .. 
- cmake --build . --config Release --parallel 8

### Windows
- md build
- cd build
- cmake .. 
- cmake --build . --config Release --parallel 8

### V-PCC main profile

For V-PCC main profile experiments build TMC2 with a different release of the HEVC test model:

- cmake .. -DHM_VERSION=HM-16.20+SCM-8.8

### External dependencies

According to the CMake options defined in the CMakeLists.txt, the TMC2 required some external dependencies to work: 

  - USE_JMAPP_VIDEO_CODEC: use JM software to encoder and decoder videos (codecId parameters must be set equal to 0 and the videoEncoderOccupancyPath, videoEncoderGeometryPath and videoEncoderAttributePath but be set the JM applications)
  - USE_HMAPP_VIDEO_CODEC: use HM software to encoder and decoder videos (codecId parametesr must be set equal to 1 and the videoEncoderOccupancyPath, videoEncoderGeometryPath and videoEncoderAttributePath but be set the JM applications)
  - USE_JMLIB_VIDEO_CODEC: use JM library to encoder and decoder videos (codecId parameter must be set equal to 2)
  - USE_HMLIB_VIDEO_CODEC: use HM library to encoder and decoder videos (codecId parameter must be set equal to 3)
  - USE_VTMLIB_VIDEO_CODEC: use VTM library to encoder and decoder videos (codecId parameter must be set equal to 4)
  - USE_FFMPEG_VIDEO_CODEC: use FFMPEG library to encoder and decoder videos (codecId parameter must be set equal to 5). This mode is only available in the FFMPEG branch. 
  - USE_SHMAPP_VIDEO_CODEC: use SHM software to encoder and decoder videos (codecId parametesr must be set equal to 6, the videoEncoderGeometryPath and videoEncoderAttributePath using the SHM application, and the videoEncoderOccupancyPath using the HM application)
  - USE_HDRTOOLS: use HDRTools to convert the raw video files.

The video encoder softwares and libraries can be found in the corresponding repositories: 

  - JM: https://vcgit.hhi.fraunhofer.de/jct-vc/JM.git
  - HM: https://vcgit.hhi.fraunhofer.de/jvet/HM.git
  - VTM: https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git
  - SHM: https://vcgit.hhi.fraunhofer.de/jvet/SHM.git

Some changes have been made on these libraries to allow to use the three libraries at the same time and to increase the codec efficiencies for the V3C contents. The three codecs must be patched with the files:

  - JM: dependencies/jm-modification/PCC_JM.patch
	- HM: dependencies/hm-modification/<HM_VERSION>.patch
  - VTM: dependencies/vtm-modification/adaptions_for_vtm_11_2.patch

By default according the the CMake options, the dependencies are cloned and patched by the cmake process.
For HM, CMake uses `HM_VERSION` (default: `HM-16.20+SCM-8.8`) and applies one of:

	- dependencies/hm-modification/<HM_VERSION>.patch
	- dependencies/hm-modification/<HM_VERSION>_with_RDO.patch (when `USE_HM_PCC_RDO=ON`)

The recommended way to build with a specific HM version is via `build.sh`:

	./build.sh                  # Release build with default HM-16.20+SCM-8.8
	./build.sh hm=18.0          # Release build with HM-18.0
	./build.sh debug hm=18.0    # Debug build with HM-18.0

Alternatively, CMake can be invoked directly (build directory must match the mode):

	cmake -S . -B build/Release -DHM_VERSION=HM-18.0
	cmake --build build/Release --parallel 8

To skip patch application entirely (e.g. to test against a raw, unmodified HM checkout):

	cmake -S . -B build/Release -DHM_VERSION=HM-18.0 -DHM_SKIP_PATCH=ON

The external dependencies could be downloaded, built and linked independenly: 

- JM:
   
   git clone checkout https://vcgit.hhi.fraunhofer.de/jct-vc/JM.git dependencies/jm19.0_lib
   cd dependencies/jm19.0_lib
   git patch ../jm-modification/PCC_JM.patch

- HM:

	HM_VERSION=HM-16.20+SCM-8.8
	git clone --depth 1 --branch ${HM_VERSION} https://vcgit.hhi.fraunhofer.de/jvet/HM.git dependencies/${HM_VERSION}
	cd dependencies/${HM_VERSION}
	git apply ../hm-modification/${HM_VERSION}.patch --whitespace=nowarn

- VTM:

   git clone checkout https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git dependencies/VTM-11.2    
   cd dependencies/VTM-11.2
   git patch ../vtm-modification/adaptions_for_vtm_11_2.patch
 
- HDRTools

    git clone -b 0.17-dev https://gitlab.com/standards/HDRTools.git dependencies/HDRTools       

The pointcloud metrics can be computed inside the TM2 encoder and decoder according to the input parameters: 

    - computeMetrics: Compute metrics
    - uncompressedDataPath: Input pointcloud to encode. Multi-frame sequences may be represented by %04i
    - normalDataPath:  Input pointcloud to encode. Multi-frame sequences may be represented by %04i
    - resolution: Specify the intrinsic resolution
    - dropdups: 0(detect), 1(drop), 2(average) subsequent points with same coordinates
    - neighborsProc: 0(undefined), 1(average), 2(weighted average), 3(min), 4(max) neighbors with same geometric distance

The computations of the metrics are the same than the distances computed with the pcc_distortion software that can be found in: https://git.mpeg.expert/MPEG/3dgh/v-pcc/software/mpeg-pcc-dmetric.git.

## Software checks

To run the software checks, set-up a Python virtual environment and run the checks:

### POSIX

```shell
python3 -m venv venv
. venv/bin/activate     
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Windows

```shell
python -m venv venv
venv\Scripts\activate  
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Running the checks

For Microsoft Visual Studio first open a x64 Native Tools Command Prompt or call `vcvars64.bat` to
add the C++ compiler toolchain to the environment.

Build with Ninja, and then run the checks:

```shell
cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ninja -C build
python checks/format.py
python checks/static_analysis.py
```
	
## Running

Configuration files are provided in the cfg directory to aid configuring the encoder.  The general pattern of usage is illustrated below, where
multiple configuration files control different aspects of the test conditions.

NB: parameters set in one configuration file override the same parameter in earlier files.  ie. order matters.

Further help text describing option usage is available using "./bin/PccAppEncoder --help" or "./bin/PccAppDecoder --help".

### PccAppEncoder

```
./bin/PccAppEncoder \
	--configurationFolder=cfg/ \
	--config=cfg/common/ctc-common.cfg \
	--config=cfg/condition/ctc-all-intra.cfg \
	--config=cfg/sequence/longdress_vox10.cfg \
	--config=cfg/rate/ctc-r3.cfg \
	--uncompressedDataFolder=~/mpeg_datasets/CfP/datasets/Dynamic_Objects/People/ \
	--frameCount=1 \
	--videoEncoderPath=../external/HM-<version>/bin/TAppEncoderStatic \
	--colorSpaceConversionPath=../external/HDRTools/bin/HDRConvert \
	--reconstructedDataPath=S26C03R03_rec_%04d.ply \
	--compressedStreamPath=S26C03R03.bin 
```

### PccAppDecoder

```
./bin/PccAppDecoder \
	--compressedStreamPath=S26C03R03.bin \
	--videoDecoderPath=../external/HM-<version>/bin/TAppDecoderStatic \
	--colorSpaceConversionPath=../external/HDRTools/bin/HDRConvert \ 
	--inverseColorSpaceConversionConfig=cfg/hdrconvert/yuv420torgb444.cfg \
	--reconstructedDataPath=S26C03R03_dec_%04d.ply 
```

### Scripts

More examples of running could be found in ./test/runme_linux.sh. 

These examples can be start based on your system with the following scripts:
- ./test/runme_linux.sh
- ./test/runme_windows.bat
- ./test/runme_osx.sh

The V3C common test condition (CTC) command lines could be found in ./test/ctc_command_line.sh. 

## SHVC Information

The SHVC software used in the program can be obtained from the link below.
	https://hevc.hhi.fraunhofer.de/svn/svn_SHVCSoftware/tags/SHM-12.4/

The additional Enhanced Layer will be used by SHVC codec according to the number of entries entered in the SHVCLayer in the encoder.
SHVCRateX and SHVCRateY refer to the width and height resolution reduction rate of 2D images of additional layers.
The decoder uses the video corresponding to the layer entered into the SHVCLayerID. LID 0 has the lowest density, and if 3 layers are used, LID 2 has the same density as V-PCC TMC2 output.
Occupancy Map video encode/decode using same version of HM encooder/decoder.


### SHVC Running 3layer PccAppEncoder

```
./bin/PccAppEncoder \
	--configurationFolder=cfg/ \
	--config=cfg/common/ctc-common.cfg \
	--config=cfg/condition/ctc-random-access-svc-3L.cfg \
	--config=cfg/sequence/longdress_vox10.cfg \
	--config=cfg/rate/ctc-r3.cfg \
	--uncompressedDataFolder=~/mpeg_datasets/CfP/datasets/Dynamic_Objects/People/ \
	--frameCount=1 \
	--videoEncoderGeometryPath=..\bin\win\TAppEncoder.exe \
	--videoEncoderAttributePath=..\bin\win\TAppEncoder.exe \
 	--videoEncoderOccupancyPath=..\bin\win\occupancy\TAppEncoder.exe \
	--colorSpaceConversionPath=../external/HDRTools/bin/HDRConvert \
	--reconstructedDataPath=S26C03R03_rec_%04d.ply \
	--compressedStreamPath=S26C03R03.bin \
	--SHVCLayer=2 \
	--SHVCRateX=2 \
	--SHVCRateY=2
```

### SHVC Running 3layer PccAppDecoder

```
./bin/PccAppDecoder \
	--compressedStreamPath=S26C03R03.bin \
	--videoDecoderGeometryPath=..\bin\win\TAppDecoder.exe \
	--videoDecoderAttributePath=..\bin\win\TAppDecoder.exe \
	--videoDecoderOccupancyPath=..\bin\win\occupancy\TAppDecoder.exe \
	--colorSpaceConversionPath=../external/HDRTools/bin/HDRConvert \ 
	--inverseColorSpaceConversionConfig=cfg/hdrconvert/yuv420torgb444.cfg \
	--reconstructedDataPath=S26C03R03_dec_%04d.ply \
	--SHVCLayerID=2 
```

### PccGs4AppEncoder

```
./bin/PccGs4AppEncoder \
	--uncompressedDataPath=/path/to/ply_file.ply \
   	--configurationFolder=./mpeg-pcc-tmc2/cfg/gs4/ \
   	--config=./mpeg-pcc-tmc2/cfg/gs4/gs-main10-common.cfg \
	--config=./mpeg-pcc-tmc2/cfg/gs4/gs-pack-none.cfg \
  	--config=./mpeg-pcc-tmc2/cfg/gs4/rate/r04/encoder.cfg \
	--compressedStreamPath=/path/to/compressed_file.v3c \
```

more information you can find [here](source/app/PccGs4AppEncoder/README.md) 

### Software manual

More informations can be found in the [user manuel](doc/mpeg-pcc-tm2-sw-manual.pdf).


###
