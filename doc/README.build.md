Obtaining the software
======================

Clone
---------------
The authoritative location of the software is the following git
repository:
   <http://mpegx.int-evry.fr/software/MPEG/PCC/TM/mpeg-pcc-tmc2>

Each released version may be identified by a version control system tag in
the form `release-v22.1` [1].

An example:

```console 
$ git clone \
   http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-tmc2.git
$ cd mpeg-pcc-tmc2
```

> It is strongly advised to obtain the software using the version control
> system rather than to download a zip (or other archive) of a particular
> release.  The build system uses the version control system to accurately
> identify the version being built.

Building 
------- 

The codec is supported on Linux, OSX and Windows platforms.  The build
configuration is managed using CMake.

> It is strongly advised to build the software in a separate build directory.

### Scripts

Bash scripts can be use to build mpeg-pcc-tmc2 project: 
- build.sh: build solutions.  
- clear.sh: clear solututions ( ./clear.sh all: to clear dependencies)
   
### OSX

```console
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build . --config Release --parallel 8 
```

### Linux

```console
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build . --config Release --parallel 8 
```

### Windows

```console
$ md build
$ cd build
$ cmake ..
$ cmake --build . --config Release --parallel 8 
```

### External dependencies

According to the CMake options defined in the CMakeLists.txt, the TMC2 required some external dependencies to work: 

* USE_JMAPP_VIDEO_CODEC: use JM software to encoder and decoder videos (codecId parameters must be set equal to 0 and the videoEncoderOccupancyPath, videoEncoderGeometryPath and videoEncoderAttributePath but be set the JM applications)
* USE_HMAPP_VIDEO_CODEC: use HM software to encoder and decoder videos (codecId parametesr must be set equal to 1 and the videoEncoderOccupancyPath, videoEncoderGeometryPath and videoEncoderAttributePath but be set the JM applications)
* USE_JMLIB_VIDEO_CODEC: use JM library to encoder and decoder videos (codecId parameter must be set equal to 2)
* USE_HMLIB_VIDEO_CODEC: use HM library to encoder and decoder videos (codecId parameter must be set equal to 3)
* USE_VTMLIB_VIDEO_CODEC: use VTM library to encoder and decoder videos (codecId parameter must be set equal to 4)
* USE_FFMPEG_VIDEO_CODEC: use FFMPEG library to encoder and decoder videos (codecId parameter must be set equal to 5). This mode is only available in the FFMPEG branch. 
* USE_SHMAPP_VIDEO_CODEC: use SHM software to encoder and decoder videos (codecId parametesr must be set equal to 6, the videoEncoderGeometryPath and videoEncoderAttributePath using the SHM application, and the videoEncoderOccupancyPath using the HM application)
* USE_HDRTOOLS: use HDRTools to convert the raw video files.

The video encoder softwares and libraries can be found in the corresponding repositories: 

* JM: https://vcgit.hhi.fraunhofer.de/jct-vc/JM.git
* HM: https://vcgit.hhi.fraunhofer.de/jvet/HM.git
* VTM: https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git
* SHM: https://vcgit.hhi.fraunhofer.de/jvet/SHM.git

Some changes have been made on these libraries to allow to use the three libraries at the same time and to increase the codec efficiencies for the V3C contents. the three codecs must be patch with the files:

* JM: dependencies/jm-modification/PCC_JM.patch
* HM: dependencies/hm-modification/pcc_me-ext_and_namespace_for_HM-16.20+SCM-8.8.patch
* VTM: dependencies/vtm-modification/adaptions_for_vtm_11_2.patch

By default according the the CMake options, the dependencies are cloned and patched by the cmake process. 
 
The external dependencies could be downloaded, built and linked independenly.

JM: 
```console
   $ git clone checkout \
       https://vcgit.hhi.fraunhofer.de/jct-vc/JM.git \
       dependencies/jm19.0_lib
   $ cd dependencies/jm19.0_lib
   $ git patch ../jm-modification/PCC_JM.patch
``` 

HM: 
```console
   $ git clone checkout \
       https://vcgit.hhi.fraunhofer.de/jvet/HM.git \
       dependencies/HM-16.20+SCM-8.8
   $ cd dependencies/HM-16.20+SCM-8.8
   $ git patch ../hm-modification/\
               pcc_me-ext_and_namespace_for_HM-16.20+SCM-8.8.patch
```

VTM: 
```console
   $ git clone checkout \
       https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM.git \
       dependencies/VTM-11.2    
   $ cd dependencies/VTM-11.2
   $ git patch ../vtm-modification/adaptions_for_vtm_11_2.patch
``` 

HDRTools: 
```console
    $ git clone -b 0.17-dev \
       https://gitlab.com/standards/HDRTools.git \
       dependencies/HDRTools      
``` 

The pointcloud metrics can be computed inside the TM2 encoder and decoder according to the input parameters: 

* computeMetrics: Compute metrics
* uncompressedDataPath: Input pointcloud to encode. Multi-frame sequences may be represented by %04i
* normalDataPath:  Input pointcloud to encode. Multi-frame sequences may be represented by %04i
* resolution: Specify the intrinsic resolution
* dropdups: 0(detect), 1(drop), 2(average) subsequent points with same coordinates
* neighborsProc: 0(undefined), 1(average), 2(weighted average), 3(min), 4(max) neighbors with same geometric distance

The computations of the metrics are the same than the distances computed with the pcc_distortion software that can be found in: http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-dmetric.git.





