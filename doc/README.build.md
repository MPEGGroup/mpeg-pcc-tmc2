Obtaining the software
======================

Clone
---------------
The authoritative location of the software is the following git
repository:
   <http://mpegx.int-evry.fr/software/MPEG/PCC/TM/mpeg-pcc-tmc2>

Each released version may be identified by a version control system tag in
the form `release-v10.0` [1].

An example:

```console 
$ git clone http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-tmc2.git
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

### Scripts

Bash scripts can be use to build mpeg-pcc-dmetric project: build.sh to build solutions and
clear.sh to clean.
 
### Linux

```console
  mkdir build
  cd build
  cmake ..
  make
  ../bin/PccAppEncoder --help
  ../bin/PccAppDecoder --help
  ../bin/PccAppMetrics --help
```

### OSX

```console
  mkdir build
  cd build
  cmake .. -G Xcode
  xcodebuild
  ../bin/PccAppEncoder --help
  ../bin/PccAppDecoder --help
  ../bin/PccAppMetrics --help
```

As an alternative, the generated XCode project may be opened and built from
XCode itself.

### Windows

```console
  md build
  cd build
  cmake .. -G "Visual Studio 15 2017 Win64"
```

Open the generated visual studio solution to build it.


### HM reference software

The common test conditions use HM reference software to encode the created videos. To respect the CTC, we must use the HM: HM-16.20+SCM-8.8 and apply a patch to this version to activate the 3D motion estimation. The patch can be found in the subfolder: mpeg-pcc-tmc2/dependencies/hm-modification/pcc_me-ext_for_HM-16.20+SCM-8.8.patch. 

The  next command lines could be used to download HM reference software and apply patch: 

```console
svn checkout https://hevc.hhi.fraunhofer.de/svn/svn_HEVCSoftware/\\
   tags/HM-16.20+SCM-8.8/external/HM-16.20+SCM-8.8+3DMC;    
cd external/HM-16.20+SCM-8.8+3DMC
svn patch ../../mpeg-pcc-tmc2/dependencies/hm-modification/\\
   pcc_me-ext_for_HM-16.20+SCM-8.8.patch 
```

### HDRTools

The HDRTools is used to perform color convertion. This software must be cloned and built and a path must be set to the PccAppEndoder and PccAppDecoder to perform CTC conditions [2].

```console
git clone -b 0.17-dev https://gitlab.com/standards/HDRTools.git
```

### Metrics

To evaluate the performance of the VPCC encoding, the PCC_distorsion software must be used: 

```console
git clone http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-dmetric.git
```


