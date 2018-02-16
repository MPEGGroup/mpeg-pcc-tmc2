# TMC2 

## Building

### OSX
- mkdir build
- cd build
- cmake .. -G Xcode 
- open the generated xcode project and build it

### Linux
- mkdir build
- cd build
- cmake .. 
- make

### Windows
- md build
- cd build
- cmake .. -G "Visual Studio 15 2017 Win64"
- open the generated visual studio solution and build it

## Running

Configuration files are provided in the cfg directory to aid configuring
the encoder.  The general pattern of usage is illustrated below, where
multiple configuration files control different aspects of the test
conditions.

```ShellSession
$ path/to/tmc2 \
    -c cfg/ctc-common.cfg \
    -c cfg/ctc-all-intra.cfg \
    -c cfg/ctc-r1.cfg \
    -c cfg/per-sequence/queen.cfg \
    --uncompressedDataPath=path/toqueen/frame_%04d.ply \
    --compressedStreamPath=output/bitstream/file.bin \
    --videoEncoderPath=path/to/HM-like/encoder \
    --videoDecoderPath=path/to/HM-like/decoder
```

NB: parameters set in one configuration file override the same parameter
in earlier files.  ie. order matters.

Further help text describing option usage is available using "./tmc2 --help".
