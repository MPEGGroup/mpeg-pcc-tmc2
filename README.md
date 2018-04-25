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

An example of running could be find in ./test/runme_linux.sh

NB: parameters set in one configuration file override the same parameter
in earlier files.  ie. order matters.

Further help text describing option usage is available using "./bin/PccAppEncoder --help" or "./bin/PccAppDecoder --help".
