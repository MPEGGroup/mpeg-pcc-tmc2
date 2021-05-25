#include "PCCCommon.h"

#ifdef USE_SHMAPP_VIDEO_CODEC

#include "PCCSHMAppVideoDecoder.h"
#include "PccHevcParser.h"
#include "PCCSystem.h"

using namespace pcc;

template <typename T>
PCCSHMAppVideoDecoder<T>::PCCSHMAppVideoDecoder() {}
template <typename T>
PCCSHMAppVideoDecoder<T>::~PCCSHMAppVideoDecoder() {}

template <typename T>
void PCCSHMAppVideoDecoder<T>::decode( PCCVideoBitstream& bitstream,
                                       PCCVideo<T, 3>&    video,
                                       size_t             outputBitDepth,
                                       const std::string& decoderPath,
                                       const std::string& fileName ) {
  if ( decoderPath.empty() || !exist( decoderPath ) ) {
    std::cerr << "decoderPath not set\n";
    exit( 1 );
  }

  size_t                  width = 0, height = 0, bitDepth = 0;
  bool                    isRGB = false;
  pcc_hevc::PccHevcParser hevcParser;
  hevcParser.getVideoSize( bitstream.vector(), width, height, bitDepth, isRGB );
  printf( "hevcParser= %zu x %zu %zu bits isRGB = %d \n", width, height, bitDepth, isRGB );
  if ( true ) { // numLayers > 1 ) {     // SHVC part

    // size_t LID = numLayers - 1;
    // width height change SHVC khu H check
    // if ( SHVCLID < numLayers ) { LID = SHVCLID; }
    // width  = width * std::pow( rateX, LID );
    // height = height * std::pow( rateY, LID );

    const std::string binFileName = fileName + ".bin";
    const std::string reconFile =
        addVideoFormat( fileName + "_rec", width, height, !isRGB, !isRGB, outputBitDepth == 10 ? "10" : "8" );
    bitstream.write( binFileName );
    std::stringstream cmd;
    cmd << decoderPath << " --BitstreamFile=" << binFileName;
    cmd << " --OutpuLayerSetIdx=" << std::to_string( layerIndex_ );
    cmd << " --ReconFile" << std::to_string( layerIndex_ ) << "=" << reconFile;

    if ( isRGB ) {
      cmd << " --OutputColourSpaceConvert=GBRtoRGB";
    } else {
      if ( outputBitDepth == 8 ) {
        cmd << " --OutputBitDepth" << std::to_string( layerIndex_ ) << "=8 --OutputBitDepthC" << std::to_string( layerIndex_ ) << "=8";
      }
    }
    std::cout << cmd.str() << '\n';
    if ( pcc::system( cmd.str().c_str() ) ) {
      std::cout << "Error: can't run system command!" << std::endl;
      exit( -1 );
    }
    PCCCOLORFORMAT format = isRGB ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV420;
    video.clear();
    video.read( reconFile, width, height, format, outputBitDepth == 8 ? 1 : 2 );
    // printf( "SHVC video LID %zu layer is loaded in %zu layers\n", LID, numLayers );
    printf( "File read size = %zu x %zu frame count = %zu \n", video.getWidth(), video.getHeight(),
            video.getFrameCount() );

    removeFile( binFileName );
    removeFile( reconFile );
  } else {  // HEVC part for occupancy map
    const std::string binFileName = fileName + ".bin";
    const std::string reconFile =
        addVideoFormat( fileName + "_rec", width, height, !isRGB, !isRGB, outputBitDepth == 10 ? "10" : "8" );
    bitstream.write( binFileName );
    std::stringstream cmd;
    cmd << decoderPath << " --BitstreamFile=" << binFileName << " --ReconFile=" << reconFile;
    if ( isRGB ) {
      cmd << " --OutputColourSpaceConvert=GBRtoRGB";
    } else {
      if ( outputBitDepth == 8 ) { cmd << " --OutputBitDepth=8 --OutputBitDepthC=8"; }
    }
    std::cout << cmd.str() << '\n';
    if ( pcc::system( cmd.str().c_str() ) ) {
      std::cout << "Error: can't run system command!" << std::endl;
      exit( -1 );
    }
    PCCCOLORFORMAT format = isRGB ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV420;
    video.clear();
    video.read( reconFile, width, height, format, outputBitDepth == 8 ? 1 : 2 );
    printf( "File read size = %zu x %zu frame count = %zu \n", video.getWidth(), video.getHeight(),
            video.getFrameCount() );

    removeFile( binFileName );
    removeFile( reconFile );
  }
}

template class pcc::PCCSHMAppVideoDecoder<uint8_t>;
template class pcc::PCCSHMAppVideoDecoder<uint16_t>;

#endif