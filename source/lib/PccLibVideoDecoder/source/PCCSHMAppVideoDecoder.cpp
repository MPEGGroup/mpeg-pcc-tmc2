#include "PCCCommon.h"

#ifdef USE_SHMAPP_VIDEO_CODEC

#include "PCCSHMAppVideoDecoder.h"
#include "PccShvcParser.h"
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
  printf( "layerIndex_ = %zu \n", layerIndex_ );

  std::vector<size_t>     width, height, bitDepth;
  std::vector<uint8_t>    isRGB;
  pcc_shvc::PccShvcParser shvcParser;
  shvcParser.getVideoSize( bitstream.vector(), width, height, bitDepth, isRGB );
  layerIndex_ = ( std::min )( width.size() - 1, layerIndex_ );

  printf( "Num Layer = %zu layerIndex = %zu \n", width.size(), layerIndex_ );
  for ( size_t i = 0; i < width.size(); i++ ) {
    printf( "  Layer %zu = %4zux%-4zu %zu bits isRGB = %d \n", i, width[i], height[i], bitDepth[i], isRGB[i] );
  }

  const std::string binFileName = fileName + ".bin";
  const std::string reconFile =
      addVideoFormat( fileName + "_rec", width[layerIndex_], height[layerIndex_], !isRGB[layerIndex_],
                      !isRGB[layerIndex_], outputBitDepth == 10 ? "10" : "8" );
  bitstream.write( binFileName );
  std::stringstream cmd;
  cmd << decoderPath;
  cmd << " --BitstreamFile=" << binFileName;
  cmd << " --ReconFile" << layerIndex_ << "=" << reconFile;
  cmd << " --LayerNum=" << layerIndex_ + 1;
  cmd << " --OutpuLayerSetIdx=" << layerIndex_;
  if ( isRGB[layerIndex_] ) {
    cmd << " --OutputColourSpaceConvert=GBRtoRGB";
  } else {
    if ( outputBitDepth == 8 ) {
      cmd << " --OutputBitDepth" << layerIndex_ << "=8";
      cmd << " --OutputBitDepthC" << layerIndex_ << "=8";
    }
  }
  std::cout << cmd.str() << '\n';
  if ( pcc::system( cmd.str().c_str() ) ) {
    std::cout << "Error: can't run system command!" << std::endl;
    exit( -1 );
  }
  PCCCOLORFORMAT format = isRGB[layerIndex_] ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV420;
  video.clear();
  video.read( reconFile, width[layerIndex_], height[layerIndex_], format, outputBitDepth == 8 ? 1 : 2 );
  printf( "File read size = %zu x %zu frame count = %zu \n", video.getWidth(), video.getHeight(),
          video.getFrameCount() );

  if ( layerIndex_ < width.size() - 1 ) {
    float rateX = (float)width[width.size() - 1] / (float)width[layerIndex_];
    float rateY = (float)height[width.size() - 1] / (float)height[layerIndex_];
    if ( rateX != rateY ) {
      printf( "Error: SHVC upsampler only works with same rate for X and Y. ( %f and %f not supported \n", rateX,
              rateY );
      exit( -1 );
    }
    if ( rateX != 1.f && rateX != 2.f && rateX != 4.f ) {
      printf( "Error: SHVC upsampler only works with rate = 1 , 2, or 4. ( %f not supported \n", rateX );
      exit( -1 );
    }
    video.upsample( rateX );
    printf( "Upsample video size = %zu x %zu frame count = %zu \n ", video.getWidth(), video.getHeight(),
            video.getFrameCount() );
    fflush( stdout );
  }
  removeFile( binFileName );
  removeFile( reconFile );
}

template class pcc::PCCSHMAppVideoDecoder<uint8_t>;
template class pcc::PCCSHMAppVideoDecoder<uint16_t>;

#endif
