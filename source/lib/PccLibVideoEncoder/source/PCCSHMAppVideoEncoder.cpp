
#include "PCCSHMAppVideoEncoder.h"
#include "PCCSystem.h"

#ifdef USE_SHMAPP_VIDEO_CODEC

using namespace pcc;

template <typename T>
PCCSHMAppVideoEncoder<T>::PCCSHMAppVideoEncoder() {}
template <typename T>
PCCSHMAppVideoEncoder<T>::~PCCSHMAppVideoEncoder() {}

std::string replace( std::string string, const std::string& from, const std::string& to ) {
  size_t position = 0;
  while ( ( position = string.find( from, position ) ) != std::string::npos ) {
    string.replace( position, from.length(), to );
    position += to.length();
  }
  return string;
}

template <typename T>
void PCCSHMAppVideoEncoder<T>::encode( PCCVideo<T, 3>&            videoSrc,
                                       PCCVideoEncoderParameters& params,
                                       PCCVideoBitstream&         bitstream,
                                       PCCVideo<T, 3>&            videoRec ) {
  int32_t numLayers = std::stoi( getParameterFromConfigurationFile( params.encoderConfig_, "NumLayers" ) );

  const size_t width      = videoSrc.getWidth();
  const size_t height     = videoSrc.getHeight();
  const size_t frameCount = videoSrc.getFrameCount();
  std::string  srcYuvName = params.srcYuvFileName_;
  std::string  recYuvName = params.recYuvFileName_;
  std::string  binName    = params.binFileName_;
  srcYuvName.insert( srcYuvName.find_last_of( "." ), "_shmapp" );
  recYuvName.insert( recYuvName.find_last_of( "." ), "_shmapp" );
  binName.insert( binName.find_last_of( "." ), "_shmapp" );
  printf( "numLayers = %d LayerIndex = %d \n", numLayers, params.shvcLayerIndex_ );
  fflush( stdout );
  if ( numLayers >= 1 ) {
    // Set Layer size and src and rec raw video names
    std::vector<std::string>    srcYuvFileName, recYuvFileName;
    std::vector<size_t>         widthLayers, heightLayers;
    std::vector<PCCVideo<T, 3>> videoSrcLayers;
    for ( size_t i = 0; i < numLayers; i++ ) {
      if ( i + 1 < numLayers ) {
        widthLayers.push_back( width / ( params.shvcRateX_ * ( numLayers - i - 1 ) ) );
        heightLayers.push_back( height / ( params.shvcRateY_ * ( numLayers - i - 1 ) ) );
        srcYuvFileName.push_back( replace( srcYuvName, stringFormat( "_%dx%d_", width, height ),
                                           stringFormat( "_%dx%d_", widthLayers[i], heightLayers[i] ) ) );
        recYuvFileName.push_back( replace( recYuvName, stringFormat( "_%dx%d_", width, height ),
                                           stringFormat( "_%dx%d_", widthLayers[i], heightLayers[i] ) ) );
      } else {
        widthLayers.push_back( width );
        heightLayers.push_back( height );
        srcYuvFileName.push_back( srcYuvName );
        recYuvFileName.push_back( recYuvName );
      }
    }

    // SHVC khu H video downsampling
    // SHVC khu H using setValue and getValue in YUV420 error channelIndex 1&2
    if ( numLayers >= 1 && params.shvcRateX_ >= 2 && params.shvcRateY_ >= 2 ) {
      std::cout << "SHVC sub video generate" << std::endl;
      for ( size_t i = 0; i < numLayers; i++ ) {
        if ( i + 1 < numLayers ) {
          int32_t        scaleX = params.shvcRateX_ * ( numLayers - i - 1 );
          int32_t        scaleY = params.shvcRateY_ * ( numLayers - i - 1 );
          PCCVideo<T, 3> videoDst;
          videoDst.resize( frameCount );
          for ( size_t j = 0; j < videoDst.getFrameCount(); j++ ) {
            auto& imageSrc = videoSrc.getFrame( j );
            auto& imageDst = videoDst.getFrame( j );
            imageDst.resize( width / scaleX, height / scaleY, imageSrc.getColorFormat() );
            for ( size_t v = 0; v < height; v += scaleY ) {
              for ( size_t u = 0; u < width; u += scaleX ) {
                imageDst.setValue( 0, u / scaleX, v / scaleY, imageSrc.getValue( 0, u, v ) );
                imageDst.setValue( 1, u / scaleX, v / scaleY, imageSrc.getValue( 1, u, v ) );
                imageDst.setValue( 2, u / scaleX, v / scaleY, imageSrc.getValue( 2, u, v ) );
              }
            }
          }
          videoSrcLayers.push_back( videoDst );
          videoDst.write( srcYuvFileName[i], params.inputBitDepth_ == 8 ? 1 : 2 );
        } else {
          videoSrcLayers.push_back( videoSrc );
          videoSrc.write( srcYuvFileName[i], params.inputBitDepth_ == 8 ? 1 : 2 );
        }
      }
    }

    for ( size_t i = 0; i < numLayers; i++ ) {
      printf( "Layer %zu : %4zux%-4zu %4zux%-4zu %s \n", i, widthLayers[i], heightLayers[i],
              videoSrcLayers[i].getWidth(), videoSrcLayers[i].getHeight(), srcYuvFileName[i].c_str() );
      fflush( stdout );
    }

    std::stringstream cmd;
    cmd << params.encoderPath_;
    cmd << " -c " << params.encoderConfig_;
    cmd << " --InputChromaFormat=" << ( params.use444CodecIo_ ? "444" : "420" );
    cmd << " --FramesToBeEncoded=" << frameCount;
    cmd << " --FrameSkip=0";
    cmd << " --BitstreamFile=" << binName;

    for ( size_t i = 0; i < numLayers; i++ ) {
      cmd << " --InputFile" << std::to_string( i ) << "=" << srcYuvFileName[i];
      cmd << " --InputBitDepth" << std::to_string( i ) << "=" << params.inputBitDepth_;
      cmd << " --OutputBitDepth" << std::to_string( i ) << "=" << params.outputBitDepth_;
      cmd << " --FrameRate" << std::to_string( i ) << "=30";
      cmd << " --SourceWidth" << std::to_string( i ) << "=" << widthLayers[i];
      cmd << " --SourceHeight" << std::to_string( i ) << "=" << heightLayers[i];
      cmd << " --ReconFile" << std::to_string( i ) << "=" << recYuvFileName[i];
      cmd << " --QP" << std::to_string( i ) << "=" << params.qp_;
    }
    cmd << " --FrameSkip=0";
    std::cout << cmd.str() << std::endl;

    for ( size_t i = 0; i < numLayers; i++ ) {
      printf( "Write video src layer %zu / %d: size = %4zux%-4zu %s \n", i, numLayers, videoSrcLayers[i].getWidth(),
              videoSrcLayers[i].getHeight(), recYuvFileName[i].c_str() );
      videoSrcLayers[i].write( recYuvFileName[i], params.inputBitDepth_ == 8 ? 1 : 2 );
    }
    if ( pcc::system( cmd.str().c_str() ) ) {
      std::cout << "Error: can't run system command!" << std::endl;
      exit( -1 );
    }
    PCCCOLORFORMAT format = getColorFormat( params.recYuvFileName_ );
    videoRec.clear();
    if ( params.shvcLayerIndex_ < numLayers - 1 ) {
      int32_t index = ( std::min )( numLayers - 1, params.shvcLayerIndex_ );
      printf( "Num Layer = %d layerIndex = %d => layer index = %d \n", numLayers, params.shvcLayerIndex_, index );
      videoRec.read( recYuvFileName[index], widthLayers[index], heightLayers[index], format,
                     params.outputBitDepth_ == 8 ? 1 : 2 );
      float rateX = (float)widthLayers[numLayers - 1] / (float)widthLayers[index];
      videoRec.upsample( rateX );
      printf( "Upsample video size = % zu x % zu frame count = % zu \n ", videoRec.getWidth(), videoRec.getHeight(),
              videoRec.getFrameCount() );
      fflush( stdout );
    } else {
      videoRec.read( recYuvName, width, height, format, params.outputBitDepth_ == 8 ? 1 : 2 );
    }
    bitstream.read( binName );

    for ( size_t i = 0; i < numLayers; i++ ) {
      removeFile( srcYuvFileName[i] );
      removeFile( recYuvFileName[i] );
    }
    removeFile( binName );
  } else {
    std::stringstream cmd;
    cmd << params.encoderPath_;
    cmd << " -c " << params.encoderConfig_;
    cmd << " --InputFile=" << srcYuvName;
    cmd << " --InputBitDepth=" << params.inputBitDepth_;
    cmd << " --InputChromaFormat=" << ( params.use444CodecIo_ ? "444" : "420" );
    cmd << " --OutputBitDepth=" << params.outputBitDepth_;
    cmd << " --OutputBitDepthC=" << params.outputBitDepth_;
    cmd << " --FrameRate=30";
    cmd << " --FrameSkip=0";
    cmd << " --SourceWidth=" << width;
    cmd << " --SourceHeight=" << height;
    cmd << " --ConformanceWindowMode=1 ";
    cmd << " --FramesToBeEncoded=" << frameCount;
    cmd << " --BitstreamFile=" << binName;
    cmd << " --ReconFile=" << recYuvName;
    cmd << " --QP=" << params.qp_;
    if ( params.transquantBypassEnable_ != 0 ) { cmd << " --TransquantBypassEnable=1"; }
    if ( params.cuTransquantBypassFlagForce_ != 0 ) { cmd << " --CUTransquantBypassFlagForce=1"; }
    if ( params.internalBitDepth_ != 0 ) {
      cmd << " --InternalBitDepth=" << params.internalBitDepth_;
      cmd << " --InternalBitDepthC=" << params.internalBitDepth_;
    }
    if ( params.usePccMotionEstimation_ ) {
      cmd << " --UsePccMotionEstimation=1";
      cmd << " --BlockToPatchFile=" << params.blockToPatchFile_;
      cmd << " --OccupancyMapFile=" << params.occupancyMapFile_;
      cmd << " --PatchInfoFile=" << params.patchInfoFile_;
    }
    if ( params.inputColourSpaceConvert_ ) { cmd << " --InputColourSpaceConvert=RGBtoGBR"; }

    std::cout << cmd.str() << std::endl;
    videoSrc.write( srcYuvName, params.inputBitDepth_ == 8 ? 1 : 2 );
    if ( pcc::system( cmd.str().c_str() ) ) {
      std::cout << "Error: can't run system command!" << std::endl;
      exit( -1 );
    }
    PCCCOLORFORMAT format = getColorFormat( params.recYuvFileName_ );
    videoRec.clear();
    videoRec.read( recYuvName, width, height, format, params.outputBitDepth_ == 8 ? 1 : 2 );
    bitstream.read( binName );
    removeFile( srcYuvName );
    removeFile( recYuvName );
    removeFile( binName );
  }
}

template <typename T>
PCCCOLORFORMAT PCCSHMAppVideoEncoder<T>::getColorFormat( std::string& name ) {
  if ( ( name.find( "_p444.rgb" ) ) != std::string::npos ) {
    return PCCCOLORFORMAT::RGB444;
  } else if ( ( name.find( "_p444.yuv" ) ) != std::string::npos ) {
    return PCCCOLORFORMAT::YUV444;
  } else if ( ( name.find( "_p420.yuv" ) ) != std::string::npos ) {
    return PCCCOLORFORMAT::YUV420;
  } else {
    printf( "PCCSHMAppVideoEncoder can't find parameters %s \n", name.c_str() );
    exit( -1 );
  }
  return PCCCOLORFORMAT::UNKNOWN;
}

template class pcc::PCCSHMAppVideoEncoder<uint8_t>;
template class pcc::PCCSHMAppVideoEncoder<uint16_t>;

#endif  //~USE_SHMAPP_VIDEO_CODEC
