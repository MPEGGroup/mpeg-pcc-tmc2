#ifndef __PCCSHMAppVideoEncoder_H__
#define __PCCSHMAppVideoEncoder_H__

#include "PCCCommon.h"
#include "PCCVideo.h"
#include "PCCVirtualVideoEncoder.h"

#ifdef USE_SHMAPP_VIDEO_CODEC

namespace pcc {

template <class T>
class PCCSHMAppVideoEncoder : public PCCVirtualVideoEncoder<T> {
 public:
  PCCSHMAppVideoEncoder();
  ~PCCSHMAppVideoEncoder();

  void encode( PCCVideo<T, 3>&            videoSrc,
               PCCVideoEncoderParameters& params,
               PCCVideoBitstream&         bitstream,
               PCCVideo<T, 3>&            videoRec );

 private:
  PCCCOLORFORMAT getColorFormat( std::string& name );
};

}  // namespace pcc

#endif  //~USE_SHMAPP_VIDEO_CODEC

#endif  //~__PCCSHMAppVideoEncoder_H__
