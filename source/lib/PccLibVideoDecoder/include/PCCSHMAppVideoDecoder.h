#ifndef PCCSHMAppVideoDecoder_h
#define PCCSHMAppVideoDecoder_h

#include "PCCCommon.h"

#ifdef USE_SHMAPP_VIDEO_CODEC

#include "PCCVideo.h"
#include "PCCVirtualVideoDecoder.h"

namespace pcc {

template <class T>
class PCCSHMAppVideoDecoder : public PCCVirtualVideoDecoder<T> {
 public:
  PCCSHMAppVideoDecoder();
  ~PCCSHMAppVideoDecoder();

  void decode( PCCVideoBitstream& bitstream,
               PCCVideo<T, 3>&    video,
               size_t             outputBitDepth = 8,
               const std::string& decoderPath    = "",
               const std::string& parameters     = "" );

  void setLayerIndex( size_t index ) { layerIndex_ = index; }

 private:
  size_t layerIndex_;  // index of the layer to decode
};

};  // namespace pcc

#endif

#endif /* PCCSHMAppVideoDecoder_h */