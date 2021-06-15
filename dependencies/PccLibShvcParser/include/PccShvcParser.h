#ifndef _PCC_SHVC_PARSER_H_
#define _PCC_SHVC_PARSER_H_

#include "PccShvcCommonDef.h"
#include "PccShvcTComSlice.h"
#include "PccShvcTDecCAVLC.h"

namespace pcc_shvc {

class PccShvcNalu {
 public:
  PccShvcNalu(){ data_.clear(); }
  ~PccShvcNalu(){ data_.clear(); }

  uint8_t* data() { return data_.data(); }
  size_t   size() const { return data_.size(); }

  const size_t getStartCodeLenght() const { return  data_[ 2 ] == 0 ? 4 : 3; }
  const size_t getNaluType       () const { return (   ( data_[ getStartCodeLenght()     ] ) &  126 ) >> 1 ; }
  const size_t getLayer          () const { return ( ( ( data_[ getStartCodeLenght()     ] ) &    1 ) << 6 ) +
                                                   ( ( ( data_[ getStartCodeLenght() + 1 ] ) &  248 ) >> 3 ); }
  const size_t getTemporal       () const { return (   ( data_[ getStartCodeLenght() + 1 ] ) &    7 ) + 1;    }

  void add( const std::vector<uint8_t>& buffer, const size_t pos, const size_t size ){
    data_.clear();
    data_.resize( size );
    memcpy( data_.data(), buffer.data() + pos, size * sizeof( uint8_t ) );
  }
 private:
  std::vector<uint8_t> data_;
};

class PccShvcFrame {
 public:
  PccShvcFrame(){  nalu_.clear(); }
  ~PccShvcFrame(){ nalu_.clear(); }
  void add( PccShvcNalu& nalu ) {  nalu_.push_back( nalu ); }
  std::vector<PccShvcNalu>& getNalu() { return nalu_; }
  PccShvcNalu&              getNalu( size_t frameIndex ) { return nalu_[frameIndex]; }
  const size_t                    getFrameCount() { return nalu_.size(); }
private:
  std::vector<PccShvcNalu> nalu_;
};

class PccShvcParser {
 public:
  PccShvcParser();
  ~PccShvcParser();
  void getVideoSize( const std::vector<uint8_t>& buffer, size_t& width, size_t& height, size_t& bitDepth, bool& is444 );

  void getVideoSize( const std::vector<uint8_t>& buffer,
                     std::vector<size_t>&        width,
                     std::vector<size_t>&        height,
                     std::vector<size_t>&        bitDepth,
                     std::vector<uint8_t>&       is444 );
  void display();

  std::vector<PccShvcNalu>&  getVps() { return vps_; }
  std::vector<PccShvcNalu>&  getSps() { return sps_; }
  std::vector<PccShvcNalu>&  getPps() { return pps_; }
  std::vector<PccShvcFrame>& getFrames() { return frames_; }

 private:
  void createNalu( const std::vector<uint8_t>& buffer, const size_t pos, const size_t size );
  void createNalu( const size_t frameIndex, const std::vector<uint8_t>& buffer, const size_t pos, const size_t size );

  std::vector<PccShvcNalu>  vps_;
  std::vector<PccShvcNalu>  sps_;
  std::vector<PccShvcNalu>  pps_;
  std::vector<PccShvcFrame> frames_;
};

}; // namespace pcc_shvc

#endif //~_PCC_SHVC_PARSER_H_
