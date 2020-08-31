#ifndef _PCC_HEVC_PARSER_H_
#define _PCC_HEVC_PARSER_H_

#include "PccHevcCommonDef.h"
#include "PccHevcTComSlice.h"
#include "PccHevcTDecCAVLC.h"

namespace pcc_hevc { 

class PccHevcNalu {
 public:
  PccHevcNalu(){ data_.clear(); }
  ~PccHevcNalu(){ data_.clear(); }

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

class PccHevcFrame {
 public:
  PccHevcFrame(){  nalu_.clear(); }
  ~PccHevcFrame(){ nalu_.clear(); }
  void add( PccHevcNalu& nalu ) {  nalu_.push_back( nalu ); }
  const std::vector<PccHevcNalu>& getNalu() const { return nalu_; }
  const PccHevcNalu&              getNalu( size_t frameIndex ) const { return nalu_[frameIndex]; }
  const size_t                    getFrameCount() { return nalu_.size(); }
private:
  std::vector<PccHevcNalu> nalu_;
};

class PccHevcParser {
 public:
  PccHevcParser();
  ~PccHevcParser();
  void getVideoSize( const std::vector<uint8_t>& buffer, size_t& width, size_t& height );
  void display();

 private:
  void setBuffer( const std::vector<uint8_t>& buffer, size_t& width, size_t& height );
  void createNalu( const std::vector<uint8_t>& buffer, const size_t pos, const size_t size );
  void createNalu( const size_t frameIndex,
                   const std::vector<uint8_t>& buffer,
                   const size_t pos,
                   const size_t size );

  const std::vector<PccHevcNalu>& getVps () const { return vps_; }
  const std::vector<PccHevcNalu>& getSps () const { return sps_; }
  const std::vector<PccHevcNalu>& getPps () const { return pps_; }
  std::vector<PccHevcNalu>  vps_;
  std::vector<PccHevcNalu>  sps_;
  std::vector<PccHevcNalu>  pps_;
  std::vector<PccHevcFrame> frames_;

};

 }; // namespace pcc_hevc

#endif //~_PCC_HEVC_PARSER_H_
