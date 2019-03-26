#ifndef _PCC_HEVC_PARSER_H_
#define _PCC_HEVC_PARSER_H_

#include "CommonDef.h"
#include "TypeDef.h"
#include "TComSlice.h"
#include "TDecCAVLC.h"


class PCCHevcNalu {
 public:
  PCCHevcNalu(){ data_.clear(); }
  ~PCCHevcNalu(){ data_.clear(); }

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

class PCCHevcFrame {
 public:
  PCCHevcFrame(){  nalu_.clear(); }
  ~PCCHevcFrame(){ nalu_.clear(); }
  void add( PCCHevcNalu& nalu ) {  nalu_.push_back( nalu ); }
  const std::vector<PCCHevcNalu>& getNalu() const { return nalu_; }
  const PCCHevcNalu&              getNalu( size_t frameIndex ) const { return nalu_[frameIndex]; }
  const size_t                    getFrameCount() { return nalu_.size(); }
private:
  std::vector<PCCHevcNalu> nalu_;
};

class PCCHevcParser {
public:
  PCCHevcParser ();
  ~PCCHevcParser();
  void getVideoSize( const std::vector<uint8_t>& buffer, size_t& width, size_t& height, size_t& bitdepth ); 
  void display();

private:
  void setBuffer ( const std::vector<uint8_t>& buffer,
                   size_t& width, 
                   size_t& height, 
                   size_t& bitdepth );

  void createNalu( const std::vector<uint8_t>& buffer, const size_t pos, const size_t size );
  void createNalu( const int frameIndex,
                   const std::vector<uint8_t>& buffer,
                   const size_t pos,
                   const size_t size );

  const std::vector<PCCHevcNalu>& getVps () const { return vps_; }
  const std::vector<PCCHevcNalu>& getSps () const { return sps_; }
  const std::vector<PCCHevcNalu>& getPps () const { return pps_; }
  std::vector<PCCHevcNalu>  vps_;
  std::vector<PCCHevcNalu>  sps_;
  std::vector<PCCHevcNalu>  pps_;
  std::vector<PCCHevcFrame> frames_;

};
#endif //~_PCC_HEVC_PARSER_H_
