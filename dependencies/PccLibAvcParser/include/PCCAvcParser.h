#ifndef _PCC_AVC_PARSER_H_
#define _PCC_AVC_PARSER_H_

#include "CommonDef.h"
#include "TComSlice_avc.h"
#include "TDecCAVLC_avc.h"


class PCCAvcNalu {
 public:
  PCCAvcNalu(){ data_.clear(); }
  ~PCCAvcNalu(){ data_.clear(); }

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

class PCCAvcFrame {
 public:
  PCCAvcFrame(){  nalu_.clear(); }
  ~PCCAvcFrame(){ nalu_.clear(); }
  void add( PCCAvcNalu& nalu ) {  nalu_.push_back( nalu ); }
  const std::vector<PCCAvcNalu>& getNalu() const { return nalu_; }
  const PCCAvcNalu&              getNalu( size_t frameIndex ) const { return nalu_[frameIndex]; }
  const size_t                    getFrameCount() { return nalu_.size(); }
private:
  std::vector<PCCAvcNalu> nalu_;
};

class PCCAvcParser {
 public:
  PCCAvcParser();
  ~PCCAvcParser();
  void getVideoSize( const std::vector<uint8_t>& buffer, size_t& width, size_t& height, int CodecId );
  void display();
  const char* getNaluType( int iNaluType );

 private:
  void setBuffer( const std::vector<uint8_t>& buffer, size_t& width, size_t& height, int CodecId );
  //void createNalu( const std::vector<uint8_t>& buffer, const size_t pos, const size_t size );
  void createNalu( const size_t frameIndex,
                   const std::vector<uint8_t>& buffer,
                   const size_t pos,
                   const size_t size );

  const std::vector<PCCAvcNalu>& getVps () const { return vps_; }
  const std::vector<PCCAvcNalu>& getSps () const { return sps_; }
  const std::vector<PCCAvcNalu>& getPps () const { return pps_; }
  std::vector<PCCAvcNalu>  vps_;
  std::vector<PCCAvcNalu>  sps_;
  std::vector<PCCAvcNalu>  pps_;
  std::vector<PCCAvcFrame> frames_;

};
#endif //~_PCC_Avc_PARSER_H_
