#ifndef _PCC_AVC_PARSER_H_
#define _PCC_AVC_PARSER_H_

#include "PccAvcCommonDef.h"
#include "PccAvcTComSlice_avc.h"
#include "PccAvcTDecCAVLC_avc.h"


class PccAvcNalu {
 public:
  PccAvcNalu(){ data_.clear(); }
  ~PccAvcNalu(){ data_.clear(); }

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

class PccAvcFrame {
 public:
  PccAvcFrame() { nalu_.clear(); }
  ~PccAvcFrame() { nalu_.clear(); }
  void                           add( PccAvcNalu& nalu ) { nalu_.push_back( nalu ); }
  const std::vector<PccAvcNalu>& getNalu() const { return nalu_; }
  const PccAvcNalu&              getNalu( size_t frameIndex ) const { return nalu_[frameIndex]; }
  const size_t                   getFrameCount() { return nalu_.size(); }

 private:
  std::vector<PccAvcNalu> nalu_;
};

class PccAvcParser {
 public:
  PccAvcParser();
  ~PccAvcParser();
  void        getVideoSize( const std::vector<uint8_t>& buffer,
                            size_t&                     width,
                            size_t&                     height,
                            int                         isAnnexB,
                            size_t&                     bitDepth,
                            bool&                       is444 );
  void        display();
  const char* getNaluType( int iNaluType );

 private:
  void setBuffer( const std::vector<uint8_t>& buffer,
                  size_t&                     width,
                  size_t&                     height,
                  int                         isAnnexB,
                  size_t&                     bitDepth,
                  bool&                       is444 );
  void createNalu( const size_t frameIndex, const std::vector<uint8_t>& buffer, const size_t pos, const size_t size );

  const std::vector<PccAvcNalu>& getVps () const { return vps_; }
  const std::vector<PccAvcNalu>& getSps () const { return sps_; }
  const std::vector<PccAvcNalu>& getPps () const { return pps_; }
  std::vector<PccAvcNalu>  vps_;
  std::vector<PccAvcNalu>  sps_;
  std::vector<PccAvcNalu>  pps_;
  std::vector<PccAvcFrame> frames_;

};
#endif //~_Pcc_Avc_PARSER_H_
