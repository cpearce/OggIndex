#ifndef __SKELETON_DECODER_HPP__
#define __SKELETON_DECODER_HPP__

#include <vector>
#include <map>
#include "bytes_io.h"
#include "Decoder.hpp"

using namespace std;

class SkeletonDecoder : public Decoder {
public:

  SkeletonDecoder(ogg_uint32_t serial);
  virtual ~SkeletonDecoder() {}

  virtual const char* TypeStr() { return "S"; }  

  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    return -1;
  }

  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      ogg_int64_t& time)
  {
    return ReadHeader(packet);
  }

  virtual bool GotAllHeaders() {
    return mGotAllHeaders;
  } 

  virtual bool ReadHeader(ogg_packet* packet);
  vector<ogg_packet*> mPackets;

  map<ogg_uint32_t, vector<KeyFrameInfo>*> mIndex;

private:
  bool mGotAllHeaders;

  bool DecodeIndex(ogg_packet* packet);
  
};

#endif // __SKELETON_DECODER_HPP__