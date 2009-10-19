#ifndef __DECODER_HPP__
#define __DECODER_HPP__

#include <ogg/ogg.h>
#include "OggIndex.h"

class KeyFrameInfo {
public:
  KeyFrameInfo() : mOffset(0), mTime(-1), mChecksum(0) {}

  KeyFrameInfo(ogg_uint64_t offset, ogg_int64_t time, ogg_uint32_t checksum) :
    mOffset(offset),
    mTime(time),
    mChecksum(checksum) {}
  ogg_int64_t mOffset; // In bytes from beginning of file.
  ogg_int64_t mTime; // In milliseconds.
  ogg_uint32_t mChecksum;

  bool operator<(const KeyFrameInfo& other) {
    return other.mOffset < mOffset;
  }
};


class Decoder {
public:
  Decoder(ogg_uint32_t serial) :
    mSerial(serial),
    mStartTime(-1),
    mEndTime(-1),
    mGotStartTime(false) {}

  virtual ~Decoder() {}

  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      ogg_int64_t& time) = 0;

  virtual bool ReadHeader(ogg_packet* packet) = 0;

  virtual bool GotAllHeaders() = 0;
  
  ogg_int64_t GetStartTime() { return mStartTime; }
  ogg_int64_t GetEndTime() { return mEndTime; }
  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) = 0;

  virtual const char* TypeStr() { return "?"; }
  
  ogg_uint32_t GetSerial() { return mSerial; }

protected:
  ogg_uint32_t mSerial;
  ogg_int64_t mStartTime;
  ogg_int64_t mEndTime;
  bool mGotStartTime;
};


#endif // __DECODER_HPP__
