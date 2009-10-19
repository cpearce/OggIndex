#ifndef __OGGSTREAM_HPP__
#define __OGGSTREAM_HPP__

#include <vector>
#include <map>
#include <ogg/ogg.h>
#include "Decoder.hpp"

using namespace std;

class OggStream
{
public:
  OggStream(ogg_uint32_t serial);
  ~OggStream();
  
  ogg_uint32_t mSerial;
  ogg_stream_state mState;
  ogg_int32_t mPacketCount;
  StreamType mType;
  
  ogg_int64_t GetStartTime() {
    return mDecoder ? mDecoder->GetStartTime() : -1;
  }
  
  ogg_int64_t GetEndTime() {
    return mDecoder ? mDecoder->GetEndTime() : -1;
  }
  
  ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    return mDecoder ? mDecoder->GranuleposToTime(granulepos) : -1;
  }
  
  bool GotAllHeaders() { return mDecoder ? mDecoder->GotAllHeaders() : false; }
  
  bool Decode(ogg_page* page,
              ogg_int64_t pageOffset);

  const char* TypeStr() { return mDecoder ? mDecoder->TypeStr() : "?"; }

  vector<KeyFrameInfo> mKeyframes;

  Decoder* mDecoder;

private:

  ogg_uint32_t mSpanningPageChecksum;
  ogg_int64_t mSpanningPageOffset;
  ogg_int64_t mStartTime;
  ogg_int64_t mEndTime;
  bool mPacketSpanningPage;
  bool mGotStartTime;

  // Returns true if the keyframe at offset/frameTime is enough after the
  // previous keyframe for us to consider indexing it.
  bool IsAfterThreshold(ogg_int64_t offset, ogg_int64_t frameTime);
  
  bool IsKeyframeVectorSorted();

  bool ReadHeaders(ogg_packet* packet,
                   ogg_int64_t pageOffset);
};



typedef map<ogg_uint32_t, OggStream*> StreamMap;

// Convert stream map of serialno->stream to a vector of streams. Easier to
// work with vectors...
vector<OggStream*>
GetStreamVector(StreamMap& m);
#endif