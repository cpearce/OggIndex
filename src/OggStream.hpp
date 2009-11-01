/*
   Copyright (C) 2009, Mozilla Foundation

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   - Neither the name of the Mozilla Foundation nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
   PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE ORGANISATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * OggStream.hpp - Encapsulates a track in an ogg file.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __OGGSTREAM_HPP__
#define __OGGSTREAM_HPP__

#include <vector>
#include <map>
#include <ogg/ogg.h>
#include "Decoder.hpp"
#include "FisboneInfo.hpp"
#include "KeyFrameInfo.hpp"


using namespace std;

class OggStream
{
public:
  OggStream(ogg_uint32_t serial);
  OggStream() : mDecoder(0) {}
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

  FisboneInfo GetFisboneInfo() {
    return mDecoder ? mDecoder->GetFisboneInfo() : FisboneInfo();
  }

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
