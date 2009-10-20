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
 * SkeletonDecoder.hpp - Decodes skeleton track.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __SKELETON_DECODER_HPP__
#define __SKELETON_DECODER_HPP__

#include <vector>
#include <map>
#include "bytes_io.h"
#include "Decoder.hpp"
#include "KeyFrameInfo.hpp"

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

  virtual FisboneInfo GetFisboneInfo() { return FisboneInfo(); }

private:
  bool mGotAllHeaders;

  bool DecodeIndex(ogg_packet* packet);
  
};

#endif // __SKELETON_DECODER_HPP__