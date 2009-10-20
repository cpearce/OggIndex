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
 * Decoder.hpp - Abstract class for an OggStream's decoder.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __DECODER_HPP__
#define __DECODER_HPP__

#include <ogg/ogg.h>
#include "FisboneInfo.hpp"

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

  virtual FisboneInfo GetFisboneInfo() = 0;

protected:
  ogg_uint32_t mSerial;
  ogg_int64_t mStartTime;
  ogg_int64_t mEndTime;
  bool mGotStartTime;
};


#endif // __DECODER_HPP__
