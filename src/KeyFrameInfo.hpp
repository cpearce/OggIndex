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
 * KeyFrameInfo.hpp - Stores a "key point".
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __KEYFRAME_INFO_HPP__
#define __KEYFRAME_INFO_HPP__

#include <map>
#include <vector>
#include <ogg/os_types.h>
#include "OggIndex.h"

using namespace std;

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

typedef map<ogg_uint32_t, vector<KeyFrameInfo>*> KeyFrameIndex;

void ClearKeyframeIndex(KeyFrameIndex& index);

#endif
