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
 * SkeletonDecoder.cpp - Decodes skeleton track.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */
 
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include "SkeletonDecoder.hpp"
#include "OggIndex.h"
#include "bytes_io.h"
#include "Utils.h"
#include <assert.h>

SkeletonDecoder::SkeletonDecoder(ogg_uint32_t serial) :
  Decoder(serial),
  mGotAllHeaders(0)    
{
  for (ogg_uint32_t i=0; i<mPackets.size(); i++) {
    delete mPackets[i]->packet;
    mPackets[i]->packet = 0;
    delete mPackets[i];
  }
  map<ogg_uint32_t, vector<KeyFrameInfo>*>::iterator itr = mIndex.begin();
  while (itr != mIndex.end()) {
    vector<KeyFrameInfo>* v = itr->second;
    delete v;
    itr++;
  }
  mIndex.clear();
}

static bool
IsSkeletonPacket(ogg_packet* packet)
{
  return (packet->e_o_s && packet->bytes == 0) ||
         (packet->bytes > 8 &&
           (memcmp(packet->packet, "fishead", 8) == 0 ||
            memcmp(packet->packet, "fisbone", 8) == 0)) ||
         IsIndexPacket(packet);
}

bool SkeletonDecoder::DecodeIndex(ogg_packet* packet)
{
  
  assert(IsIndexPacket(packet));
  unsigned char* p = packet->packet + HEADER_MAGIC_LEN;
  ogg_uint32_t serialno = LEUint32(p);
  p += 4;
  ogg_int32_t numKeyPoints = LEUint32(p);
  p += 4;

  // Check that the packet's not smaller or significantly larger than
  // we expect. These cases denote a malicious or invalid num_key_points
  // field.
  ogg_int32_t expectedPacketSize = HEADER_MAGIC_LEN + 8 + numKeyPoints * KEY_POINT_SIZE;
  ogg_int32_t actualNumPackets = (packet->bytes - HEADER_MAGIC_LEN - 8) / KEY_POINT_SIZE;
  assert(((packet->bytes - HEADER_MAGIC_LEN - 8) % KEY_POINT_SIZE) == 0);
  if (packet->bytes < expectedPacketSize ||
      numKeyPoints > actualNumPackets) {
    cerr << "WARNING: Possibly malicious number of keyframes detected in index packet." << endl;
    return false;
  }

  vector<KeyFrameInfo>* keypoints = new vector<KeyFrameInfo>();
  keypoints->reserve(numKeyPoints);
    
  /* Read in key points. */
  assert(p == packet->packet + 14);
  for (ogg_int32_t i=0; i<numKeyPoints; i++) {
    assert(p < packet->packet + packet->bytes);
    ogg_uint64_t offset=0;
    ogg_uint32_t checksum=0;
    ogg_uint64_t time=0;
    
    offset = LEInt64(p);
    p += 8;

    assert(p < packet->packet + packet->bytes);
    checksum = LEUint32(p);
    p += 4;

    assert(p < packet->packet + packet->bytes);
    time = LEInt64(p);
    p += 8;
    
    keypoints->push_back(KeyFrameInfo(offset, time, checksum));
  }
  
  mIndex[serialno] = keypoints;
  
  assert(mIndex[serialno] == keypoints);
  
  return true;
}

bool SkeletonDecoder::ReadHeader(ogg_packet* packet)
{
  if (!IsSkeletonPacket(packet)) {
    return false;
  }

  if (IsIndexPacket(packet)) {
    assert(!packet->e_o_s);
    return DecodeIndex(packet);
  } else {
    assert(!IsIndexPacket(packet));
    // Don't record index packets, we'll recompute them.
    mPackets.push_back(Clone(packet));
  }
  
  // Check if the skeleton version is 3.0+, fail otherwise.
  if (IsFisheadPacket(packet)) {
    ogg_uint16_t ver_maj = LEUint16(packet->packet + 8);
    ogg_uint16_t ver_min = LEUint16(packet->packet + 10);
    ogg_uint32_t version = SKELETON_VERSION(ver_maj, ver_min);
    if (version < SKELETON_VERSION(3,0) ||
        version >= SKELETON_VERSION(4,0)) { 
      cerr << "FAIL: Skeleton version " << ver_maj << "." <<ver_min   
           << " detected. I can only handle version 3.x" << endl;
      exit(-1);
    }
  }
  
  // We've read all headers when we receive the EOS packet.
  if (packet->e_o_s) {
    mGotAllHeaders = true;
    return true;
  }
  
  return true;
}

