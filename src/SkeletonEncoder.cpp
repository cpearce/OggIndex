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
 * SkeletonEncoder.cpp - Encodes skeleton track.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */


#include <fstream>
#include <iostream>
#include <algorithm>
#include <assert.h>
#include <time.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include "SkeletonEncoder.hpp"
#include "Options.hpp"
#include "Utils.hpp"
#include "Decoder.hpp"

using namespace std;


#define SKELETON_3_0_HEADER_LENGTH 64
#define SKELETON_3_2_HEADER_LENGTH 112
#define FISBONE_MAGIC "fisbone"
#define FISBONE_MAGIC_LEN (sizeof(FISBONE_MAGIC) / sizeof(FISBONE_MAGIC[0]))
#define FISBONE_BASE_SIZE 52
#define FISBONE_MESSAGE_HEADER_OFFSET 44


static bool
IsIndexable(Decoder* decoder) {
  return decoder->Type() == TYPE_VORBIS ||
         decoder->Type() == TYPE_THEORA;
}

static bool
IsUniqueSerialno(ogg_uint32_t serialno, vector<Decoder*>& decoders)
{
  for (unsigned i=0; i<decoders.size(); i++) {
    if (decoders[i]->GetSerial() == serialno) {
      return false;
    }
  }
  return true;
}

static ogg_uint32_t
GetUniqueSerialNo(vector<Decoder*>& decoders)
{
  ogg_uint32_t serialno;
  srand((unsigned)time(0));
  do {
    serialno = rand();
  } while (!IsUniqueSerialno(serialno, decoders));
  return serialno;
}

SkeletonEncoder::SkeletonEncoder(DecoderMap& decoders,
                                 ogg_int64_t fileLength,
                                 ogg_int64_t oldSkeletonLength,
                                 ogg_uint64_t contentOffset)
  : mSkeletonDecoder(0),
    mFileLength(fileLength),
    mOldSkeletonLength(oldSkeletonLength),
    mPacketCount(0),
    mContentOffset(contentOffset)
{
  DecoderMap::iterator itr = decoders.begin();
  while (itr != decoders.end()) {
    Decoder* d = itr->second;
    if (IsIndexable(d)) {
      mDecoders.push_back(d);
    }
    if (d->Type() == TYPE_SKELETON) {
      mSkeletonDecoder = (SkeletonDecoder*)d;
    }
    itr++;
  }
  mSerial = mSkeletonDecoder ? mSkeletonDecoder->GetSerial()
                             : GetUniqueSerialNo(mDecoders);
}

SkeletonEncoder::~SkeletonEncoder() {
  for (ogg_uint32_t i=0; i<mIndexPackets.size(); i++) {
    delete[] mIndexPackets[i]->packet;
    delete mIndexPackets[i];
  }
  ClearIndexPages();
}


static ogg_int64_t
GetMinStartTime(vector<Decoder*>& streams) 
{
  ogg_int64_t m = LLONG_MAX;
  for (ogg_uint32_t i=0; i<streams.size(); i++) {
    ogg_int64_t t = streams[i]->GetStartTime();
    if (t < m && t > -1) {
      m = t;
    }
  }
  return m;
}

static ogg_int64_t
GetMaxEndtime(vector<Decoder*>& streams)
{
  ogg_int64_t m = LLONG_MIN;
  for (ogg_uint32_t i=0; i<streams.size(); i++) {
    ogg_int64_t t = streams[i]->GetEndTime();
    if (t > m && t > -1) {
      m = t;
    }
  }
  return m;
}

void
SkeletonEncoder::AddBosPacket()
{
  // Should not call this more than once, this should be the first packet.
  assert(mPacketCount == 0);

  // Allocate packet to return...
  ogg_packet* bos = new ogg_packet();
  memset(bos, 0, sizeof(ogg_packet));
  bos->packet = new unsigned char[SKELETON_3_2_HEADER_LENGTH];
  memset(bos->packet, 0, SKELETON_3_2_HEADER_LENGTH);
  bos->bytes = SKELETON_3_2_HEADER_LENGTH;
  bos->b_o_s = 1;
  
  if (mSkeletonDecoder) {
    // We already have a skeleton track. Use what data we can from its
    // bos packet.
    ogg_packet* original = mSkeletonDecoder->mPackets.front();
    memcpy(bos->packet, original->packet, SKELETON_3_0_HEADER_LENGTH);
  } else {
    // We need to construct the skeleton bos packet...
    memcpy(bos->packet, "fishead", 8);
    
    WriteLEUint64(bos->packet + SKELETON_PRES_TIME_DENOM_OFFSET, 1000);
    WriteLEUint64(bos->packet + SKELETON_BASE_TIME_DENOM_OFFSET, 1000);
  }
  
  // Set the version fields.
  WriteLEUint16(bos->packet + SKELETON_VERSION_MAJOR_OFFSET, SKELETON_VERSION_MAJOR);
  WriteLEUint16(bos->packet + SKELETON_VERSION_MINOR_OFFSET, SKELETON_VERSION_MINOR);
  
  // Write start time and end time.
  WriteLEInt64(bos->packet + SKELETON_FIRST_NUMER_OFFSET, GetMinStartTime(mDecoders));
  WriteLEInt64(bos->packet + SKELETON_FIRST_DENOM_OFFSET, 1000);
 
  WriteLEInt64(bos->packet + SKELETON_LAST_NUMER_OFFSET, GetMaxEndtime(mDecoders));
  WriteLEInt64(bos->packet + SKELETON_LAST_DENOM_OFFSET, 1000);
  
  WriteLEUint64(bos->packet + SKELETON_CONTENT_OFFSET, mContentOffset);

  mPacketCount++;

  mIndexPackets.push_back(bos);

}

bool
SkeletonEncoder::Encode() {
  AddBosPacket();

  AddFisbonePackets();

  // Construct and store the index packets.
  ConstructIndexPackets();
  
  AddEosPacket();
  
  // Convert packets to pages. We now know how much space they take up when
  // stored as pages. This tells us how much extra data we're adding to the
  // file. Stores the result in mExtraLength.
  ConstructPages();

  // Adjust index packets' page offsets to account for extra file length.
  CorrectOffsets();
  
  // Reconstruct index pages, their contents have changed because page
  // offsets change when we add stuff to the start of the file.
  ConstructPages();


  return true;
}

void
SkeletonEncoder::AddEosPacket() {
  ogg_packet* eos = new ogg_packet();
  memset(eos, 0, sizeof(ogg_packet));
  eos->e_o_s = 1;
  eos->packetno = mPacketCount;
  mPacketCount++;
  mIndexPackets.push_back(eos);
}

void
SkeletonEncoder::ConstructIndexPackets() {
  assert(mIndexPackets.size() > 0);
  for (ogg_uint32_t i=0; i<mDecoders.size(); i++) {
    ogg_packet* packet = new ogg_packet();
    memset(packet, 0, sizeof(ogg_packet));
    
    const vector<KeyFrameInfo>& keyframes = mDecoders[i]->GetKeyframes();
    
    const ogg_int32_t packetSize = INDEX_KEYPOINT_OFFSET +
                                  (int)keyframes.size() * KEY_POINT_SIZE;
    packet->bytes = packetSize;
    unsigned char* p = new unsigned char[packetSize];
    memset(p, 0, packetSize);
    packet->packet = p;

    // Identifier bytes.
    memcpy(packet->packet, HEADER_MAGIC, HEADER_MAGIC_LEN);

    // Stream serialno.
    WriteLEUint32(packet->packet + INDEX_SERIALNO_OFFSET,
                  mDecoders[i]->GetSerial());
    
    // Number of key points.
    assert(keyframes.size() < UINT_MAX);
    WriteLEUint64(packet->packet + INDEX_NUM_KEYPOINTS_OFFSET,
                  (ogg_uint64_t)keyframes.size());
    
    // Timestamp denominator.
    WriteLEInt64(packet->packet + INDEX_TIME_DENOM_OFFSET, 1000);

    p = packet->packet + INDEX_KEYPOINT_OFFSET;
    for (ogg_uint32_t i=0; i<keyframes.size(); i++) {
      const KeyFrameInfo& k = keyframes[i];
      p = WriteLEUint64(p, k.mOffset);
      p = WriteLEUint32(p, k.mChecksum);
      p = WriteLEUint64(p, k.mTime);
    }
    
    assert(p == packet->packet + packetSize);

    packet->packetno = mPacketCount;
    mPacketCount++;

    assert(packet->e_o_s == 0);
    assert(packet->b_o_s == 0);
    mIndexPackets.push_back(packet);
  }
}


void
SkeletonEncoder::ConstructPages() {
  
  assert(mIndexPackets.size() == 2 * mDecoders.size() + 2);
  
  ClearIndexPages();

  ogg_int32_t ret = 0;
  ogg_page page;
  ogg_stream_state state;
  memset(&state, 0, sizeof(ogg_stream_state));
  memset(&page, 0, sizeof(ogg_page));
  
  ret = ogg_stream_init(&state, mSerial);
  assert(ret == 0);

  // BOS packet, must be on own page.
  ret = ogg_stream_packetin(&state, mIndexPackets[0]);
  assert(ret == 0);
  ret = ogg_stream_flush(&state, &page);
  assert(ret != 0);
  AppendPage(page);

  // Normal skeleton header packets...
  for (ogg_uint32_t i=1; i<mIndexPackets.size(); i++) {
    ret = ogg_stream_packetin(&state, mIndexPackets[i]);
    assert(ret == 0);
  }
  
  while (ogg_stream_pageout(&state, &page) != 0) {
    assert(!ogg_page_bos(&page));
    AppendPage(page);
  }

  ret = ogg_stream_flush(&state, &page);
  if (ret != 0) {
    AppendPage(page);
  }
   
  ogg_stream_clear(&state); 

}

void
SkeletonEncoder::AppendPage(ogg_page& page) {
  ogg_page* clone = Clone(&page);
  mIndexPages.push_back(clone);
}

void
SkeletonEncoder::ClearIndexPages() {
  for (ogg_uint32_t i=0; i<mIndexPages.size(); i++) {
    FreeClone(mIndexPages[i]);
  }
  mIndexPages.clear();
}  

ogg_int64_t
SkeletonEncoder::GetTrackLength()
{
  ogg_int64_t length = 0;
  int packets = 0;
  for (ogg_uint32_t i=0; i<mIndexPages.size(); i++) {
    length += mIndexPages[i]->header_len + mIndexPages[i]->body_len;
    packets += ogg_page_packets(mIndexPages[i]);
  }
  return length;
}

void
SkeletonEncoder::CorrectOffsets() {
  assert(mIndexPackets.size() != 0);
  ogg_int64_t fileLength = mFileLength - mOldSkeletonLength + GetTrackLength();
  cout << "I think indexed file length is " << fileLength << endl;

  // First correct the BOS packet's segment length field.
  WriteLEUint64(mIndexPackets[0]->packet + SKELETON_FILE_LENGTH_OFFSET, fileLength);

  // Difference in file lengths before and after indexing. We must add this
  // amount to the page offsets in the index packets, as they've changed by
  // this much.
  ogg_int64_t lengthDiff = fileLength - mFileLength;
  assert(lengthDiff == (GetTrackLength() - mOldSkeletonLength));
  mContentOffset += lengthDiff;

  // Correct the BOS packet's content offset field.
  WriteLEUint64(mIndexPackets[0]->packet + SKELETON_CONTENT_OFFSET, mContentOffset);

  for (ogg_uint32_t idx=0; idx<mIndexPackets.size(); idx++) {
    ogg_packet* packet = mIndexPackets[idx];
    assert(packet);
    
    if (!IsIndexPacket(packet)) {
      continue;
    }
    
    ogg_uint64_t n = LEUint64(packet->packet + INDEX_NUM_KEYPOINTS_OFFSET);
    assert(n == ((packet->bytes - INDEX_KEYPOINT_OFFSET) / KEY_POINT_SIZE));
    assert(n >= 0);
    unsigned char* p = packet->packet + INDEX_KEYPOINT_OFFSET;
    for (ogg_uint64_t i=0; i<n && (p < packet->packet + packet->bytes); i++) {
      ogg_uint64_t o = LEUint64(p);
      o += lengthDiff;
      assert((ogg_int64_t)o < fileLength);
      WriteLEUint64(p, o);
      p += KEY_POINT_SIZE;
    }
  }
}

// Write out the new skeleton BOS page.
void
SkeletonEncoder::WriteBosPage(ofstream& output) {
  assert(mIndexPages.size() > 0);

  // Write out the new skeleton page.
  WritePage(output, *mIndexPages[0]);
}

void
SkeletonEncoder::WritePages(ofstream& output) {
  assert(mIndexPages.size() > 0);
  for (ogg_uint32_t i=1; i<mIndexPages.size(); i++) {
    WritePage(output, *mIndexPages[i]);
  }
}

bool
SkeletonEncoder::HasFisbonePackets() {
  return mSkeletonDecoder &&
         mSkeletonDecoder->mPackets.size() == mDecoders.size() + 2;
}

void
SkeletonEncoder::AddFisbonePackets() {
  if (HasFisbonePackets()) {
    // We have fisbone packets from the existing skeleton track. Use them.
    for (ogg_uint32_t i=1; i<mSkeletonDecoder->mPackets.size()-1; i++) {
      mIndexPackets.push_back(Clone(mSkeletonDecoder->mPackets[i]));
      mPacketCount++;
    }
  } else {
    // Have to construct fisbone packets.
    for (ogg_uint32_t i=0; i<mDecoders.size(); i++) {
      FisboneInfo info = mDecoders[i]->GetFisboneInfo();
      unsigned packetSize = FISBONE_BASE_SIZE + (unsigned)info.mContentType.size();

      ogg_packet* packet = new ogg_packet();
      memset(packet, 0, sizeof(ogg_packet));
      
      packet->packet = new unsigned char[packetSize];
      memset(packet->packet, 0, packetSize);

      // Magic bytes identifier.
      memcpy(packet->packet, FISBONE_MAGIC, FISBONE_MAGIC_LEN);
      
      // Offset of the message header fields.
      WriteLEInt32(packet->packet+8, FISBONE_MESSAGE_HEADER_OFFSET);
      
      // Serialno of the stream.
      WriteLEUint32(packet->packet+12, mDecoders[i]->GetSerial());
      
      // Number of header packets. 3 for both vorbis and theora.
      WriteLEUint32(packet->packet+16, 3);
      
      // Granulrate numerator.
      WriteLEInt64(packet->packet+20, info.mGranNumer);
      
      // Granulrate denominator.
      WriteLEInt64(packet->packet+28, info.mGranDenom);
      
      // Start granule.
      WriteLEInt64(packet->packet+36, 0);
      
      // Preroll.
      WriteLEUint32(packet->packet+44, info.mPreroll);
      
      // Granule shift.
      WriteLEUint32(packet->packet+48, info.mGranuleShift);

      // Message header field, Content-Type */
      memcpy(packet->packet+FISBONE_BASE_SIZE,
             info.mContentType.c_str(),
             info.mContentType.size());

      packet->b_o_s = 0;
      packet->e_o_s = 0;
      packet->bytes = packetSize;

      packet->packetno = mPacketCount;      
      mIndexPackets.push_back(packet);
      mPacketCount++;
    }        
  }
}
