#include <fstream>
#include <iostream>
#include <algorithm>
#include <assert.h>
#include "SkeletonEncoder.hpp"
#include "Options.hpp"
#include "Utils.h"

using namespace std;


static bool
IsIndexable(OggStream* stream) {
  return stream->mType == TYPE_VORBIS ||
         stream->mType == TYPE_THEORA;
}


SkeletonEncoder::SkeletonEncoder(vector<OggStream*>& streams,
                                 ogg_int64_t fileLength,
                                 ogg_int64_t oldSkeletonLength)
  : mFileLength(fileLength),
    mOldSkeletonLength(oldSkeletonLength),
    mPacketCount(0),
    mSkeletonDecoder(0)
{
  for (ogg_uint32_t i=0; i<streams.size(); i++) {
    if (IsIndexable(streams[i])) {
      mStreams.push_back(streams[i]);
    }
    if (streams[i]->mType == TYPE_SKELETON) {
      mSkeletonDecoder = (SkeletonDecoder*)streams[i]->mDecoder;
    }
  }
  mSerial = mSkeletonDecoder->GetSerial();
}

SkeletonEncoder::~SkeletonEncoder() {
  for (ogg_uint32_t i=0; i<mIndexPackets.size(); i++) {
    delete mIndexPackets[i]->packet;
    delete mIndexPackets[i];
  }
  ClearIndexPages();
}


static ogg_int64_t
GetMinStartTime(vector<OggStream*>& streams) 
{
  ogg_int64_t m = INT64_MAX;
  for (ogg_uint32_t i=0; i<streams.size(); i++) {
    ogg_int64_t t = streams[i]->GetStartTime();
    if (t < m && t > -1) {
      m = t;
    }
  }
  return m;
}

static ogg_int64_t
GetMaxEndtime(vector<OggStream*>& streams)
{
  ogg_int64_t m = INT64_MIN;
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
  bos->packet = new unsigned char[SKELETON_3_1_HEADER_LENGTH];
  memset(bos->packet, 0, SKELETON_3_1_HEADER_LENGTH);
  bos->bytes = SKELETON_3_1_HEADER_LENGTH;
  bos->b_o_s = 1;
  
  if (mSkeletonDecoder) {
    // We already have a skeleton track. Use what data we can from its
    // bos packet.
    ogg_packet* original = mSkeletonDecoder->mPackets.front();
    memcpy(bos->packet, original->packet, SKELETON_3_0_HEADER_LENGTH);
  } else {
    // We need to construct the skeleton bos packet...
    WriteLEUint64(bos->packet+20, 1000); // "prestime" denom.
    WriteLEUint64(bos->packet+36, 1000); // basetime denom.
  }
  
  // Set the version fields to 3.1.
  WriteLEUint16(bos->packet+8, 3);
  WriteLEUint16(bos->packet+10, 1);
  
  // Write start time and end time.
  WriteLEInt64(bos->packet + 64, GetMinStartTime(mStreams));
  WriteLEInt64(bos->packet + 72, GetMaxEndtime(mStreams));
  
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

// Compare function, used to sort index vector of packets.
// returns true if a is before b.
static bool
compare_index_packet(ogg_packet* a, ogg_packet* b)
{
  ogg_uint32_t aSerial = LEUint32(a->packet);
  ogg_uint32_t bSerial = LEUint32(b->packet);
  return aSerial < bSerial;
}

static bool
IsSorted(vector<ogg_packet*>& indexes)
{
  for (ogg_uint32_t i=1; i<indexes.size(); i++) {
    ogg_packet* prev = indexes[i-1];
    ogg_packet* here = indexes[i];
    ogg_uint32_t prevSerial = LEUint32(prev->packet + HEADER_MAGIC_LEN);
    ogg_uint32_t hereSerial = LEUint32(here->packet + HEADER_MAGIC_LEN);
    assert(prevSerial < hereSerial);
    if (prevSerial >= hereSerial)
      return false;
  }
  return true;
}

void
SkeletonEncoder::ConstructIndexPackets() {
  assert(mIndexPackets.size() > 0);
  for (ogg_uint32_t i=0; i<mStreams.size(); i++) {
    ogg_packet* packet = new ogg_packet();
    memset(packet, 0, sizeof(ogg_packet));
    
    vector<KeyFrameInfo>& keyframes = mStreams[i]->mKeyframes;
    
    const ogg_int32_t tableSize = HEADER_MAGIC_LEN + 4 + 4 + (int)keyframes.size() * KEY_POINT_SIZE;
    packet->bytes = tableSize;
    unsigned char* p = new unsigned char[tableSize];
    memset(p, 0, tableSize);
    packet->packet = p;

    // Identifier bytes.
    memcpy(p, HEADER_MAGIC, HEADER_MAGIC_LEN);
    p += HEADER_MAGIC_LEN;

    // Stream serialno.
    p = WriteLEUint32(p, mStreams[i]->mSerial);
    
    // Number of key points.
    assert(keyframes.size() < UINT_MAX);
    p = WriteLEUint32(p, (ogg_uint32_t)keyframes.size());

    for (ogg_uint32_t i=0; i<keyframes.size(); i++) {
      KeyFrameInfo& k = keyframes[i];
      p = WriteLEUint64(p, k.mOffset);
      p = WriteLEUint32(p, k.mChecksum);
      p = WriteLEUint64(p, k.mTime);
    }
    
    assert(p == packet->packet + tableSize);

    packet->packetno = mPacketCount;
    mPacketCount++;

    assert(packet->e_o_s == 0);
    assert(packet->b_o_s == 0);
    mIndexPackets.push_back(packet);
  }
  // TODO: sort index skeleton packets.
  //sort(mIndexPackets.begin(), mIndexPackets.end(), compare_index_packet);
  //assert(IsSorted(mIndexPackets));
}


void
SkeletonEncoder::ConstructPages() {
  
  assert(mIndexPackets.size() == 2 * mStreams.size() + 2);
  
  ClearIndexPages();

  ogg_int32_t ret = 0;
  ogg_page page;
  ogg_stream_state state;
  memset(&state, 0, sizeof(ogg_stream_state));
  memset(&page, 0, sizeof(ogg_page));
  ogg_int64_t length = 0;
  
  ret = ogg_stream_init(&state, mSerial);
  assert(ret == 0);

  // BOS packet, must be on own page.
  ret = ogg_stream_packetin(&state, mIndexPackets[0]);
  assert(ret == 0);
  ret = ogg_stream_flush(&state, &page);
  AppendPage(page);

  // Normal skeleton header packets...
  for (ogg_uint32_t i=1; i<mIndexPackets.size(); i++) {
    ret = ogg_stream_packetin(&state, mIndexPackets[i]);
    assert(ret == 0);
  }
  
  while (ogg_stream_pageout(&state, &page) != 0) {
    assert(!ogg_page_bos(&page));
    //assert(!ogg_page_eos(&page));
    AppendPage(page);
  }

  ret = ogg_stream_flush(&state, &page);
  AppendPage(page);
   
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
  for (ogg_uint32_t i=0; i<mIndexPages.size(); i++) {
    length += mIndexPages[i]->body_len + mIndexPages[i]->header_len;
  }
  return length;
}

void
SkeletonEncoder::CorrectOffsets() {
  assert(mIndexPackets.size() != 0);
  ogg_int64_t fileLength = mFileLength - mOldSkeletonLength + GetTrackLength();
  cout << "I think indexed file length is " << fileLength << endl;

  // First correct the BOS packet's segment length field.
  WriteLEUint64(mIndexPackets[0]->packet + 80, fileLength);
  
  // Difference in file lengths before and after indexing. We must add this
  // amount to the page offsets in the index packets, as they've changed by
  // this much.
  ogg_int64_t lengthDiff = fileLength - mFileLength;

  for (ogg_uint32_t idx=0; idx<mIndexPackets.size(); idx++) {
    ogg_packet* packet = mIndexPackets[idx];
    assert(packet);
    
    if (!IsIndexPacket(packet)) {
      continue;
    }
    
    unsigned char* p = packet->packet + HEADER_MAGIC_LEN + 4;
    ogg_int32_t n = LEUint32(p);
    p += 4;
    assert(n == ((packet->bytes - 14) / KEY_POINT_SIZE));
    assert(n >= 0);
    for (ogg_int32_t i=0; i<n; i++) {
      assert(p < packet->packet + packet->bytes);
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
         mSkeletonDecoder->mPackets.size() == mStreams.size() + 2;
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
    // Have to construct fisbone packets. Damn.
  }
}
