#include <iostream>
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
  
  // TODO: Check if the skeleton version is 3.0 or 3.1, fail otherwise.

  // We've read all headers when we receive the EOS packet.
  if (packet->e_o_s) {
    mGotAllHeaders = true;
    return true;
  }
  
  return true;
}

