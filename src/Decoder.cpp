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
 * Decoder.cpp - Decodes ogg content type.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#include <assert.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <ogg/ogg.h>
#include <theora/theora.h>
#include <theora/theoradec.h>
#include <vorbis/codec.h>
#include "Options.hpp"
#include "Utils.hpp"

// Need to index keyframe if we've not seen 1 in 64K.
#define MIN_KEYFRAME_OFFSET (64 * 1024)



static ogg_uint32_t
Checksum(ogg_page* page)
{
  assert(page != 0);
  assert(page->header != 0);
  assert(page->header_len > 25);
  return LEUint32(page->header + 22);
}


Decoder::Decoder(ogg_uint32_t serial) :
  mSerial(serial),
  mStartTime(-1),
  mEndTime(-1)
{
  int ret = ogg_stream_init(&mState, mSerial);
  assert(ret == 0);
}

Decoder::~Decoder() {
  ogg_stream_clear(&mState);
}

// Returns the number of packets that start on a page.
static int
CountPacketStarts(ogg_page* page)
{
  int i;
  // If we're not continuing a packet, we're at a packet start.
  int packets_start = (page->header[5]&0x01) ? 0 : 1; 
  int num_lacing_vals = page->header[26];
  unsigned char* lacing_vals = &page->header[27];
  for (i=1; i<num_lacing_vals; i++) {
    if (lacing_vals[i-1] < 0xff)
      packets_start++;
  }
  return packets_start;
}


class TheoraDecoder : public Decoder { 
public:

  ogg_int64_t mNextKeyframeThreshold; // in ms
  ogg_int64_t mGranulepos;

  TheoraDecoder(ogg_uint32_t serial) :
    Decoder(serial),
    mSetup(0),
    mCtx(0),
    mHeadersRead(0),
    mSetFirstGranulepos(false),
    mPacketCount(0),
    mGranulepos(-1),
    mNextKeyframeThreshold(-INT_MAX)
  {
    th_info_init(&mInfo);
    th_comment_init(&mComment);
  }

  virtual ~TheoraDecoder() {
    th_setup_free(mSetup);
    th_decode_free(mCtx);
  }

  virtual StreamType Type() { return TYPE_THEORA; }

  virtual const char* TypeStr() { return "T"; }

  virtual bool GotAllHeaders() {
    // Theora has 3 header packets, itentification, comment and setup.
    return mHeadersRead == 3;
  }

  struct Page {
    // Offset of page in bytes.
    ogg_int64_t offset;
    
    // Checksum of this page.
    ogg_uint32_t checksum;
    
    // Number of packets that start on this page.
    int num_packets; 
  };

  struct Frame {
    Frame(ogg_packet& packet) :
      packetno(packet.packetno),
      granulepos(packet.granulepos),
      is_keyframe(th_packet_iskeyframe(&packet) != 0) {}
    ogg_int64_t packetno;
    ogg_int64_t granulepos;
    bool is_keyframe;
  };
  
  // List of pages in the ogg file. We use this to determine the page which
  // a frame exists in.
  vector<Page> mPages;

  // List of keyframes which are in the file.
  vector<Frame> mFrames;

  // Keypoint info which we'll write into the index packet.
  vector<KeyFrameInfo> mKeyFrames;
    
  virtual const vector<KeyFrameInfo>& GetKeyframes() {
    // Construct list of keyframes from page and frame info lists.
    // Need to determine frame start offsets and fill key points array.
    ogg_int64_t packet_start_count = 3; // Account for header packets
    unsigned pageno = 0;
    for (unsigned f=0; f<mFrames.size(); f++) {
      Frame& frame = mFrames[f];
      ogg_int64_t packetno = frame.packetno;
      while (pageno < mPages.size() && 
             packet_start_count + mPages[pageno].num_packets < packetno)
      {
        packet_start_count += mPages[pageno].num_packets;
        pageno++;  
      } // pages
      assert(pageno < mPages.size());
      KeyFrameInfo k(mPages[pageno].offset,
                     StartTime(frame.granulepos),
                     mPages[pageno].checksum);
      mKeyFrames.push_back(k);
    }// frames
    return mKeyFrames;
  }

  ogg_int64_t StartTime(ogg_int64_t granulepos) {
    return (th_granule_frame(mCtx, granulepos)) * 1000 *
            mInfo.fps_denominator / mInfo.fps_numerator;
  }

  ogg_int64_t EndTime(ogg_int64_t granulepos) {
    return (th_granule_frame(mCtx, granulepos) + 1) * 1000 *
           mInfo.fps_denominator / mInfo.fps_numerator;
  }

  bool Decode(ogg_page* page, ogg_int64_t offset) {
    if (GotAllHeaders()) {
      Page record;
      record.checksum = Checksum(page);
      record.num_packets = CountPacketStarts(page);
      record.offset = offset;
      mPages.push_back(record);
    }
 
    int ret = ogg_stream_pagein(&mState, page);
    ogg_int64_t page_granulepos = ogg_page_granulepos(page);
    assert(ret == 0);

    ogg_packet packet;
    int num_packets = 0;
    while (ogg_stream_packetout(&mState, &packet) == 1) {
      num_packets++;
      if (!GotAllHeaders()) {
        // Read Headers...
        ret = th_decode_headerin(&mInfo,
                                 &mComment,
                                 &mSetup,
                                 &packet);
        assert(ret > 0);
        if (ret > 0) {
          // Read Theora header.
          mHeadersRead++;
        }
        if (GotAllHeaders()) {
          // Read all headers, setup decoder context.
          mCtx = th_decode_alloc(&mInfo, mSetup);
          assert(mCtx != NULL);
        }
        continue;
      }      
      
      int shift = mInfo.keyframe_granule_shift;
      if (mGranulepos == -1) {
      
        // Packet should only have a granulepos if the page does.
        assert(page_granulepos != -1 || packet.granulepos == -1);
      
        // We've not yet determined the granulepos of the first (or previous)
        // packet. Remember the packet, even if it's not a keyframe so that
        // we can backtrack to get all packets' start time.
        mFrames.push_back(Frame(packet));
        
        if (packet.granulepos != -1) {
          for (int i=(int)mFrames.size()-2; i>=0; i--) {
            ogg_int64_t prev_granulepos = mFrames[i+1].granulepos;
            assert(prev_granulepos != -1);
            ogg_int64_t granulepos = -1;
            if (mFrames[i].is_keyframe) {
              // Note th_granule_frame() returns the frame index (e.g. frame
              // 1's index is 0) so we don't need to decrement it, as
              // th_granule_frame() effectively does that for us.
              ogg_int64_t frame = th_granule_frame(mCtx, prev_granulepos) - 1;
              granulepos = frame << shift;
            } else {
              granulepos = prev_granulepos - 1;
            }
            // This frame's granule number should be one less than the previous.
            assert(th_granule_frame(mCtx, granulepos) + 1 ==
                   th_granule_frame(mCtx, prev_granulepos));
            mFrames[i].granulepos = granulepos;
          }
          // Now all packets have a known time.
          assert(mStartTime == -1);
          mStartTime = StartTime(mFrames[0].granulepos);
          assert(mStartTime >= 0);
          mEndTime = EndTime(mFrames[mFrames.size()-1].granulepos);
          assert(mEndTime >= mStartTime);

          // Remove the frames that aren't keyframes.
          ogg_int64_t prev_keyframe = -INT_MAX;
          for (unsigned i=0; i<mFrames.size(); i++) {
            if (!mFrames[i].is_keyframe ||
                EndTime(mFrames[i].granulepos) < prev_keyframe + MIN_KEYFRAME_OFFSET)
            {
              mFrames.erase(mFrames.begin()+i);
              i--;
              continue;
            }
            assert(mFrames[i].is_keyframe);
            prev_keyframe = EndTime(mFrames[i].granulepos);
          }
          mGranulepos = packet.granulepos;
        }
        continue;
      }

      // mGranulepos != -1. We know the previous packet's granulepos.
      // This packet's granulepos is the previous one's incremented.
      ogg_int64_t granulepos = 0;
      bool is_keyframe = th_packet_iskeyframe(&packet) != 0;
      if (is_keyframe) {
        granulepos = (th_granule_frame(mCtx, mGranulepos) + 1 +
                      TheoraVersion(&mInfo,3,2,1)) << shift;
      } else {
        granulepos = mGranulepos + 1;
      }
      assert(th_granule_frame(mCtx, mGranulepos) + 1 ==
             th_granule_frame(mCtx, granulepos));
      assert(packet.granulepos == -1 || packet.granulepos == granulepos);
      packet.granulepos = granulepos;
      mGranulepos = granulepos;
      
      if (is_keyframe && EndTime(granulepos) > mNextKeyframeThreshold) {
        mFrames.push_back(Frame(packet));
        mNextKeyframeThreshold = EndTime(granulepos) +
                                 gOptions.GetKeyPointInterval();
      }
      mEndTime = EndTime(mGranulepos);
    } // end while packetout.
    assert(num_packets == ogg_page_packets(page));
    
    return true;
  }


  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    assert(GotAllHeaders());
    return EndTime(granulepos);
  }

  virtual FisboneInfo GetFisboneInfo() {
    FisboneInfo f;
    f.mGranNumer = mInfo.fps_numerator;
    f.mGranDenom = mInfo.fps_denominator;
    f.mPreroll = 0;
    f.mGranuleShift = mInfo.keyframe_granule_shift;
    f.mContentType = "Content-Type: video/theora\r\n";
    assert(f.mContentType.size() == 28);
    return f;
  }

protected:

  th_info mInfo;
  th_comment mComment;
  th_setup_info *mSetup;
  th_dec_ctx* mCtx;

  ogg_int32_t mHeadersRead;
  ogg_int64_t mPacketCount; 

  bool mSetFirstGranulepos;
};

class VorbisDecoder : public Decoder {
public:

  VorbisDecoder(ogg_uint32_t serial) :
    Decoder(serial),
    mHeadersRead(0)
  {
    vorbis_info_init(&mInfo);
    vorbis_comment_init(&mComment);    
  }

  virtual ~VorbisDecoder() {}

  virtual const char* TypeStr() { return "V"; }  
  virtual StreamType Type() { return TYPE_VORBIS; }

  ogg_int64_t mNextKeyframeThreshold; // in ms

  vector<KeyFrameInfo> mKeyFrames;

  virtual const vector<KeyFrameInfo>& GetKeyframes() {
    return mKeyFrames;
  }

  ogg_int64_t Time(ogg_int64_t granulepos) {
    assert(GotAllHeaders());
    return (1000 * granulepos) / mDsp.vi->rate;
  }

  bool Decode(ogg_page* page, ogg_int64_t offset) {
    assert(!ogg_page_continued(page));

    ogg_stream_reset(&mState);
    if (GotAllHeaders()) {
      // Reset the vorbis syntheis. This simulates what happens when we seek
      // to this page and start decoding with no prior knowledge. This forces
      // the decoder to preroll again.
      vorbis_synthesis_restart(&mDsp);
    }

    ogg_packet packet;
    assert(ogg_stream_packetout(&mState, &packet) == 0);
    int ret = ogg_stream_pagein(&mState, page);
    assert(ret == 0);
    int total_samples = 0;
    ogg_int64_t start_time = -1;

    while ((ret = ogg_stream_packetout(&mState, &packet)) == 1) {

      if (!GotAllHeaders()) {
        ogg_int32_t ret = vorbis_synthesis_headerin(&mInfo, &mComment, &packet);
        if (ret == 0) {
          mHeadersRead++;
        }
        if (GotAllHeaders()) {
          ret = vorbis_synthesis_init(&mDsp, &mInfo);
          assert(ret == 0);
          ret = vorbis_block_init(&mDsp, &mBlock);
          assert(ret == 0);
        }
        continue;
      }
      assert(GotAllHeaders());

      // Decode page, get start and end time.
      // We only expect the last packet in a page to have non -1 granulepos.
      assert(start_time == -1);
      int samples = 0;
      
      // Decode the vorbis to determine how many samples are in each packet.
      if (vorbis_synthesis(&mBlock, &packet) == 0) {
        ret = vorbis_synthesis_blockin(&mDsp, &mBlock);
        assert(ret == 0);
      }
      while ((samples = vorbis_synthesis_pcmout(&mDsp, 0)) > 0) {
        total_samples += samples;
        ret = vorbis_synthesis_read(&mDsp, samples);
        assert(ret == 0);
      }

      if (packet.granulepos != -1) {
        assert(packet.granulepos == ogg_page_granulepos(page));
        ogg_int64_t start_granule = packet.granulepos - total_samples;
        start_time = Time(start_granule);
        ogg_int64_t end_time = Time(packet.granulepos);
        // First packet will be included in the index, the cut off threshold
        // is then set relative to that.

        if (start_time > mNextKeyframeThreshold) {
          mKeyFrames.push_back(KeyFrameInfo(offset, start_time, Checksum(page)));
          mNextKeyframeThreshold = Time(ogg_page_granulepos(page)) +
                                   gOptions.GetKeyPointInterval();
        }
        assert(ogg_stream_packetout(&mState, &packet) == 0);
        
        if (mStartTime == -1) {
          mStartTime = start_time;
        }
        mEndTime = end_time;
      }
    } // while packetout

    return true;
  } // Decode()

  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    assert(GotAllHeaders());
    return Time(granulepos);
  }

  virtual bool GotAllHeaders() {
    // Vorbis has exactly 3 header packets, identification, comment and setup.
    return mHeadersRead == 3;
  } 

  virtual FisboneInfo GetFisboneInfo() {
    FisboneInfo f;
    f.mGranNumer = mInfo.channels * mInfo.rate;
    f.mGranDenom = 1;
    f.mPreroll = 2;
    f.mGranuleShift = 0;
    f.mContentType = "Content-Type: audio/vorbis\r\n";
    assert(f.mContentType.size() == 28);
    return f;
  }  

private:
  ogg_int32_t mHeadersRead;
  
  vorbis_info mInfo;
  vorbis_comment mComment;
  vorbis_dsp_state mDsp;
  vorbis_block mBlock;
};


SkeletonDecoder::SkeletonDecoder(ogg_uint32_t serial) :
  Decoder(serial),
  mGotAllHeaders(0)    
{
  for (ogg_uint32_t i=0; i<mPackets.size(); i++) {
    delete mPackets[i]->packet;
    mPackets[i]->packet = 0;
    delete mPackets[i];
  }
  ClearKeyframeIndex(mIndex);
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

bool SkeletonDecoder::Decode(ogg_page* page, ogg_int64_t offset) {
  int ret = ogg_stream_pagein(&mState, page);
  ogg_int64_t granulepos = ogg_page_granulepos(page);
  assert(ret == 0);

  ogg_packet packet;
  int num_packets = 0;
  while (true) {

    ret = ogg_stream_packetout(&mState, &packet);
    if (ret == 0) {
      // We need another page to decode more packets.
      return true;
    }
    if (ret == -1) {
      // Some kind of error?
      return true;
    }
    assert(ret == 1);
    num_packets++;

    if (!IsSkeletonPacket(&packet)) {
      return false;
    }

    if (IsIndexPacket(&packet)) {
      assert(!packet.e_o_s);
      return ::DecodeIndex(mIndex, &packet);
    } else {
      assert(!IsIndexPacket(&packet));
      // Don't record index packets, we'll recompute them.
      mPackets.push_back(Clone(&packet));
    }
    
    // Check if the skeleton version is 3.0+, fail otherwise.
    if (IsFisheadPacket(&packet)) {
      ogg_uint16_t ver_maj = LEUint16(packet.packet + 8);
      ogg_uint16_t ver_min = LEUint16(packet.packet + 10);
      ogg_uint32_t version = SKELETON_VERSION(ver_maj, ver_min);
      if (version < SKELETON_VERSION(3,0) ||
          version >= SKELETON_VERSION(4,0)) { 
        cerr << "FAIL: Skeleton version " << ver_maj << "." <<ver_min   
             << " detected. I can only handle version 3.x" << endl;
        exit(-1);
      }
    }
    
    // We've read all headers when we receive the EOS packet.
    if (packet.e_o_s) {
      mGotAllHeaders = true;
      return true;
    }
  }
  return true;
}

Decoder* Decoder::Create(ogg_page* page)
{
  assert(ogg_page_bos(page));
  ogg_uint32_t serialno = ogg_page_serialno(page);
  Decoder* decoder = 0;
  if (page->body_len > 8 &&
      strncmp("theora", (const char*)page->body+1, 6) == 0)
  {
    return new TheoraDecoder(serialno);
  } else if (page->body_len > 8 &&
             strncmp("vorbis", (const char*)page->body+1, 6) == 0)
  {
    return new VorbisDecoder(serialno);
  } else if (page->body_len > 8 &&
             strncmp("fishead", (const char*)page->body, 8) == 0)
  {
    return new SkeletonDecoder(serialno);
  }
  return 0;
}

bool DecodeIndex(KeyFrameIndex& index, ogg_packet* packet) {
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
  
  index[serialno] = keypoints;
  
  assert(index[serialno] == keypoints);
  
  return true;
}

void ClearKeyframeIndex(KeyFrameIndex& index) {
  KeyFrameIndex::iterator itr = index.begin();
  while (itr != index.end()) {
    vector<KeyFrameInfo>* v = itr->second;
    delete v;
    itr++;
  }
  index.clear();
}