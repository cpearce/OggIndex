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
 *   ogg.k.ogg.k <ogg.k.ogg.k@googlemail.com>
 */

#include <assert.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <ogg/ogg.h>
#include <theora/theora.h>
#include <theora/theoradec.h>
#include <vorbis/codec.h>
#ifdef HAVE_KATE
#include <kate/oggkate.h>
#endif
#include "Options.hpp"
#include "Utils.hpp"
#include "SkeletonEncoder.hpp"

// Need to index keyframe if we've not seen 1 in 64K.
#define MIN_KEYFRAME_OFFSET (64 * 1024)

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

class TheoraDecoder : public Decoder {
protected:

  th_info mInfo;
  th_comment mComment;
  th_setup_info *mSetup;
  th_dec_ctx* mCtx;

  ogg_int32_t mHeadersRead;
  ogg_int64_t mPacketCount; 

  bool mSetFirstGranulepos;

public:

  ogg_int64_t mNextKeyframeThreshold; // in ms
  ogg_int64_t mGranulepos;

  // Records the packetno of the first complete non-header packet in the stream.
  ogg_int64_t mFirstPacketno;

  TheoraDecoder(ogg_uint32_t serial) :
    Decoder(serial),
    mSetup(0),
    mCtx(0),
    mHeadersRead(0),
    mPacketCount(0),
    mSetFirstGranulepos(false),
    mNextKeyframeThreshold(-INT_MAX),
    mGranulepos(-1),
    mFirstPacketno(-1)
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
    
    // Number of packets that start on this page.
    int num_packets;
  };

  struct Frame {
    Frame(ogg_packet& packet) :
      packetno(packet.packetno),
      granulepos(packet.granulepos),
      is_keyframe(th_packet_iskeyframe(&packet) != 0),
      e_o_s(packet.e_o_s != 0) {}
    ogg_int64_t packetno;
    ogg_int64_t granulepos;
    bool is_keyframe;
    bool e_o_s;
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

    if (mKeyFrames.size() > 0) {
      return mKeyFrames;
    }

    // Packetno of the last packet which has started.
    ogg_int64_t started_packetno = mFirstPacketno - 1;
    // Must at least have header packets.
    assert(started_packetno >= 2);
    ogg_int64_t pageno = 0;
    ogg_int64_t prev_keyframe_pageno = -INT_MAX;
    ogg_int64_t prev_keyframe_start_time = -INT_MAX;
    for (unsigned f=0; f<mFrames.size(); f++) {
      Frame& frame = mFrames[f];
      ogg_int64_t packetno = frame.packetno;

      while (pageno < mPages.size() &&
             started_packetno + mPages[pageno].num_packets < packetno)
      {
        started_packetno += mPages[pageno].num_packets;
        pageno++;  
      } // pages
      assert(pageno < mPages.size());
      assert(started_packetno < packetno &&
             started_packetno + mPages[pageno].num_packets >= packetno);

      if (pageno == prev_keyframe_pageno) {
        // Only consider the pages' first key point.
        continue;
      }

      KeyFrameInfo k(mPages[pageno].offset,
                     StartTime(frame.granulepos));

      // Only add the keyframe to the list if it's far enough after the
      // previous keyframe.
      if (k.mTime > prev_keyframe_start_time + gOptions.GetKeyPointInterval()) {
        prev_keyframe_start_time = k.mTime;
        mKeyFrames.push_back(k);
      }
      prev_keyframe_pageno = pageno;

    } // for each frame

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

  const char* TheoraHeaderType(ogg_packet* packet) {
    switch (packet->packet[0]) {
      case 0x80: return "Ident";
      case 0x81: return "Comment";
      case 0x82: return "Setup";
      default: return "UNKNOWN";
    }
  }

  bool Decode(ogg_page* page, ogg_int64_t offset) {
    assert((ogg_uint32_t)ogg_page_serialno(page) == mSerial);
    if (GotAllHeaders()) {
      Page record;
      record.num_packets = CountPacketStarts(page);
      record.offset = offset;
      mPages.push_back(record);
    }
 
    int ret = ogg_stream_pagein(&mState, page);
    ogg_int64_t page_granulepos = ogg_page_granulepos(page);
    assert(ret == 0);

    ogg_packet packet;
    int num_packets = 0;
    while ((ret = ogg_stream_packetout(&mState, &packet)) != 0) {
      if (ret == -1) {
        cerr << "WARNING: Lost sync decoding packets on theora page "
             << mPages.size() << endl;
        continue;
      }
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
        if (gOptions.GetDumpPackets()) {
          cout << "[T] ver="
               << (int)mInfo.version_major << "."
               << (int)mInfo.version_minor << "."
               << (int)mInfo.version_subminor << " "
               << TheoraHeaderType(&packet) << " packet"
               << (packet.e_o_s ? " eos" : "")
               << endl;
        }
        continue;
      }      

      if (mFirstPacketno == -1) {
        mFirstPacketno = packet.packetno;
      }
      
      int shift = mInfo.keyframe_granule_shift;
      if (mGranulepos == -1) {
      
        // Packet should only have a granulepos if the page does.
        assert(page_granulepos != -1 || packet.granulepos == -1);
        assert(packet.granulepos == -1 || packet.granulepos == page_granulepos);
      
        // We've not yet determined the granulepos of the first (or previous)
        // packet. Remember the packet, even if it's not a keyframe so that
        // we can backtrack to get all packets' start time.
        mFrames.push_back(Frame(packet));

        if (packet.granulepos == -1) {
          // We've stored the packet, once we find one with a non -1 granulepos
          // we can find the first packet's granulepos.
          continue;
        }
        
        // We know a packet's granulepos, use it to tag the packets buffered
        // before it so that they have valid granulepos.
        for (int i=(int)mFrames.size()-2; i>=0; i--) {
          ogg_int64_t prev_granulepos = mFrames[i+1].granulepos;
          assert(prev_granulepos != -1);
          ogg_int64_t granulepos = -1;
          if (mFrames[i].is_keyframe) {
            ogg_int64_t frame = th_granule_frame(mCtx, prev_granulepos) +
                                TheoraVersion(&mInfo,3,2,1) - 1;
            // 3.2.0 streams store the frame index in the granule position.
            // 3.2.1 and later store the frame count. th_granule_frame() returns
            // the frame index, so |frame| can be 0 when we're theora version
            // 3.2.0 or less.
            assert(frame > 0 || !TheoraVersion(&mInfo,3,2,1));
            granulepos = frame << shift;
          } else {
            // We must be offset by more than 1 frame for this to work.
            assert((prev_granulepos & ((1 << shift) - 1)) > 0);
            granulepos = prev_granulepos - 1;
          }
          // This frame's granule number should be one less than the previous.
          assert(th_granule_frame(mCtx, granulepos) ==
                 th_granule_frame(mCtx, prev_granulepos) - 1);
          mFrames[i].granulepos = granulepos;
        }
        // Now all packets have a known time.
        assert(mStartTime == -1);
        mStartTime = StartTime(mFrames.front().granulepos);
        assert(mStartTime >= 0);
        mEndTime = EndTime(mFrames.back().granulepos);
        assert(mEndTime >= mStartTime);

        // Print/log packets before we remove non-keyframe packets.
        for (unsigned i=0; i<mFrames.size(); i++) {
          DumpPacket(mFrames[i]);
        }

        // Remove the frames that aren't keyframes.
        for (unsigned i=0; i<mFrames.size(); i++) {
          if (!mFrames[i].is_keyframe) {
            mFrames.erase(mFrames.begin()+i);
            i--;
            continue;
          }
          assert(mFrames[i].is_keyframe);
        }
        mGranulepos = packet.granulepos;
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
        ogg_int64_t max_offset = (1 << shift) - 1;
        assert((mGranulepos & max_offset) + 1 <= max_offset);
      }
      assert(th_granule_frame(mCtx, mGranulepos) + 1 ==
             th_granule_frame(mCtx, granulepos));
      assert(packet.granulepos == -1 || packet.granulepos == granulepos);
      packet.granulepos = granulepos;
      mGranulepos = granulepos;
      
      Frame f(packet);
      DumpPacket(f);
      if (is_keyframe) {
        mFrames.push_back(f);
      }
      mEndTime = EndTime(mGranulepos);
    } // end while packetout.
    if (num_packets != ogg_page_packets(page)) {
      cerr << "WARNING: Fewer packets finished on theora page "
           << mPages.size() << " than expected." << endl;
    }
    return true;
  }

  void DumpPacket(Frame& f) {
    if (gOptions.GetDumpPackets() ||
        (f.is_keyframe && gOptions.GetDumpKeyPackets()))
    {
      cout << "[T] " << (f.is_keyframe ? "keyframe" : "frame")
           << " time_ms=[" << StartTime(f.granulepos) << ","
           << EndTime(f.granulepos) << "] granulepos=" << f.granulepos
           << " packetno=" << f.packetno
           << (f.e_o_s ? " eos" : "")
           << endl;
    }
  }  

  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    return (!GotAllHeaders()) ? -1 : EndTime(granulepos);
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
};

class VorbisDecoder : public Decoder {
public:

  VorbisDecoder(ogg_uint32_t serial) :
    Decoder(serial),
    mNextKeyframeThreshold(-INT_MAX),
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

  const char* VorbisHeaderType(ogg_packet* packet) {
    switch (packet->packet[0]) {
      case 0x1: return "Ident";
      case 0x3: return "Comment";
      case 0x5: return "Setup";
      default: return "UNKNOWN";
    }
  }

  bool Decode(ogg_page* page, ogg_int64_t offset) {
    if (GotAllHeaders()) {
      // Reset the vorbis syntheis. This simulates what happens when we seek
      // to this page and start decoding with no prior knowledge. This forces
      // the decoder to preroll again.
      ogg_stream_reset(&mState);
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
        if (gOptions.GetDumpPackets()) {
          cout << "[V] ver="
               << (int)mInfo.version << " "
               << VorbisHeaderType(&packet) << " packet"
               << (packet.e_o_s ? " eos" : "")
               << endl;
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

        if (gOptions.GetDumpKeyPackets() || gOptions.GetDumpPackets()) {
          cout << "[V] sample time_ms=[" << start_time << "," << end_time
               << "] granulepos=[" << start_granule << ","
               << packet.granulepos << "]"
               << (packet.e_o_s ? " eos" : "")
               << endl;
        }    

        if (start_time > mNextKeyframeThreshold) {
          mKeyFrames.push_back(KeyFrameInfo(offset, start_time));
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
    return (!GotAllHeaders()) ? -1 : Time(granulepos);
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

#ifdef HAVE_KATE
class KateDecoder : public Decoder {
protected:

  kate_info mInfo;
  kate_comment mComment;
  kate_state mCtx;

  int mNumHeaders;
  ogg_int32_t mFirstPacketno;
  ogg_int32_t mHeadersRead;
  ogg_int64_t mPacketCount; 

public:

  ogg_int64_t mNextKeyframeThreshold; // in ms
  ogg_int64_t mGranulepos;

  KateDecoder(ogg_uint32_t serial) :
    Decoder(serial),
    mNumHeaders(-1),
    mFirstPacketno(-1),
    mHeadersRead(0),
    mPacketCount(0),
    mNextKeyframeThreshold(-INT_MAX),
    mGranulepos(-1)
  {
    kate_info_init(&mInfo);
    kate_comment_init(&mComment);
  }

  virtual ~KateDecoder() {
    kate_clear(&mCtx);
    kate_info_clear(&mInfo);
    kate_comment_clear(&mComment);
  }

  virtual StreamType Type() { return TYPE_KATE; }

  virtual const char* TypeStr() { return "K"; }

  virtual bool GotAllHeaders() {
    // Kate has a variable number of header packets, specified in the first header.
    return mNumHeaders >= 1 && mHeadersRead == mNumHeaders;
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
      start(-1),
      duration(-1),
      backlink(-1),
      end(-1),
      e_o_s(packet.e_o_s != 0)
    {
      if (packet.bytes > 0 && (packet.packet[0] == 0x00 || packet.packet[0] == 0x02)) {
        start = LEInt64(packet.packet+1);
        duration = LEInt64(packet.packet+1+8);
        backlink = LEInt64(packet.packet+1+16);
        end = start+duration;
      }
    }
    ogg_int64_t packetno;
    ogg_int64_t granulepos;
    ogg_int64_t start;
    ogg_int64_t duration;
    ogg_int64_t backlink;
    ogg_int64_t end;
    bool e_o_s;
  };
  
  // List of pages in the ogg file. We use this to determine the page which
  // a frame exists in.
  vector<Page> mPages;

  // List of keyframes which are in the file.
  vector<Frame> mFrames;

  // Keypoint info which we'll write into the index packet.
  vector<KeyFrameInfo> mKeyFrames;

  ogg_int64_t GranuleRateToMilliseconds(ogg_int64_t duration) const {
    return duration * 1000 *
           mInfo.gps_denominator / mInfo.gps_numerator;
  }

  ogg_int64_t GetTime(ogg_int64_t granulepos) const {
    ogg_int64_t base = granulepos >> mInfo.granule_shift;
    ogg_int64_t offset = granulepos - (base << mInfo.granule_shift);
    return GranuleRateToMilliseconds(base+offset);
  }

  struct Boundary {
    Boundary(ogg_int64_t p, ogg_int64_t t, bool s): packetno(p), time(t), start(s) {}

    bool operator==(const Boundary &other) const { return time == other.time; }
    bool operator<(const Boundary &other) const {
      if (time < other.time) return true;
      if (time > other.time) return false;
      return start && !other.start;
    }

    ogg_int64_t packetno;
    ogg_int64_t time;
    bool start;
  };

  ogg_int64_t FindBacklink(const std::vector<Frame> &frames, ogg_int64_t t) {
    ogg_int64_t earliest_packetno = -1;
    for (std::vector<Frame>::const_iterator i = frames.begin(), end = frames.end(); i != end; ++i) {
      const Frame &frame = (*i);
      ogg_int64_t time = GetTime(frame.granulepos);
      if (time > t) {
        // the frames are sorted by start time, and we're past the target time, so we'll return
        // this frame (the first to follow the target time), as this is the best frame to seek to
        // for that stream if there are no active events at the target time.
        earliest_packetno = frame.packetno;
        break;
      }

      // if the event is active, it will be the earliest as the frames are sorted by start time
      if (t >= time && frame.end >= 0 && t < GranuleRateToMilliseconds(frame.end)) {
        earliest_packetno = frame.packetno;
        break;
      }
    }
    return earliest_packetno;
  }

  Frame *FindFrame(ogg_int64_t packetno) {
    for (size_t n = 0; n < mFrames.size(); ++n) {
      if (mFrames[n].packetno == packetno) {
        return &mFrames[n];
      }
    }
    return NULL;
  }

  virtual const vector<KeyFrameInfo>& GetKeyframes() {
    ogg_int64_t started_packetno = mFirstPacketno - 1;
    ogg_int64_t pageno = 0, npages = mPages.size();
    ogg_int64_t prev_keyframe_pageno = -INT_MAX;
    ogg_int64_t prev_keyframe_start_time = -INT_MAX;

    // Make a list of all boundaries - start or end of events
    std::vector<Boundary> boundaries;
    boundaries.reserve(mFrames.size()*2);
    for (size_t n = 0; n < mFrames.size(); ++n) {
      ogg_int64_t start = GetTime(mFrames[n].granulepos);
      boundaries.push_back(Boundary(FindBacklink(mFrames, start), start, true));
      if (mFrames[n].end >= 0) {
        ogg_int64_t end = GranuleRateToMilliseconds(mFrames[n].end);
        boundaries.push_back(Boundary(FindBacklink(mFrames, end), end, false));
      }
    }

    // sort boundaries, and eliminate duplicates
    std::sort(boundaries.begin(), boundaries.end());
    std::vector<Boundary>::const_iterator end = std::unique(boundaries.begin(), boundaries.end());

    // iterate through boundaries and work out the appropriate page
    for (std::vector<Boundary>::const_iterator i = boundaries.begin(); i != end; ++i) {
      ogg_int64_t packetno = (*i).packetno;

      if (packetno < 0) {
        continue;
      }

      // Find matching page
      while (pageno < npages &&
             started_packetno + mPages[pageno].num_packets < packetno)
      {
        started_packetno += mPages[pageno].num_packets;
        pageno++;  
      } // pages
      assert(pageno < npages);
      assert(started_packetno < packetno &&
             started_packetno + mPages[pageno].num_packets >= packetno);

      if (pageno == prev_keyframe_pageno) {
        // Only consider the pages' first key point.
        continue;
      }

      KeyFrameInfo k(mPages[pageno].offset,
                     (*i).time);

      // Only add the keyframe to the list if it's far enough after the
      // previous keyframe.
      if (k.mTime >= prev_keyframe_start_time + gOptions.GetKeyPointInterval()) {
        prev_keyframe_start_time = k.mTime;
        mKeyFrames.push_back(k);
      }
      prev_keyframe_pageno = pageno;
    }

    return mKeyFrames;
  }

  const char* KateHeaderType(ogg_packet* packet) {
    switch (packet->packet[0]) {
      case 0x80: return "Ident";
      case 0x81: return "Comment";
      case 0x82: return "Regions";
      case 0x83: return "Styles";
      case 0x84: return "Curves";
      case 0x85: return "Motions";
      case 0x86: return "Palettes";
      case 0x87: return "Bitmaps";
      case 0x88: return "Fonts";
      default: return "UNKNOWN";
    }
  }

  bool Decode(ogg_page* page, ogg_int64_t offset) {
    assert((ogg_uint32_t)ogg_page_serialno(page) == mSerial);
    if (GotAllHeaders()) {
      Page record;
      record.num_packets = CountPacketStarts(page);
      record.offset = offset;
      mPages.push_back(record);
    }
 
    int ret = ogg_stream_pagein(&mState, page);
    assert(ret == 0);

    ogg_packet packet;
    int num_packets = 0;
    while ((ret = ogg_stream_packetout(&mState, &packet)) != 0) {
      if (ret == -1) {
        cerr << "WARNING: Lost sync decoding packets on kate page "
             << mPages.size() << endl;
        continue;
      }
      num_packets++;
      if (!GotAllHeaders()) {
        // Read Headers...
        ret = kate_ogg_decode_headerin(&mInfo,
                                      &mComment,
                                      &packet);
        assert(ret >= 0);
        if (ret >= 0) {
          // Read Kate header.
          mHeadersRead++;

          // if it's the first header, extract the number of headers
          if (packet.packet[0] == 0x80) {
            mNumHeaders = packet.packet[11];
          }
        }
        if (GotAllHeaders()) {
          // Read all headers, setup decoder state.
          int ret = kate_decode_init(&mCtx, &mInfo);
          assert(ret >= 0);
        }
        if (gOptions.GetDumpPackets()) {
          cout << "[K] ver="
               << (int)mInfo.bitstream_version_major << "."
               << (int)mInfo.bitstream_version_minor << " "
               << KateHeaderType(&packet) << " packet"
               << (packet.e_o_s ? " eos" : "")
               << endl;
        }
        continue;
      }

      if (mFirstPacketno < 0) {
        mFirstPacketno = packet.packetno;
      }

      // Get granpos, and deduce current time, and backlink time
      if (packet.granulepos >= 0) {
        Frame f(packet);
        mFrames.push_back(f);

        DumpPacket(f);

        ogg_int64_t base = packet.granulepos >> mInfo.granule_shift;
        ogg_int64_t offset = packet.granulepos - (base << mInfo.granule_shift);
        ogg_int64_t start_time = GranuleRateToMilliseconds(base+offset);

        if (mStartTime == -1) {
          mStartTime = start_time;
        }
        if (mStartTime > mEndTime)
          mEndTime = mStartTime;

        // update end time from data packets
        if (packet.bytes > 0 && (packet.packet[0] == 0x00 || packet.packet[0] == 0x02)) {
          ogg_int64_t start = LEInt64(packet.packet+1);
          ogg_int64_t duration = LEInt64(packet.packet+1+8);
          ogg_int64_t end = GranuleRateToMilliseconds(start+duration);
          if (end > mEndTime)
            mEndTime = end;
        }
      }

    } // end while packetout.

    if (num_packets != ogg_page_packets(page)) {
      cerr << "WARNING: Fewer packets finished on kate page "
           << mPages.size() << " than expected." << endl;
    }
    return true;
  }

  void DumpPacket(Frame& f) {
    if (!gOptions.GetDumpPackets() &&
        !gOptions.GetDumpKeyPackets())
      return;
    cout << "[K] " << "event"
         << " time_ms=[" << GranuleRateToMilliseconds(f.start) << ","
         << GranuleRateToMilliseconds(f.end) << "] granulepos=" << f.granulepos
         << " packetno=" << f.packetno
         << (f.e_o_s ? " eos" : "")
         << endl;
  }  

  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    return (!GotAllHeaders()) ? -1 : (ogg_int64_t)(1000*kate_granule_time(&mInfo, granulepos)+0.5f);
  }

  virtual FisboneInfo GetFisboneInfo() {
    FisboneInfo f;
    f.mGranNumer = mInfo.gps_numerator;
    f.mGranDenom = mInfo.gps_denominator;
    f.mPreroll = 0;
    f.mGranuleShift = mInfo.granule_shift;
    f.mContentType = "Content-Type: application/x-kate\r\n";
    assert(f.mContentType.size() == 34);
    return f;
  }
};
#endif

SkeletonDecoder::SkeletonDecoder(ogg_uint32_t serial) :
  Decoder(serial),
  mGotAllHeaders(0),
  mVersionMajor(0),
  mVersionMinor(0),
  mVersion(0)
{
  for (ogg_uint32_t i=0; i<mPackets.size(); i++) {
    delete[] mPackets[i]->packet;
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

    if (IsIndexPacket(&packet)) {
      assert(!packet.e_o_s);
      if (SKELETON_VERSION(SKELETON_VERSION_MAJOR,SKELETON_VERSION_MINOR) != mVersion) {
        cerr << "WARNING: Encountered an index packet of version " 
             << mVersionMajor << "." << mVersionMinor
             << ". I can only read version "
             << SKELETON_VERSION_MAJOR << "." << SKELETON_VERSION_MINOR 
             << ", so skipping index packet." << endl;
      } else if (!::DecodeIndex(mIndex, &packet)) {
        cerr << "WARNING: Index packet " << packet.packetno << " of stream "
             << ogg_page_serialno(page) << " failed to parse." << endl;
      }
    } else if (IsSkeletonPacket(&packet)) {
      assert(!IsIndexPacket(&packet));
      // Don't record index packets, we'll recompute them.
      mPackets.push_back(Clone(&packet));
    }
    
    // Check if the skeleton version is 3.x, fail otherwise.
    if (IsFisheadPacket(&packet)) {
      mVersionMajor = LEUint16(packet.packet + 8);
      mVersionMinor = LEUint16(packet.packet + 10);
      mVersion = SKELETON_VERSION(mVersionMajor, mVersionMinor);
      if (mVersion < SKELETON_VERSION(3,0) ||
          mVersion >= SKELETON_VERSION(4,0)) { 
        cerr << "FAIL: Skeleton version " << mVersionMajor << "." << mVersionMinor   
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
  if (page->body_len > 8 &&
      strncmp("theora", (const char*)page->body+1, 6) == 0)
  {
    return new TheoraDecoder(serialno);
  } else if (page->body_len > 8 &&
             strncmp("vorbis", (const char*)page->body+1, 6) == 0)
  {
    return new VorbisDecoder(serialno);
#ifdef HAVE_KATE
  } else if (page->body_len > 8 &&
             memcmp("kate\0\0\0", (const char*)page->body+1, 7) == 0)
  {
    return new KateDecoder(serialno);
#endif
  } else if (page->body_len > 8 &&
             strncmp("fishead", (const char*)page->body, 8) == 0)
  {
    return new SkeletonDecoder(serialno);
  }
  return 0;
}

bool DecodeIndex(KeyFrameIndex& index, ogg_packet* packet) {
  assert(IsIndexPacket(packet));
  ogg_uint32_t serialno = LEUint32(packet->packet + INDEX_SERIALNO_OFFSET);
  ogg_int64_t numKeyPoints = LEUint64(packet->packet + INDEX_NUM_KEYPOINTS_OFFSET);
  ogg_int64_t time_denom = LEInt64(packet->packet + INDEX_TIME_DENOM_OFFSET);
  ogg_int64_t time_multiplier = 1000;

  // Ensure that the packet's not smaller or significantly larger than
  // we expect. These cases denote a malicious or invalid num_key_points
  // field.
  ogg_int64_t min_packet_size = INDEX_KEYPOINT_OFFSET + numKeyPoints * MIN_KEY_POINT_SIZE;
  if (packet->bytes < min_packet_size) {
    // Packet is less than the theoretical minimum size. This means that the
    // num_key_points field is too large for the packet to possibly contain as
    // many packets as it claims to, so the num_key_points field is probably
    // malicious. Don't try decoding this file, we may run out of memory.
    cerr << "WARNING: Possibly malicious number of key points reported in index packet." << endl;
    return false;
  }
  ogg_int64_t max_packet_size = INDEX_KEYPOINT_OFFSET + numKeyPoints * MAX_KEY_POINT_SIZE;
  if (max_packet_size > INT_MAX) {
    cerr << "ERROR: I can't handle index sizes greater than 2^32." << endl;
    return false;
  }

  if (time_denom == 0) {
    cerr << "WARNING: Index packet for stream " << serialno
         << " has 0 timestamp denominator." << endl;
    return false;
  }

  vector<KeyFrameInfo>* keypoints = new vector<KeyFrameInfo>();
  keypoints->reserve((ogg_int32_t)numKeyPoints);
    
  /* Read in key points. */
  unsigned char* p = packet->packet + INDEX_KEYPOINT_OFFSET;
  ogg_int64_t offset = 0;
  ogg_int64_t time = 0;
  for (ogg_int64_t i=0; i<numKeyPoints; i++) {
    assert(p < packet->packet + packet->bytes);

    ogg_int64_t offset_delta = 0;
    p = ReadVariableLength(p, &offset_delta);
    offset += offset_delta;

    assert(p < packet->packet + packet->bytes);
    ogg_int64_t time_delta = 0;
    p = ReadVariableLength(p, &time_delta);
    time += time_delta;
    ogg_int64_t time_ms = (time * time_multiplier) / time_denom;    
    
    keypoints->push_back(KeyFrameInfo(offset, time_ms));
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
