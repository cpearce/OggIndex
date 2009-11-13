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
 * OggStream.cpp - Encapsulates a track in an ogg file.
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
#include "OggStream.hpp"
#include "bytes_io.h"
#include "Options.hpp"
#include "SkeletonDecoder.hpp"

// Need to index keyframe if we've not seen 1 in 64K.
#define MIN_KEYFRAME_OFFSET (64 * 1024)

OggStream::OggStream(ogg_uint32_t serial) :
  mSerial(serial),
  mPacketCount(0),
  mType(TYPE_UNSUPPORTED),
  mDecoder(0),
  mPacketSpanningPage(false),
  mSpanningPageOffset(0),
  mSpanningPageChecksum(0),
  mStartTime(0),
  mEndTime(0),
  mGotStartTime(false)
{
  int ret = ogg_stream_init(&mState, serial);
  assert(ret == 0);
  // TODO: Is OOM the only way this can fail?
}

OggStream::~OggStream()
{
  if (mDecoder)
    delete mDecoder;
}

static ogg_uint32_t
GetChecksum(ogg_page* page)
{
  assert(page != 0);
  assert(page->header != 0);
  assert(page->header_len > 25);
  return LEUint32(page->header + 22);
}

static bool
pageHasContinuation(ogg_page* page)
{
  if (!page || page->header_len < 27)
    return false;
  int page_segments = page->header[26];
  assert(page->header_len == page_segments + 27);
  if (page->header_len < page_segments + 26)
    return false;
  return page->header[page_segments + 26] == 0xff;
}

bool
OggStream::Decode(ogg_page* page,
                  ogg_int64_t pageOffset)
{
  ogg_int32_t ret = ogg_stream_pagein(&mState, page);
  ogg_int64_t granulepos = ogg_page_granulepos(page);
  assert(ret == 0);

  int continued = ogg_page_continued(page);
  ogg_int64_t offset = continued ? mSpanningPageOffset : pageOffset;
  ogg_uint32_t checksum = continued ? mSpanningPageChecksum : GetChecksum(page);

  bool hasContinuation = pageHasContinuation(page); 

  if (hasContinuation && !continued) {
    mSpanningPageOffset = pageOffset;
    mSpanningPageChecksum = GetChecksum(page);
  }

  ogg_packet packet; 
  while (true) {
    #if _DEBUG
    memset(&packet, 0, sizeof(ogg_packet));
    #endif
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

    mPacketCount++;
                  
    if (!mDecoder || !mDecoder->GotAllHeaders()) {
      ReadHeaders(&packet, pageOffset);
      if (gOptions.GetDumpPackets()) {
        cout << "[" << (mDecoder ? mDecoder->TypeStr() : "?")
             << "] header packet gp=" << packet.granulepos << "\tpn=" << packet.packetno 
             << "\to=" << pageOffset << "\ts=" << mSerial << endl;
      }
      continue;
    }

    assert(mDecoder != 0);
    bool isKeyFrame = false;
    ogg_int64_t frameTime = 0;
    mDecoder->Decode(&packet, isKeyFrame, frameTime);

    if (gOptions.GetDumpPackets() || 
        (gOptions.GetDumpKeyPackets() && isKeyFrame))
    {
      cout << "[" << (mDecoder ? mDecoder->TypeStr() : "?")
           << "] packet gp=" << packet.granulepos << "\tpn=" << packet.packetno 
           << "\to=" << pageOffset << "\ts=" << mSerial << "\tt=" << frameTime
           << (isKeyFrame ? "\tkeyframe" : "\tinterframe") << endl;
    }

    if (!isKeyFrame) {
      continue;
    }

    #ifdef _DEBUG
    if (offset < INT_MAX) {
      // Verify that there's a page at offset, and that it's the one we think it is.
      ifstream file(gOptions.GetInputFilename().c_str(), ios::in | ios::binary);
      assert(file);
      
      // Verify page at offset.
      file.seekg((ogg_int32_t)offset, ios_base::beg);
      char buf[5];
      file.read(buf, 5);
      assert(file.gcount() == 5);
      assert(strcmp(buf, "OggS") == 0);
      
      // Verify checksum, ensures the page is the one we think it is.
      file.seekg((ogg_int32_t)offset+22, ios_base::beg);
      file.read(buf, 4);
      assert(file.gcount() == 4);
      ogg_uint32_t page_checksum = LEUint32((unsigned char*)buf);
      assert(page_checksum == checksum);
      file.close();
    }
    #endif

    if (IsAfterThreshold(offset, frameTime)) {
      mKeyframes.push_back(KeyFrameInfo(offset, frameTime, checksum));
      assert(IsKeyframeVectorSorted());
    }
  }

  return true;
}

// Returns true if the keyframe at offset/frameTime is enough after the
// previous keyframe for us to consider indexing it.
bool OggStream::IsAfterThreshold(ogg_int64_t offset,
                                 ogg_int64_t frameTime)
{ 
  if (mKeyframes.size() == 0)
    return true;
  KeyFrameInfo& prev = mKeyframes.back();
  return offset > (prev.mOffset + MIN_KEYFRAME_OFFSET) &&
         frameTime > (prev.mTime + gOptions.GetKeyPointInterval());
}

bool
OggStream::IsKeyframeVectorSorted()
{
  const vector<KeyFrameInfo>& K = mKeyframes;
  for (ogg_uint32_t i=1; i<K.size(); i++) {
    if (K[i-1].mOffset >= K[i].mOffset ||
        K[i-1].mTime >= K[i].mTime)
    {
      return false;
    }
  }
  return true;
}

static ogg_int64_t
Granule(ogg_int64_t granulepos, ogg_int32_t granuleshift) {
  if (granulepos == -1) {
    // Assume that -1 means it starts at zero...
    return 1;
  }
  ogg_int64_t iframe = granulepos >> granuleshift;
  ogg_int64_t pframe = granulepos - (iframe << granuleshift);
  ogg_int64_t granule = iframe + pframe;  
  return granule;
}


class TheoraDecoder : public Decoder { 
public:
  TheoraDecoder(ogg_uint32_t serial) :
    Decoder(serial),
    mSetup(0),
    mCtx(0),
    mHeadersRead(0),
    mCurrentFrameInfo(0),
    mSpanFrameInfo(0),    
    mSetFirstGranulepos(false),
    mPacketCount(0)
  {
    th_info_init(&mInfo);
    th_comment_init(&mComment);
  }

  virtual ~TheoraDecoder() {
    th_setup_free(mSetup);
    th_decode_free(mCtx);
  }

  virtual const char* TypeStr() { return "T"; }

  virtual bool GotAllHeaders() {
    // Theora has 3 header packets, itentification, comment and setup.
    return mHeadersRead == 3;
  }

  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    ogg_int64_t g = Granule(granulepos, mInfo.keyframe_granule_shift);
    ogg_int64_t time = 1000 * g * mInfo.fps_denominator / mInfo.fps_numerator;
    return time;
  }
 
  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      ogg_int64_t& time)
  {
    ogg_int32_t ret = 0;
    assert(GotAllHeaders());
    if (!mSetFirstGranulepos) {
      mSetFirstGranulepos = true;
      if (packet->granulepos != -1) {
        ogg_int64_t granule = Granule(packet->granulepos,
                                mInfo.keyframe_granule_shift);
        if (granule != 1) {
          // Non-zero start time of the theora stream. We need to set the
          // granulepos of the next frame to the packet granulepos, else
          // th_decode_packetin() will give a granulepos starting at 0, not
          // the time of the first frame.
          mPacketCount = granule;
        }
      }
    }
    ogg_int64_t granulepos = mPacketCount << mInfo.keyframe_granule_shift;

    double seconds = th_granule_time(mCtx, granulepos);
    double frameRate = (double)(mInfo.fps_numerator) / (double)(mInfo.fps_denominator);
    ogg_int64_t frameDuration = (1000 * mInfo.fps_denominator) / mInfo.fps_numerator;
    time = (ogg_int64_t)(seconds * 1000) - frameDuration;
    if (!mGotStartTime) {
      mGotStartTime = true;
      mStartTime = time;
      assert(mStartTime >= 0);
    }
    mEndTime = time + frameDuration;
    isKeyFrame = th_packet_iskeyframe(packet) == 1;
    mPacketCount++;
    if (packet->granulepos != -1) {
      assert(packet->granulepos == granulepos);
    }
    return true;
  }

  virtual bool ReadHeader(ogg_packet* packet) {
    ogg_int32_t ret = th_decode_headerin(&mInfo,
                                         &mComment,
                                         &mSetup,
                                         packet);
    if (ret == TH_ENOTFORMAT) {
      // Not theora, continue;
      return false;
    }
    if (ret > 0) {
      // Read Theora header.
      mHeadersRead++;
    }
    if (GotAllHeaders()) {
      // Read all headers, setup decoder context.
      mCtx = th_decode_alloc(&mInfo, mSetup);
      assert(mCtx != NULL);
    }
    return true;
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
  
  KeyFrameInfo* mCurrentFrameInfo;
  KeyFrameInfo* mSpanFrameInfo;
  
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

  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    if (!GotAllHeaders())
      return -1;
    return (ogg_int64_t)(1000 * vorbis_granule_time(&mDsp, granulepos));
  }


  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      ogg_int64_t& time)
  {
    ogg_int32_t ret = 0;

    if (vorbis_synthesis(&mBlock, packet) == 0) {
      ret = vorbis_synthesis_blockin(&mDsp, &mBlock);
      assert(ret == 0);
    }

    float** pcm = 0;
    ogg_int32_t samples = 0;
    ogg_int32_t totalSamples = 0;
    while ((samples = vorbis_synthesis_pcmout(&mDsp, &pcm)) > 0) {
      totalSamples += samples;
      ret = vorbis_synthesis_read(&mDsp, samples);
      assert(ret == 0);
    }
    
    if (mDsp.granulepos != -1) {
      ogg_int64_t endTime = (ogg_int64_t)(1000 * vorbis_granule_time(&mDsp, mDsp.granulepos));
      ogg_int64_t duration = 1000 * totalSamples / (mInfo.channels * mInfo.rate);
      time = endTime;
      isKeyFrame = true;
      if (!mGotStartTime) {
        mGotStartTime = true;
        mStartTime = time - duration;
      }
      mEndTime = time;
    } else {
      time = -1;
      isKeyFrame = false;
    }
    return true;
  }

  virtual bool GotAllHeaders() {
    // Vorbis has exactly 3 header packets, identification, comment and setup.
    return mHeadersRead == 3;
  } 

  virtual bool ReadHeader(ogg_packet* packet)
  {
    ogg_int32_t ret = vorbis_synthesis_headerin(&mInfo, &mComment, packet);
    if (ret == 0) {
      mHeadersRead++;
    }
    if (GotAllHeaders()) {
      ret = vorbis_synthesis_init(&mDsp, &mInfo);
      assert(ret == 0);
      ret = vorbis_block_init(&mDsp, &mBlock);
      assert(ret == 0);
    }
    return true;
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


static bool IsSkeletonHeader(ogg_packet* packet) {
  return strcmp((const char*)packet->packet, "fishead") == 0;
}

bool OggStream::ReadHeaders(ogg_packet* packet,
                            ogg_int64_t pageOffset) 
{
  if (!mDecoder) {
    // We've not got a decoder for this stream.
    
    // See if this is Theora...
    TheoraDecoder* theora = new TheoraDecoder(mSerial);
    if (theora->ReadHeader(packet)) {
      mDecoder = theora;
      mType = TYPE_THEORA;
      return true;
    } else {
      delete theora;
      theora = 0;
    }
          
    // See if this is Vorbis..
    if (vorbis_synthesis_idheader(packet)) {
      VorbisDecoder* vorbis = new VorbisDecoder(mSerial);
      if (vorbis->ReadHeader(packet)) {
        mDecoder = vorbis;
        mType = TYPE_VORBIS;
        return true;
      } else {
        delete vorbis;
        vorbis = 0;
      }
    }
    
    if (IsSkeletonHeader(packet)) {
      mDecoder = new SkeletonDecoder(mSerial);
      mType = TYPE_SKELETON;
      mDecoder->ReadHeader(packet);
      return true;
    }
    
    // Else check for other formats...
    mType = TYPE_UNKNOWN;
    return false;  
  }
  
  return mDecoder->ReadHeader(packet);
}

// Convert stream map of serialno->stream to a vector of streams. Easier to
// work with vectors...
vector<OggStream*>
GetStreamVector(StreamMap& m)
{
  vector<OggStream*> v;
  StreamMap::iterator itr = m.begin();
  while (itr != m.end()) {
    OggStream* s = itr->second;
    v.push_back(s);
    itr++;
  }
  return v; 
}

