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
 * OggIndex.cpp - ogg file indexer main file.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#include <iostream>
#include <fstream>
#include <algorithm>
#include <assert.h>
#include <time.h>
#include <map>
#include <vector>
#include <iomanip>
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <ogg/ogg.h>
#include <theora/theora.h>
#include <theora/theoradec.h>
#include <vorbis/codec.h>
#include "Options.hpp"
#include "ogg_index.h"


#ifdef WIN32
typedef __int64 int64;
typedef unsigned __int64 uint64;
#define DEBUG _DEBUG
#else
typedef int64_t int64;
typedef uint64_t uint64;
#endif

#define INT64_MAX LLONG_MAX
#define INT64_MIN LLONG_MIN
#define UINT64_MAX ULLONG_MAX
#define UINT64_MIN 0

using namespace std;

// Need to index keyframe if we've not seen 1 in 64K.
#define MIN_KEYFRAME_OFFSET (64 * 1024)

static const int INDEX_HEADER_SIZE = 31;

static const unsigned char INDEX_VERSION = 0x01;

// Size of one key point entry in the index.
// sizeof(int64) + sizeof(int32) + sizeof(int64)
static const int KEY_POINT_SIZE = 20;

static const int FILE_BUFFER_SIZE = 1024 * 1024;

static const char HEADER_MAGIC[] = "index";
static const int HEADER_MAGIC_LEN = sizeof(HEADER_MAGIC) / sizeof(HEADER_MAGIC[0]);

enum StreamType {
  TYPE_UNKNOWN,
  TYPE_VORBIS,
  TYPE_THEORA,
  TYPE_SKELETON,
  TYPE_UNSUPPORTED
};

// Command line parameter parser and global program options.
Options gOptions;

// Get the length in bytes of a file. 32bit only, won't work for files larger
// than 2GB.
static int64
FileLength(const char* aFileName)
{
  ifstream is;
  is.open (aFileName, ios::binary);
  is.seekg (0, ios::end);
  streampos length = is.tellg();
  is.close();
  return (int64)length;
}

static inline int64
InputFileLength()
{
  return FileLength(gOptions.GetInputFilename().c_str());
}


class KeyFrameInfo {
public:
  KeyFrameInfo() : mOffset(0), mTime(-1), mChecksum(0) {}

  KeyFrameInfo(uint64 offset, int64 time, unsigned checksum) :
    mOffset(offset),
    mTime(time),
    mChecksum(checksum) {}
  uint64 mOffset; // In bytes from beginning of file.
  int64 mTime; // In milliseconds.
  unsigned mChecksum;

  bool operator<(const KeyFrameInfo& other) {
    return other.mOffset < mOffset;
  }
};

class Decoder {
public:
  Decoder(long serial) :
    mSerial(serial),
    mStartTime(-1),
    mEndTime(-1),
    mGotStartTime(false) {}

  virtual ~Decoder() {}

  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      int64& time) = 0;

  virtual bool ReadHeader(ogg_packet* packet) = 0;

  virtual bool GotAllHeaders() = 0;
  
  int64 GetStartTime() { return mStartTime; }
  int64 GetEndTime() { return mEndTime; }
  virtual int64 GranuleposToTime(int64 granulepos) = 0;

  virtual const char* TypeStr() { return "?"; }

protected:
  long mSerial;
  int64 mStartTime;
  int64 mEndTime;
  bool mGotStartTime;
};


static int64
Granule(int64 granulepos, int granuleshift) {
  if (granulepos == -1) {
    // Assume that -1 means it starts at zero...
    return 1;
  }
  int64 iframe = granulepos >> granuleshift;
  int64 pframe = granulepos - (iframe << granuleshift);
  int64 granule = iframe + pframe;  
  return granule;
}

static int64
GranuleposToTime(int64 granulepos, int granuleshift, int denom, int num)
{
  int64 g = Granule(granulepos, granuleshift);
  int64 time = 1000 * g * denom / num;
  return time;
}

class TheoraDecoder : public Decoder { 
public:
  TheoraDecoder(long serial) :
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

  virtual int64 GranuleposToTime(int64 granulepos) {
    int64 g = Granule(granulepos, mInfo.keyframe_granule_shift);
    int64 time = 1000 * g * mInfo.fps_denominator / mInfo.fps_numerator;
    return time;
  }
 
  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      int64& time)
  {
    int ret = 0;
    assert(GotAllHeaders());
    if (!mSetFirstGranulepos) {
      mSetFirstGranulepos = true;
      if (packet->granulepos != -1) {
        int64 granule = Granule(packet->granulepos,
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
    int64 granulepos = mPacketCount << mInfo.keyframe_granule_shift;

    double seconds = th_granule_time(mCtx, granulepos);
    double frameRate = (double)(mInfo.fps_numerator) / (double)(mInfo.fps_denominator);
    int64 frameDuration = (1000 * mInfo.fps_denominator) / mInfo.fps_numerator;
    time = (int64)(seconds * 1000);
    if (!mGotStartTime) {
      mGotStartTime = true;
      mStartTime = time;
      assert(mStartTime >= 0);
    }
    mEndTime = time;
    isKeyFrame = th_packet_iskeyframe(packet) == 1;
    mPacketCount++;
    return true;
  }

  virtual bool ReadHeader(ogg_packet* packet) {
    int ret = th_decode_headerin(&mInfo,
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

protected:

  th_info mInfo;
  th_comment mComment;
  th_setup_info *mSetup;
  th_dec_ctx* mCtx;

  int mHeadersRead;
  int64 mPacketCount; 
  
  KeyFrameInfo* mCurrentFrameInfo;
  KeyFrameInfo* mSpanFrameInfo;
  
  bool mSetFirstGranulepos;
};

class VorbisDecoder : public Decoder {
public:

  VorbisDecoder(long serial) :
    Decoder(serial),
    mHeadersRead(0)    
  {
    vorbis_info_init(&mInfo);
    vorbis_comment_init(&mComment);    
  }

  virtual ~VorbisDecoder() {}

  virtual const char* TypeStr() { return "V"; }  

  virtual int64 GranuleposToTime(int64 granulepos) {
    if (!GotAllHeaders())
      return -1;
    return (int64)(1000 * vorbis_granule_time(&mDsp, granulepos));
  }


  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      int64& time)
  {
    int ret = 0;

    if (vorbis_synthesis(&mBlock, packet) == 0) {
      ret = vorbis_synthesis_blockin(&mDsp, &mBlock);
      assert(ret == 0);
    }

    float** pcm = 0;
    int samples = 0;
    int totalSamples = 0;
    while ((samples = vorbis_synthesis_pcmout(&mDsp, &pcm)) > 0) {
      totalSamples += samples;
      ret = vorbis_synthesis_read(&mDsp, samples);
      assert(ret == 0);
    }
    
    if (mDsp.granulepos != -1) {
      int64 endTime = (int64)(1000 * vorbis_granule_time(&mDsp, mDsp.granulepos));
      int64 duration = 1000 * totalSamples / (mInfo.channels * mInfo.rate);
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
    int ret = vorbis_synthesis_headerin(&mInfo, &mComment, packet);
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

private:
  int mHeadersRead;
  
  vorbis_info mInfo;
  vorbis_comment mComment;
  vorbis_dsp_state mDsp;
  vorbis_block mBlock;
  
};

class SkeletonDecoder : public Decoder {
public:

  SkeletonDecoder(long serial) :
    Decoder(serial),
    mGotAllHeaders(0)    
  {
  }

  virtual ~SkeletonDecoder() {}

  virtual const char* TypeStr() { return "S"; }  

  virtual int64 GranuleposToTime(int64 granulepos) {
    return -1;
  }

  virtual bool Decode(ogg_packet* packet,
                      bool& isKeyFrame,
                      int64& time)
  {
    return ReadHeader(packet);
  }

  virtual bool GotAllHeaders() {
    return mGotAllHeaders;
  } 

  virtual bool ReadHeader(ogg_packet* packet)
  {
    // We've read all headers when we receive the EOS packet.
    if (packet->e_o_s)
      mGotAllHeaders = true;
    return true;
  }
private:
  bool mGotAllHeaders;
};

static unsigned
LEUint32(unsigned const char* p) {
  unsigned i =  p[0] +
               (p[1] << 8) + 
               (p[2] << 16) +
               (p[3] << 24);
  return i;  
}

static unsigned
GetChecksum(ogg_page* page)
{
  assert(page != 0);
  assert(page->header != 0);
  assert(page->header_len > 25);
  return LEUint32(page->header + 22);
}

class OggStream
{
public:
  OggStream(long serial) :
    mSerial(serial),
    mPacketCount(0),
    mType(TYPE_UNSUPPORTED),
    mDecoder(0),
    mPacketSpanningPage(false),
    mSpanningPageOffset(0),
    mSpanningPageChecksum(0),
    mStartTime(0),
    mEndTime(0),
    mGotStartTime(false) {}

  ~OggStream() {
    if (mDecoder)
      delete mDecoder;
  }
  
  long mSerial;
  ogg_stream_state mState;
  int mPacketCount;
  StreamType mType;
  
  int64 GetStartTime() {
    return mDecoder ? mDecoder->GetStartTime() : -1;
  }
  
  int64 GetEndTime() {
    return mDecoder ? mDecoder->GetEndTime() : -1;
  }
  
  int64 GranuleposToTime(int64 granulepos) {
    return mDecoder ? mDecoder->GranuleposToTime(granulepos) : -1;
  }
  
  bool Decode(ogg_page* page,
              int64 pageOffset)
  {
    int ret = ogg_stream_pagein(&mState, page);
    ogg_int64_t granulepos = ogg_page_granulepos(page);
    assert(ret == 0);

    ogg_packet packet;
    bool packetSpans = false;
    while (true) {
      ret = ogg_stream_packetout(&mState, &packet);
      if (ret == 0) {
        // This packet spans a page boundary.
        if (!mPacketSpanningPage) {
          // This is the first page boundary this packet spans, remember
          // its page offset so we know where this spanning packet started.
          mSpanningPageOffset = pageOffset;
          mSpanningPageChecksum = GetChecksum(page);
        }
        mPacketSpanningPage = true;
        return true;
      }
      // We've either read a packet, or failed. Reset spanning flag so we
      // don't need to worry about resetting it on every exit of loop scope.
      packetSpans = mPacketSpanningPage;
      mPacketSpanningPage = false;
            
      if (ret == -1) {
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
      int64 frameTime = 0;
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

      int64 offset = packetSpans ? mSpanningPageOffset : pageOffset;
      unsigned checksum = packetSpans ? mSpanningPageChecksum : GetChecksum(page);

      #ifdef DEBUG
      if (offset < INT_MAX) {
        // Verify that there's a page at offset, and that it's the one we think it is.
        ifstream file(gOptions.GetInputFilename().c_str(), ios::in | ios::binary);
        assert(file);
        
        // Verify page at offset.
        file.seekg((int)offset, ios_base::beg);
        char buf[5];
        file.read(buf, 5);
        assert(file.gcount() == 5);
        assert(strcmp(buf, "OggS") == 0);
        
        // Verify checksum, ensures the page is the one we think it is.
        file.seekg((int)offset+22, ios_base::beg);
        file.read(buf, 4);
        assert(file.gcount() == 4);
        unsigned page_checksum = LEUint32((unsigned char*)buf);
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

  const char* TypeStr() { return mDecoder ? mDecoder->TypeStr() : "?"; }

  vector<KeyFrameInfo> mKeyframes;

private:

  unsigned mSpanningPageChecksum;
  int64 mSpanningPageOffset;
  int64 mStartTime;
  int64 mEndTime;
  bool mPacketSpanningPage;
  bool mGotStartTime;

  // Returns true if the keyframe at offset/frameTime is enough after the
  // previous keyframe for us to consider indexing it.
  bool IsAfterThreshold(uint64 offset, int64 frameTime) { 
    if (mKeyframes.size() == 0)
      return true;
    KeyFrameInfo& prev = mKeyframes.back();
    return offset > (prev.mOffset + MIN_KEYFRAME_OFFSET) &&
           frameTime > (prev.mTime + gOptions.GetKeyPointInterval());
  }

  bool IsKeyframeVectorSorted()
  {
    const vector<KeyFrameInfo>& K = mKeyframes;
    for (unsigned i=1; i<K.size(); i++) {
      if (K[i-1].mOffset >= K[i].mOffset ||
          K[i-1].mTime >= K[i].mTime)
      {
        return false;
      }
    }
    return true;
  }

  bool IsSkeletonHeader(ogg_packet* packet) {
    return strcmp((const char*)packet->packet, "fishead") == 0;
  }

  bool ReadHeaders(ogg_packet* packet,
                   int64 pageOffset) 
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

  Decoder* mDecoder;
};



typedef map<long, OggStream*> StreamMap;

// Convert stream map of serialno->stream to a vector of streams. Easier to
// work with vectors...
static vector<OggStream*>
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

// Returns nuber of bytes read.
bool ReadPage(ogg_sync_state* state,
              ogg_page* page,
              istream& stream,
              uint64& bytesRead)
{
  int bytes = 0;
  int r = 0;
  uint64 intialBytesRead = bytesRead;
  while ((r = ogg_sync_pageout(state, page)) != 1) {
    char* buffer = ogg_sync_buffer(state, FILE_BUFFER_SIZE);
    assert(buffer);

    stream.read(buffer, FILE_BUFFER_SIZE);
    bytes = stream.gcount();
    bytesRead += bytes;
    if (bytes == 0) {
      // End of file
      assert(stream.eof());
      if (intialBytesRead != bytesRead) {
        cerr << "WARNING: Reached end of file, when expecting to find more data! "
             << "Page header may be incorrect!" << endl;
      }
      return false;
    }

    int ret = ogg_sync_wrote(state, bytes);
    assert(ret == 0);
  }  
  return true;
}

static bool
IsPageAtOffset(string& filename, int64 offset, ogg_page* page)
{
  ifstream file(filename.c_str(), ios::in | ios::binary);
  assert(file);
  file.seekg((int)offset, ios_base::beg);
  char* buf = new char[max(page->body_len, page->header_len)];
  file.read(buf, page->header_len);
  assert(file.gcount() == page->header_len);
  if (memcmp(buf, page->header, page->header_len) != 0) {
    cerr << "Incorrect page offset calculation for page at offset "
         << offset << endl;
    delete buf;
    return false;
  }
  
  file.read(buf, page->body_len);
  assert(file.gcount() == page->body_len);
  if (memcmp(buf, page->body, page->body_len) != 0) {
    cerr << "Incorrect page offset calculation for page at offset "
         << offset << endl;
    delete buf;
    return false;
  }

  delete buf;
  return true;
}

static int64
GetMinStartTime(vector<OggStream*>& streams) 
{
  int64 m = INT64_MAX;
  for (unsigned i=0; i<streams.size(); i++) {
    int64 t = streams[i]->GetStartTime();
    if (t < m && t > -1) {
      m = t;
    }
  }
  return m;
}

static int64
GetMaxEndtime(vector<OggStream*>& streams)
{
  int64 m = INT64_MIN;
  for (unsigned i=0; i<streams.size(); i++) {
    int64 t = streams[i]->GetEndTime();
    if (t > m && t > -1) {
      m = t;
    }
  }
  return m;
}

static uint64
ReadLEUint64(unsigned char* p)
{
  uint64 lo = LEUint32(p);
  uint64 hi = LEUint32(p+4);
  return lo + (hi << 32);
};

static unsigned char*
WriteLEUint64(unsigned char* p, const uint64 num)
{
  assert(p);
  uint64 n = num;
  for (int i=0; i<8; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(ReadLEUint64(p) == num);
  return p + 8;
}

static unsigned char*
WriteLEUint32(unsigned char* p, const unsigned num)
{
  assert(p);
  unsigned n = num;
  for (int i=0; i<4; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEUint32(p) == num);
  return p + 4;
}

// Compare function, used to sort index vector of packets.
// returns true if a is before b.
static bool
compare_index_packet(ogg_packet* a, ogg_packet* b)
{
  long aSerial = LEUint32(a->packet);
  long bSerial = LEUint32(b->packet);
  return aSerial < bSerial;
}

static bool
IsSorted(vector<ogg_packet*>& indexes)
{
  for (unsigned i=1; i<indexes.size(); i++) {
    ogg_packet* prev = indexes[i-1];
    ogg_packet* here = indexes[i];
    long prevSerial = LEUint32(prev->packet);
    long hereSerial = LEUint32(here->packet);
    assert(prevSerial < hereSerial);
    if (prevSerial >= hereSerial)
      return false;
  }
  return true;
}

class IndexEncoder {
private:

  bool
  IsIndexable(OggStream* stream) {
    return stream->mType == TYPE_VORBIS ||
           stream->mType == TYPE_THEORA;
  }

public:
  IndexEncoder(vector<OggStream*>& streams,
               int64 length)
    : mFileLength(length),
      mPacketCount(0)
  {
    for (unsigned i=0; i<streams.size(); i++) {
      if (IsIndexable(streams[i])) {
        mStreams.push_back(streams[i]);
      }
    }
  }
  
  ~IndexEncoder() {
  
  }
  
  // Gets the data for the complete index stream, ready for
  // writing to file.
  void GetTrackData(unsigned char** track,
                    unsigned& length)
  {
    ogg_packet* header = GetHeaderPacket();
    vector<ogg_packet*> indexes;
    GetIndexPackets(indexes);
    
    ogg_packet* eos = new ogg_packet();
    memset(eos, 0, sizeof(ogg_packet));
    eos->e_o_s = 1;
    eos->packetno = mPacketCount;

    mSerial = GetUniqueSerialNo();
  
    // First, encode all the packets into pages, so we can determine the length
    // of the stream.
    unsigned len=0;
    GetTrackData(0, len, mSerial, header, indexes, eos);

    // We know the length of the stream, we must add this to the offsets of
    // every known point's offset, else they'll be incorrect.
    assert(len != 0);
    uint64 streamLength = CorrectStreamLength(header, len);
    CorrectIndexOffsets(indexes, len, streamLength);
    
    // Write stream to buf.
    *track = new unsigned char[len];
    GetTrackData(track, length, mSerial, header, indexes, eos);
    assert(length == len);

    delete header->packet;
    delete header;
    for (unsigned i=0; i<indexes.size(); i++) {
      delete indexes[i]->packet;
    }
    delete eos;
  }
  
  int GetIndexSerial() {
    return mSerial;
  }
  
private:

  vector<OggStream*> mStreams;
  int64 mFileLength;
  int mSerial;
  int mPacketCount;

  void
  CorrectIndexOffsets(vector<ogg_packet*>& indexes,
                      uint64 headerLength,
                      uint64 streamLength)
  {
    uint64 newFileLength = InputFileLength() + headerLength;
    for (unsigned idx=0; idx<indexes.size(); idx++) {
      ogg_packet* packet = indexes[idx];
      assert(packet);
      long serial = LEUint32(packet->packet);
      int n = LEUint32(packet->packet+4);
      unsigned char* p = packet->packet + 8;
      assert(n == ((packet->bytes - 8) / KEY_POINT_SIZE));
      assert(n >= 0);
      for (int i=0; i<n; i++) {
        assert(p < packet->packet + packet->bytes);
        uint64 o = ReadLEUint64(p);
        o += headerLength;
        assert(o < streamLength);
        assert(o < newFileLength);
        WriteLEUint64(p, o);
        p += KEY_POINT_SIZE;
      }
    }
  }

  uint64
  CorrectStreamLength(ogg_packet* packet, uint64 streamLength)
  {
    // [23-30] The length of the indexed segment, in bytes, uint64.
    unsigned char* p = packet->packet + 23;
    WriteLEUint64(p, mFileLength + streamLength);
    assert(ReadLEUint64(p) == mFileLength + streamLength);
    return mFileLength + streamLength;
  }

  void
  CopyPage(unsigned char** buf, ogg_page& page, unsigned& offset)
  {
    if (buf && *buf) {
      memcpy(*buf + offset, page.header, page.header_len);
      memcpy(*buf + offset + page.header_len, page.body, page.body_len);
    }
    offset += page.body_len + page.header_len;
  }

  void GetTrackData(unsigned char** track,
                    unsigned& length,
                    int serialno,
                    ogg_packet* header,
                    vector<ogg_packet*>& indexes,
                    ogg_packet* eos)
  {
    int ret = 0;
    ogg_page page;
    ogg_stream_state state;
    memset(&state, 0, sizeof(ogg_stream_state));
    memset(&page, 0, sizeof(ogg_page));
    length = 0;
    
    ret = ogg_stream_init(&state, serialno);
    assert(ret == 0);

    ret = ogg_stream_packetin(&state, header);
    assert(ret == 0);

    ret = ogg_stream_flush(&state, &page);
    assert(ret != 0);
    CopyPage(track, page, length);

    for (unsigned i=0; i<indexes.size(); i++) {
      ret = ogg_stream_packetin(&state, indexes[i]);
      assert(ret == 0);

      while (ogg_stream_pageout(&state, &page) != 0) {
        CopyPage(track, page, length);
      }

      ret = ogg_stream_flush(&state, &page);
      if (ret != 0) {
        CopyPage(track, page, length);
      }
    }

    ret = ogg_stream_packetin(&state, eos);
    assert(ret == 0);

    ret = ogg_stream_flush(&state, &page);
    assert(ret != 0);
    CopyPage(track, page, length);
    
    ogg_stream_clear(&state);  
  }

  ogg_packet*
  GetHeaderPacket()
  {
    ogg_packet* packet = new ogg_packet();
    memset(packet, 0, sizeof(ogg_packet));
    packet->b_o_s = 1;
    packet->bytes = INDEX_HEADER_SIZE;
    unsigned char* p = new unsigned char[INDEX_HEADER_SIZE];
    memset(p, 0, INDEX_HEADER_SIZE);
    packet->packet = p;

    int64 startTime = GetMinStartTime(mStreams);
    int64 endTime = GetMaxEndtime(mStreams);

    // [0-5] "index\0".
    assert(HEADER_MAGIC_LEN == strlen(HEADER_MAGIC) + 1);
    memcpy(p, HEADER_MAGIC, HEADER_MAGIC_LEN);
    p += HEADER_MAGIC_LEN;
    
    // [6] 0x01 version number, 1 unsigned byte.
    *p = INDEX_VERSION;
    p++;

    // [7-14] The playback start time, in milliseconds, uint64.
    assert(*p == 0);
    p = WriteLEUint64(p, startTime);

    // [15-22] The playback end time, in milliseconds, uint64.
    p = WriteLEUint64(p, endTime);
    
    // [23-30] The length of the indexed segment, in bytes, uint64.
    p = WriteLEUint64(p, mFileLength);

    assert(packet->packetno == 0);
    mPacketCount++;
    assert(mPacketCount == 1);

    return packet;
  }

  void
  GetIndexPackets(vector<ogg_packet*>& indexes)
  {
    assert(mPacketCount == 1);
    for (unsigned i=0; i<mStreams.size(); i++) {
      ogg_packet* packet = new ogg_packet();
      memset(packet, 0, sizeof(ogg_packet));
      
      vector<KeyFrameInfo>& keyframes = mStreams[i]->mKeyframes;
      
      const int tableSize = 8 + (int)keyframes.size() * KEY_POINT_SIZE;
      packet->bytes = tableSize;
      unsigned char* p = new unsigned char[tableSize];
      memset(p, 0, tableSize);
      packet->packet = p;

      // Stream serialno.
      p = WriteLEUint32(p, mStreams[i]->mSerial);
      
      // Number of key points.
      assert(keyframes.size() < UINT_MAX);
      p = WriteLEUint32(p, (unsigned)keyframes.size());

      for (unsigned i=0; i<keyframes.size(); i++) {
        KeyFrameInfo& k = keyframes[i];
        p = WriteLEUint64(p, k.mOffset);
        p = WriteLEUint32(p, k.mChecksum);
        p = WriteLEUint64(p, k.mTime);
      }
      
      assert(p == packet->packet + tableSize);

      packet->packetno = mPacketCount;
      mPacketCount++;

      assert(packet->e_o_s == 0);
      indexes.push_back(packet);
    }
    sort(indexes.begin(), indexes.end(), compare_index_packet);
    assert(IsSorted(indexes));
  }

  int
  GetUniqueSerialNo()
  {
    int serialno = gOptions.GetSerialNo();
    bool warn = false;
    if (serialno != -1) {
      // User supplied serialno.
      if (IsUniqueSerialno(serialno)) {
        return serialno;
      }
      // User supplied a serialno in use, warn them.
      warn = true;
    }
      
    srand((unsigned)time(0));
    do {
      serialno = rand();
    } while (!IsUniqueSerialno(serialno));
    
    if (warn) {
      cerr << "WARNING: User specified serialno " << gOptions.GetSerialNo()
           << " is already in use, using " << serialno << " instead." << endl;
    }
    
    return serialno; 
  }

  bool
  IsUniqueSerialno(int serialno)
  {
    for (unsigned i=0; i<mStreams.size(); i++) {
      if (mStreams[i]->mSerial == serialno) {
        return false;
      }
    }
    return true;
  }

};

static void
CopyFileData(istream& input, ostream& output, int64 bytesToCopy)
{
  assert(input.good());
  assert(output.good());
  // Copy data in chunks at most 1mb in size.
  assert(bytesToCopy >= 0);
  assert((int64)FILE_BUFFER_SIZE < (int64)INT_MAX);
  int len = (int)min(bytesToCopy, (int64)FILE_BUFFER_SIZE);
  char* buf = new char[len];
  int64 bytesCopied = 0;
  while (bytesCopied != bytesToCopy) {
    int64 remaining = bytesToCopy - bytesCopied;
    int x = (int)min(remaining, (int64)len);
    input.read(buf, x);
    assert(x == input.gcount());
    output.write(buf, x);
    bytesCopied += x;
  }
  delete buf;
}

// Reads the index out of an indexed file, and checks that the offsets
// line up with the pages they think they do, by checking the checksum.
bool VerifyIndex(int indexSerial) {
  bool valid = true;
  
  string filename = gOptions.GetOutputFilename();
  ifstream input(filename.c_str(), ios::in | ios::binary);
  int64 outputFileLength = FileLength(filename.c_str());
  ogg_sync_state state;
  int ret = ogg_sync_init(&state);
  assert(ret==0);
  
  ogg_stream_state streamState;
  ret = ogg_stream_init(&streamState, indexSerial);
  assert(ret==0);

  ogg_page page;
  memset(&page, 0, sizeof(ogg_page));

  ogg_index index;
  ogg_index_init(&index);

  uint64 bytesRead = 0;
  uint64 offset = 0;
  unsigned pageNumber = 0;
  int packetCount = 0;
  while (!ogg_index_is_loaded(&index) && ReadPage(&state, &page, input, bytesRead)) {
    assert(IsPageAtOffset(filename, offset, &page));
    
    if (ogg_page_serialno(&page) == indexSerial) {
      ret = ogg_stream_pagein(&streamState, &page);
      assert(ret == 0);

      ogg_packet packet;
      memset(&packet, 0, sizeof(ogg_packet));
      while (1 == ogg_stream_packetout(&streamState, &packet)) {
        packetCount++;
        if (packetCount == 1) {
          // Header packet
          if (!packet.b_o_s) {
            valid = false;
            cerr << "Verification failure: first packet must be BOS packet." << endl;
            break;
          }
          ret = ogg_index_decode(&index, &packet);
          if (ret != 0) {
            valid = false;
            cerr << "Verification failure: ogg_index_decode() failure reading header packet." << endl;
            break;
          }
        } else if (!packet.e_o_s) {
          // Index packet
          ret = ogg_index_decode(&index, &packet);
          if (ret != 0) {
            valid = false;
            cerr << "Verification failure: ogg_index_decode() failure reading index packet." << endl;
            break;
          }
        } else {
          assert(packet.e_o_s);
          ret = ogg_index_decode(&index, &packet);
          assert(ret == 1);
          assert(ogg_index_is_loaded(&index));
          break;
        }
      }
    }
    
    unsigned length = page.body_len + page.header_len;
    offset += length;
    memset(&page, 0, sizeof(ogg_page));
  }
  
  if (!ogg_index_is_loaded(&index)) {
    cerr << "Verification failure: Some problem reading pages." << endl;
  }

  if (valid) {
    for (unsigned s=0; s<index.num_streams; s++) {
      ogg_stream_index* stream_index = &index.stream[s];
      for (unsigned i=0; valid && i<stream_index->num_key_points; i++) {
        ogg_sync_reset(&state);
        memset(&page, 0, sizeof(ogg_page));
        char* buf = ogg_sync_buffer(&state, 8*1024);
        assert(buf);
        ogg_index_keypoint* keypoint = &stream_index->key_points[i];
        if (keypoint->offset > INT_MAX) {
          cerr << "WARNING: Can only verified up to 2^31 bytes into the file." << endl;
          break;
        }
        
        if (keypoint->offset > outputFileLength) {
          valid = false;
          cerr << "Verification failure: keypoint offset out of file range." << endl;
          break;
        }
        
        input.seekg((std::streamoff)keypoint->offset);
        int bytes = (int)min((int64)8*1024, (outputFileLength - keypoint->offset)); 
        assert(bytes > 0);
        input.read(buf, bytes);
        int bytesToRead = input.gcount();
        ret = ogg_sync_wrote(&state, bytesToRead);
        if (ret != 0) {
          valid = false;
          cerr << "Verification failure: ogg_sync_wrote() failure reading data in verification." << endl;
          break;
        }
        ret = ogg_sync_pageout(&state, &page);
        if (ret != 1) {
          valid = false;
          cerr << "Verification failure: ogg_sync_pageout() failure reading data in verification." << endl;
          break;
        }
        valid = (GetChecksum(&page) == keypoint->checksum);
        if (!valid) {
          cerr << "Verification failure: Incorrect checksum for page at offset "
               << keypoint->offset << endl;
          break;
        }
      }

    }
  }

  ogg_stream_clear(&streamState);
  ogg_sync_clear(&state);

  return valid;
}



int main(int argc, char** argv) 
{
  if (!gOptions.Parse(argc, argv)) {
    return -1;
  }

  string filename = gOptions.GetInputFilename();
  ifstream input(filename.c_str(), ios::in | ios::binary);
  ogg_sync_state state;
  int ret = ogg_sync_init(&state);
  assert(ret==0);

  StreamMap streams;
  ogg_page page;
  memset(&page, 0, sizeof(ogg_page));

  time_t startTime = time(0);

  uint64 bytesRead = 0;
  uint64 offset = 0;
  unsigned pageNumber = 0;
  uint64 insertionPoint = -1;
  while (ReadPage(&state, &page, input, bytesRead)) {
    assert(IsPageAtOffset(filename, offset, &page));
    pageNumber++;
    int serial = ogg_page_serialno(&page);
    OggStream* stream = 0;
    if (ogg_page_bos(&page)) {
      stream = new OggStream(serial);
      ret = ogg_stream_init(&stream->mState, serial);
      assert(ret == 0);
      streams[serial] = stream;
    } else {
      if (insertionPoint == -1) {
        // First non bos page, we insert our index track here.
        insertionPoint = offset;
      }
      stream = streams[serial];
    }

    unsigned length = page.body_len + page.header_len;
    if (gOptions.GetDumpPages()) {
      int64 granulepos = ogg_page_granulepos(&page);
      cout << "[" << stream->TypeStr() << "] page @" << offset
           << " length=" << length << " granulepos=" << granulepos 
           << " time=" << stream->GranuleposToTime(granulepos) << "ms"
           << " s=" << serial << endl;
    }

    if (stream->mType == TYPE_UNKNOWN) {
      cerr << "FAIL: Unhandled content type in stream serialno="
           << stream->mSerial << " aborting indexing!" << endl;
      return -1;
    }

    stream->Decode(&page, offset);

    offset += length;
    memset(&page, 0, sizeof(ogg_page));
  }

  const uint64 fileLength = bytesRead;
  assert(input.eof());
  if (offset != fileLength) {
    cerr << "WARNING: Ogg page lengths don't sum to file length!" << endl;
  }
  
  assert(fileLength == InputFileLength());
  
  ogg_sync_clear(&state);
  
  vector<OggStream*> sv = GetStreamVector(streams);

  IndexEncoder encoder(sv, fileLength);
  unsigned char* trackData = 0;
  unsigned trackLength = 0;
  encoder.GetTrackData(&trackData, trackLength);

  // Reopen the file so we can write it out with the index.
  input.close();
  input.clear();
  input.open(filename.c_str(), ios::in | ios::binary);
  assert(input.good());
  
  // Write the file with its index out to disk.
  ofstream output(gOptions.GetOutputFilename().c_str(), ios::out | ios::binary);
  
  // Copy up to header insertion point over to new file.
  CopyFileData(input, output, insertionPoint);
  
  // Write out header.
  output.write((const char*)trackData, trackLength);
    
  // Write out the rest of the file.
  CopyFileData(input, output, fileLength - insertionPoint);
  
  output.close();
  input.close();

  delete trackData;

  if (gOptions.GetVerifyIndex()) {
    if (!VerifyIndex(encoder.GetIndexSerial())) {
      cerr << "FAIL: Verification of the index failed!" << endl;
    } else {
      cout << "SUCCESS: index passes verification." << endl;
    }
  }
  cout << "Index track length: " << trackLength << " bytes, "
       << ((float)trackLength / (float)(fileLength + trackLength)) * 100.0
       << "% overhead" << endl;
  return 0;
}
