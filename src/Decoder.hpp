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
 * Decoder.hpp - Classes for decoding ogg streams for indexing.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __DECODER_HPP__
#define __DECODER_HPP__

#include <string>
#include <map>
#include <vector>
#include <ogg/ogg.h>

using namespace std;

// Maximum possible size of one uncompressed keypoint entry in the index. This
// takes into account the maximum possible values for all fields, and the number
// of bytes required to encode their maximum values with variable byte encoding.
#define MAX_KEY_POINT_SIZE 24

// Magic bytes for index packet.
#define HEADER_MAGIC "index"
#define HEADER_MAGIC_LEN (sizeof(HEADER_MAGIC) / sizeof(HEADER_MAGIC[0]))

// Stores codec-specific skeleton info.
class FisboneInfo {
public:
  
  FisboneInfo()
    : mGranNumer(0)
    , mGranDenom(0)
    , mPreroll(0)
    , mGranuleShift(0)
  {}

  // Granulerate numerator.
  ogg_int64_t mGranNumer;

  // Granulerate denominator.
  ogg_int64_t mGranDenom;  
  
  ogg_int32_t mPreroll;
  ogg_int32_t mGranuleShift;
  
  // "Content-Type: major/minor\r\n".
  string mContentType;
};

// Stores info about a key point.
class KeyFrameInfo {
public:
  KeyFrameInfo() : mOffset(0), mTime(-1) {}

  KeyFrameInfo(ogg_int64_t offset, ogg_int64_t time) :
    mOffset(offset),
    mTime(time) {}
  ogg_int64_t mOffset; // In bytes from beginning of file.
  ogg_int64_t mTime; // In milliseconds.
};

// Maps a track's serialno to its keyframe index.
typedef map<ogg_uint32_t, vector<KeyFrameInfo>*> KeyFrameIndex;

// Free's all memory stored in the key frame index.
void ClearKeyframeIndex(KeyFrameIndex& index);

// Decodes an index packet, storing the decoded index in the KeyFrameIndex,
// mapped to by the track's serialno.
bool DecodeIndex(KeyFrameIndex& index, ogg_packet* packet);

enum StreamType {
  TYPE_UNKNOWN = 0,
  TYPE_VORBIS = 1,
  TYPE_THEORA = 2,
  TYPE_KATE = 3,
  TYPE_SKELETON = 4,
  TYPE_UNSUPPORTED = 5
};

// Superclass for indexer-decoder.
class Decoder {
protected:
  ogg_stream_state mState;

  // Serial of the stream we're decoding.
  ogg_uint32_t mSerial;
  
  // Presentation time of the first frame/sample.
  ogg_int64_t mStartTime;
  
  // End time of the last frame/sample.
  ogg_int64_t mEndTime;
  
  // Initialize decoder.
  Decoder(ogg_uint32_t serial);

public:

  virtual ~Decoder();

  // Factory, creates appropriate decoder for the give beginning of stream page.
  static Decoder* Create(ogg_page* bos_page);

  // Decode page at offset, record relevant info to index keypoints.
  virtual bool Decode(ogg_page* page, ogg_int64_t offset) = 0;

  // Returns true when we've decoded all header packets.
  virtual bool GotAllHeaders() = 0;

  // Returns the keyframes for indexing. Call this after the entire stream
  // has been decoded.
  virtual const vector<KeyFrameInfo>& GetKeyframes() = 0;

  virtual StreamType Type() = 0;
  virtual const char* TypeStr() = 0;
  ogg_int64_t GetStartTime() { return mStartTime; }
  ogg_int64_t GetEndTime() { return mEndTime; }
  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) = 0;
  ogg_uint32_t GetSerial() { return mSerial; }

  // Returns the info for this stream to be stored in the skeleton fisbone
  // packet.
  virtual FisboneInfo GetFisboneInfo() = 0;

};

typedef map<ogg_uint32_t, Decoder*> DecoderMap;

#define SKELETON_VERSION_MAJOR_OFFSET 8
#define SKELETON_VERSION_MINOR_OFFSET 10
#define SKELETON_PRES_TIME_DENOM_OFFSET 20
#define SKELETON_BASE_TIME_DENOM_OFFSET 36
#define SKELETON_FIRST_NUMER_OFFSET 64
#define SKELETON_FIRST_DENOM_OFFSET 72
#define SKELETON_LAST_NUMER_OFFSET 80
#define SKELETON_LAST_DENOM_OFFSET 88
#define SKELETON_FILE_LENGTH_OFFSET 96
#define SKELETON_CONTENT_OFFSET 104

#define INDEX_SERIALNO_OFFSET 6
#define INDEX_NUM_KEYPOINTS_OFFSET 10
#define INDEX_TIME_DENOM_OFFSET 18
#define INDEX_KEYPOINT_OFFSET 26


// Skeleton decoder. Must have public interface, as we use this in the
// skeleton encoder as well.
class SkeletonDecoder : public Decoder {
public:

  SkeletonDecoder(ogg_uint32_t serial);
  virtual ~SkeletonDecoder() {}

  virtual const char* TypeStr() { return "S"; }  

  virtual StreamType Type() { return TYPE_SKELETON; }

  virtual ogg_int64_t GranuleposToTime(ogg_int64_t granulepos) {
    return -1;
  }

  virtual bool Decode(ogg_page* page, ogg_int64_t offset);

  vector<KeyFrameInfo> mDummy;

  virtual const vector<KeyFrameInfo>& GetKeyframes() {
    return mDummy;
  }

  virtual bool GotAllHeaders() { return mGotAllHeaders; } 
  virtual FisboneInfo GetFisboneInfo() { return FisboneInfo(); }

  vector<ogg_packet*> mPackets;

  // Maps track serialno to keyframe index, storing the keyframe indexes
  // as they're read from the skeleton track.
  map<ogg_uint32_t, vector<KeyFrameInfo>*> mIndex;

private:
  bool mGotAllHeaders;

  // Decoded stream version.
  ogg_uint16_t mVersionMajor;
  ogg_uint16_t mVersionMinor;
  ogg_uint32_t mVersion;
 
};

#endif // __DECODER_HPP__
