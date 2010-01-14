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
 * Validate.cpp - ogg index validator.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */
 
#include <list>
#include <limits.h>
#include <stdlib.h>
#include <ogg/ogg.h>
#include <theora/theora.h>
#include <theora/theoradec.h>
#include <vorbis/codec.h>
#include <assert.h>
#include <string.h>
#include "Utils.hpp"
#include "Decoder.hpp"

using namespace std;

class VerifyDecoder {
public:
  ogg_uint32_t mSerial;
  bool mReadHeaders;
  ogg_stream_state mStreamState;
  
  VerifyDecoder(ogg_uint32_t serial) :
    mSerial(serial),
    mReadHeaders(false)
  {
    ogg_stream_init(&mStreamState, mSerial);
  }
  
  virtual ~VerifyDecoder() {
    ogg_stream_clear(&mStreamState);
  }
  
  virtual ogg_int64_t Decode(ogg_page* page, ogg_int64_t& end_time) = 0;
  
  virtual void Reset() = 0;
  
  virtual const char* Type() { return "unknown"; }  
};


class Theora : public VerifyDecoder {
private:
  
  struct PacketInfo {
    ogg_int64_t granulepos;
    bool isKeyFrame;
  };
  
  list<PacketInfo> mBuffer;

  th_info mInfo;
  th_comment mComment;
  th_setup_info *mSetup;
  th_dec_ctx* mCtx;
  int mHeaderPacketsRead;
  ogg_int64_t mFrameDuration;
  ogg_int64_t mGranulepos;

  ogg_int64_t Time(ogg_int64_t granulepos) {
    ogg_int64_t frame_index = th_granule_frame(mCtx, granulepos);
    ogg_int64_t start_time = (1000 * mInfo.fps_denominator * frame_index) /
                              mInfo.fps_numerator;
    return start_time;
  }

  ogg_int64_t NextKeyframeTime() {
    while (mBuffer.size() > 0 && mBuffer.front().granulepos != -1) {
      PacketInfo p = mBuffer.front();
      mBuffer.pop_front();
      if (p.isKeyFrame) {
        assert(p.granulepos != -1);
        return Time(p.granulepos);
      }
    }
    return -1;
  }

  bool AllBufferedGranuleposInvalid() {
    list<PacketInfo>::iterator itr = mBuffer.begin();
    while (itr != mBuffer.end()) {
      PacketInfo* p = &(*itr);
      assert(p->granulepos == -1);
      itr++;      
    }
    return true;
  }

public:
  Theora(ogg_uint32_t serial)
    : VerifyDecoder(serial)
    , mSetup(0)
    , mHeaderPacketsRead(0)
  {
    th_info_init(&mInfo);
    th_comment_init(&mComment);
  }

  virtual ~Theora() {
    th_setup_free(mSetup);
    th_decode_free(mCtx);
  }
  
  virtual void Reset() {
    mBuffer.clear(); 
    ogg_stream_reset(&mStreamState);
    mGranulepos = -1;
  }
  
  virtual const char* Type() { return "theora"; }

  // Decode all keyframes in the given page, and return the start time of
  // the next keyframe.
  virtual ogg_int64_t Decode(ogg_page* page, ogg_int64_t& end_time) {
    end_time = -1;
    if ((ogg_uint32_t)ogg_page_serialno(page) != mSerial) {
      return -1;
    }
    ogg_int64_t start_time = INT_MAX;

    if (mGranulepos == -1 &&
        ogg_page_granulepos(page) != -1 &&
        ogg_page_packets(page) == 1 &&
        ogg_page_continued(page))
    {
      mGranulepos = ogg_page_granulepos(page);
    }

    assert(AllBufferedGranuleposInvalid());

    int shift = mInfo.keyframe_granule_shift;
    ogg_packet op;
    int ret = ogg_stream_pagein(&mStreamState, page);
    assert(ret == 0);
    while ((ret = ogg_stream_packetout(&mStreamState, &op)) != 0) {
      if (ret == -1) {
        cout << "WARNING: Theora decoder out of sync!" << endl;
        continue;
      }
      if (!mReadHeaders) {
        ogg_int32_t ret = th_decode_headerin(&mInfo,
                                             &mComment,
                                             &mSetup,
                                             &op);
        if (ret > 0) {
          mHeaderPacketsRead++;
          mReadHeaders = (mHeaderPacketsRead == 3);
          if (mReadHeaders) {
            mCtx = th_decode_alloc(&mInfo, mSetup);
            assert(mCtx != NULL);
            mFrameDuration = (1000 * mInfo.fps_denominator) / mInfo.fps_numerator;
          }
          continue;
        } else {
          cout << "th_decode_headerin returned " << ret << endl;
          return false;
        }
      }

      PacketInfo p;
      p.granulepos = op.granulepos;
      p.isKeyFrame = th_packet_iskeyframe(&op) != 0;

      // If we know the previous granulepos, we can just increment that.
      if (p.granulepos == -1 && mGranulepos != -1) {
        // Packets granulepos is the previous known granulepos incremented.
        if (p.isKeyFrame) {
          p.granulepos =
            (th_granule_frame(mCtx, mGranulepos) + 1 + TheoraVersion(&mInfo,3,2,1)) << shift;
        } else {
          p.granulepos = mGranulepos + 1;
        }
        assert(th_granule_frame(mCtx, mGranulepos) + 1 ==
               th_granule_frame(mCtx, p.granulepos));
        
      }
      mBuffer.push_back(p);

      if (p.granulepos == -1) {
        continue;
      }

      assert(op.granulepos == -1 || op.granulepos == ogg_page_granulepos(page));

      // We've got buffered packets, and the last one has a known
      // granulepos. Use its granulepos to calculate the other granulepos.          
      mGranulepos = p.granulepos;
      list<PacketInfo>::reverse_iterator rev_itr = mBuffer.rbegin();
      PacketInfo *prev = &(*rev_itr);
      rev_itr++;
      while (rev_itr != mBuffer.rend()) {
        PacketInfo* p = &(*rev_itr);
        assert(prev->granulepos != -1);
        ogg_int64_t granulepos;
        if (p->isKeyFrame) {
          granulepos =
            (th_granule_frame(mCtx, prev->granulepos) + TheoraVersion(&mInfo,3,2,1) - 1) << shift;
        } else {
          if (prev->isKeyFrame) {
            // The previous frame is a keyframe, so we can't just subtract 1
            // from the "keyframe offset" part of its granulepos, as it
            // doesn't have one! So fake it, take the keyframe offset as the
            // max possible keyframe offset. This means the granulepos claims
            // that it depends on the wrong keyframe, but at least its granule
            // number will be correct, so the times we calculate from this
            // granulepos will also be correct.
            ogg_int64_t frameno = th_granule_frame(mCtx, prev->granulepos);
            ogg_int64_t max_offset = min((frameno - 1), (ogg_int64_t)(1 << shift) - 1);
            ogg_int64_t granule = frameno +
                                  TheoraVersion(&mInfo,3,2,1) - 1 - max_offset;
            assert(granule > 0);
            granulepos = (granule << shift) + max_offset;
          } else {
            granulepos = prev->granulepos - 1;
          }
        }
        p->granulepos = granulepos;
        assert(th_granule_frame(mCtx, p->granulepos) ==
               th_granule_frame(mCtx, prev->granulepos) - 1);
        assert(Time(p->granulepos) < Time(prev->granulepos));
        prev = p;
        rev_itr++;
      }
      assert(mBuffer.front().granulepos != -1);
      assert(mBuffer.size() > 0);

      ogg_int64_t time = NextKeyframeTime();
      if (time != -1) {
        start_time = min(time, start_time);
        end_time = start_time + mFrameDuration;
        ogg_int64_t page_end_time = Time(ogg_page_granulepos(page));
        assert(time <= page_end_time);
      }        
    }
   
    return (start_time != INT_MAX) ? start_time : -1;
  }
};

class Vorbis : public VerifyDecoder {
private:
  vorbis_info mInfo;
  vorbis_comment mComment;
  vorbis_dsp_state mDsp;
  vorbis_block mBlock;
  int mHeaderPacketsRead;  

public:
  Vorbis(ogg_uint32_t serial)
    : VerifyDecoder(serial)
    , mHeaderPacketsRead(0)
  {
    vorbis_info_init(&mInfo);
    vorbis_comment_init(&mComment);    
  }

  virtual void Reset() {
    ogg_stream_reset(&mStreamState);  
    ogg_packet op;
    assert(ogg_stream_packetout(&mStreamState, &op) == 0);
    if (mReadHeaders) {
      vorbis_synthesis_restart(&mDsp);
    }
  }

  virtual const char* Type() { return "vorbis"; }

  virtual ogg_int64_t Decode(ogg_page* page, ogg_int64_t& end_time) {
    ogg_packet op;
    assert(ogg_stream_packetout(&mStreamState, &op) == 0);
    int ret = ogg_stream_pagein(&mStreamState, page);
    assert(ret == 0);
    int total_samples = 0;
    ogg_int64_t start_time = -1;
    int packet_count = 0;
    while (ogg_stream_packetout(&mStreamState, &op) == 1) {
      packet_count++;
      if (!mReadHeaders) {
        ogg_int32_t ret = vorbis_synthesis_headerin(&mInfo, &mComment, &op);
        if (ret == 0) {
          mHeaderPacketsRead++;
        }
        mReadHeaders = (mHeaderPacketsRead == 3);
        if (mReadHeaders) {
          ret = vorbis_synthesis_init(&mDsp, &mInfo);
          assert(ret == 0);
          ret = vorbis_block_init(&mDsp, &mBlock);
          assert(ret == 0);        
        }
        continue;
      }

      // We only expect the last packet in a page to have non -1 granulepos.
      assert(start_time == -1);
      int samples = 0;
      
      // Decode the vorbis to determine how many samples are in each packet.
      if (vorbis_synthesis(&mBlock, &op) == 0) {
        ret = vorbis_synthesis_blockin(&mDsp, &mBlock);
        assert(ret == 0);
      }
      while ((samples = vorbis_synthesis_pcmout(&mDsp, 0)) > 0) {
        total_samples += samples;
        ret = vorbis_synthesis_read(&mDsp, samples);
        assert(ret == 0);
      }
      // We only expect the last packet in a page to have a granulepos.
      assert(start_time == -1);
      if (op.granulepos != -1) {
        assert(op.granulepos == ogg_page_granulepos(page));
        ogg_int64_t start_granule = op.granulepos - total_samples;
        start_time = (1000 * start_granule) / mDsp.vi->rate;
        end_time = (1000 * ogg_page_granulepos(page)) / mDsp.vi->rate;
      }
    }

    return start_time;
  }
};




class Skeleton : public VerifyDecoder {
public:

  KeyFrameIndex mIndex;
  ogg_int64_t mStartTime; // in ms
  ogg_int64_t mEndTime; // in ms
  ogg_int64_t mFileLength; // in bytes.
  ogg_uint64_t mContentOffset; // in bytes.

  Skeleton(ogg_uint32_t serial)
    : VerifyDecoder(serial)
    , mStartTime(-1)
    , mEndTime(-1)
    , mFileLength(-1)
  {
  }

  ~Skeleton() {
    ClearKeyframeIndex(mIndex);
  }
  
  virtual void Reset() {
  }
  
  virtual const char* Type() { return "skeleton"; }

  virtual ogg_int64_t Decode(ogg_page* page, ogg_int64_t& end_time) {
    end_time = -1;
    int ret = ogg_stream_pagein(&mStreamState, page);
    assert(ret == 0);
    ogg_packet op;
    int packets_decoded = 0;
    while ((ret = ogg_stream_packetout(&mStreamState, &op)) != 0) {
      packets_decoded++;
      assert(!mReadHeaders);
      if (ret != 1) {
        cout << "WARNING: out of sync while reading skeleton packets!" << endl;
        continue;
      }

      if (IsFisheadPacket(&op)) {
        ogg_uint16_t ver_maj = LEUint16(op.packet + SKELETON_VERSION_MAJOR_OFFSET);
        ogg_uint16_t ver_min = LEUint16(op.packet + SKELETON_VERSION_MINOR_OFFSET);
        ogg_uint32_t version = SKELETON_VERSION(ver_maj, ver_min);
        if (version < SKELETON_VERSION(3,0) ||
            version >= SKELETON_VERSION(4,0)) { 
          cerr << "FAIL: Skeleton version " << ver_maj << "." <<ver_min   
               << " detected. I can only validate version 3.1" << endl;
          exit(-1);
        }

        // Decode the 3.x header fields, for validation later.
        // TODO: How can I validate these further?
        ogg_int64_t start_num = LEUint64(op.packet + SKELETON_FIRST_NUMER_OFFSET);
        ogg_int64_t start_denom = LEUint64(op.packet + SKELETON_FIRST_DENOM_OFFSET);
        mStartTime = (start_denom == 0) ? -1 : (start_num * 1000) / start_denom;

        ogg_int64_t last_num = LEUint64(op.packet + SKELETON_LAST_NUMER_OFFSET);
        ogg_int64_t last_denom = LEUint64(op.packet + SKELETON_LAST_DENOM_OFFSET);
        mEndTime = (last_denom == 0) ? -1 : (last_num * 1000) / last_denom;

        mContentOffset = LEUint64(op.packet + SKELETON_CONTENT_OFFSET);
        if (mContentOffset == 0) {
          cerr << "Verification Failure: content offset is 0" << endl;
          return false;
        }

        mFileLength = LEUint64(op.packet + SKELETON_FILE_LENGTH_OFFSET);

        if (mEndTime <= mStartTime) {
          cerr << "Verification Failure: end_time (" << mEndTime
               << ") <= start_time (" << mStartTime << ")." << endl;
          return false;
        }
        
        continue;
      }
      
      if (IsFisbonePacket(&op)) {
        // Nothing to check here.
        continue;
      }
      
      if (IsIndexPacket(&op)) {
        bool r = DecodeIndex(mIndex, &op);
        if (!r) {
          cerr << "FAIL: Can't parse skeleton index packet." << endl;
        }
        continue;
      }

      if (op.e_o_s) {
        assert(ogg_page_eos(page) != 0);
        mReadHeaders = true;
        continue;
      }

      cout << "WARNING: Unknown packet in skeleton track!" << endl;
    }  
    if (packets_decoded != ogg_page_packets(page)) {
      cout << "WARNING: Only decoded " << packets_decoded << " packets from "
           << "skeleon page, but expected " << ogg_page_packets(page) << endl;
    }
    return true;
  }
};

typedef map<ogg_uint32_t, VerifyDecoder*> VerifyDecoderMap;

static bool ReadAllHeaders(Theora* theora, Vorbis* vorbis, Skeleton* skeleton)
{
  return (theora || vorbis || skeleton) &&
         (!theora || theora->mReadHeaders) &&
         (!vorbis || vorbis->mReadHeaders) &&
         (!skeleton || skeleton->mReadHeaders);
}

bool ValidateIndexedOgg(const string& filename) {
  ifstream input(filename.c_str(), ios::in | ios::binary);
  ogg_sync_state state;
  ogg_int32_t ret = ogg_sync_init(&state);
  assert(ret==0);

  VerifyDecoderMap decoders;
  ogg_page page;
  memset(&page, 0, sizeof(ogg_page));
  ogg_uint64_t bytesRead = 0;
  Theora* theora = 0;
  Vorbis* vorbis = 0;
  Skeleton* skeleton = 0;
  bool index_valid = true;
  ogg_uint64_t contentOffset = 0;
  
  while (!ReadAllHeaders(theora, vorbis, skeleton) && 
         ReadPage(&state, &page, input, bytesRead))
  {
    contentOffset += page.header_len + page.body_len;
    int serialno = ogg_page_serialno(&page);
    VerifyDecoder* decoder = 0;
    if (ogg_page_bos(&page)) {
      if (page.body_len > 8 &&
          strncmp("theora", (const char*)page.body+1, 6) == 0)
      {
        theora = new Theora(serialno);
        decoders[serialno] = theora;
      } else if (page.body_len > 8 &&
                 strncmp("vorbis", (const char*)page.body+1, 6) == 0)
      {
        vorbis = new Vorbis(serialno);
        decoders[serialno] = vorbis;
      } else if (page.body_len > 8 &&
                 strncmp("fishead", (const char*)page.body, 8) == 0)
      {
        skeleton = new Skeleton(serialno);
        decoders[serialno] = skeleton;
      }
    }
    decoder = decoders[serialno];
    if (!decoder) {
      cout << "WARNING: Unknown stream type, serialno=" << serialno << endl;
      continue;
    }
    ogg_int64_t ignore;
    if (!decoder->Decode(&page, ignore)) {
      index_valid = false;
    }
  }

  if (skeleton->mContentOffset != contentOffset) {
    cerr << "FAIL: skeleton header's reported content offset (" << skeleton->mContentOffset
         << ") does not match actual content offset (" << contentOffset << ")" << endl;
    index_valid = false;
  }

  ogg_int64_t fileLength = FileLength(filename.c_str());
  if (skeleton->mFileLength != fileLength) {
    cerr << "FAIL: index's reported file length (" << skeleton->mFileLength
         << ") doesn't match file's actual length (" << fileLength << ")" << endl;
    index_valid = false;
  }

  KeyFrameIndex::iterator itr = skeleton->mIndex.begin();
  if (itr == skeleton->mIndex.end()) {
    cerr << "WARNING: No tracks in skeleton index." << endl;
  }

  while (itr != skeleton->mIndex.end()) {
    vector<KeyFrameInfo>* v = itr->second;
    ogg_uint32_t serialno = itr->first;
    itr++;
    VerifyDecoder* decoder = decoders[serialno];

    if (v->size() == 0) {
      cerr << "WARNING: Index for " << decoder->Type() << " track s="
           << serialno << " has no keyframes" << endl;
      continue;
    }

    cout << "Index for " << decoder->Type()
         << " track serialno=" << serialno << ", " << v->size() << " keypoints." << endl;

    bool valid = true;
    for (ogg_uint32_t i=0; i<v->size(); i++) {
    
      KeyFrameInfo& keypoint = v->at(i);
      if (keypoint.mOffset > INT_MAX) {
        cerr << "WARNING: Only verified up to 2^31 bytes into the file." << endl;
        break;
      }
      
      if (keypoint.mOffset > fileLength) {
        valid = false;
        cerr << "FAIL: Keypoint " << i << " offset of " << keypoint.mOffset
             << " out of file range." << endl;
        break;
      }

      // Check that the checksum of the page here matches, and that the
      // presentation time of the stream matches what's reported in the index.
      ogg_sync_reset(&state);
      if (input.eof()) {
        // Probably hit eof while reading pages for previous keypoint.
        input.close();
        input.clear();        
        input.open(filename.c_str(), ios::in | ios::binary);
      }
      input.seekg(keypoint.mOffset);
      if (!input.good()) {
        cerr << "FAIL: Can't seek to keypoint at byte offset " << keypoint.mOffset << endl;
        return false;
      }
      decoder->Reset();
      ogg_int64_t pres_time = -1;
      bool checksum_checked = false;
      ogg_int64_t page_offset = keypoint.mOffset;
      ogg_int64_t end_time = -1;
      while (pres_time  == -1) {
        if (!ReadPage(&state, &page, input, bytesRead)) {
          cerr << "FAIL: Can't read page at offset " << page_offset << endl;
          return false;
        }
        page_offset += page.header_len + page.body_len;
        if (!checksum_checked) {
          checksum_checked = true;
          if (GetChecksum(&page) != keypoint.mChecksum) {
            cerr << "FAIL: Incorrect checksum for page at byte offset "
                 << keypoint.mOffset << endl;
            valid = false;
          }
        }
        // Extract the presentation time. Decode() returns -1 if it needs
        // another page.
        if ((ogg_uint32_t)ogg_page_serialno(&page) != decoder->mSerial) {
          // Skip until we read another page on this stream.
          continue;
        }
        pres_time = decoder->Decode(&page, end_time);
        // Vorbis decoders should never return -1 for prestime, it should be
        // calculable from only one page.
        assert(decoder != vorbis || pres_time != -1);
      }

      // We consider a theora the key point is invalid if it's reported time
      // doesn't match the presentation time of the first keyframe which
      // can be decoded from this page.
      if (decoder == theora && keypoint.mTime != pres_time) {
        cerr << "FAIL: theora keypoint " << i << " for page at offset "
             << keypoint.mOffset << " reports start time of "
             << keypoint.mTime << " but should be " << pres_time << endl;
        valid = false;
      }

      // A vorbis keypoint is considered invalid if lies before the start time
      // of this page, or after the end time.
      if (decoder == vorbis &&
          (keypoint.mTime < pres_time || keypoint.mTime > end_time))
      {
        cerr << "FAIL: vorbis keypoint " << i << " for page at offset "
             << keypoint.mOffset << " reports start time of "
             << keypoint.mTime << " but should lie in the range ["
             << pres_time << "," << end_time << "]" << endl;
        valid = false;
      }
    }
    if (valid) {
      cout << "Keyframe index for " << decoder->Type() << " track s="
           << serialno << " is accurate" << endl;
    } else {
      cout << "FAIL: Keyframe index for " << decoder->Type() << " track s="
           << serialno << " is NOT accurate." << endl;
      index_valid = false;
    }
  }

  ogg_sync_clear(&state);
  
  return index_valid;
}
