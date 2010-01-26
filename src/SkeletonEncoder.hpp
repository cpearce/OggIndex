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
 * SkeletonEncoder.hpp - Encodes skeleton track.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __SKELETON_ENCODER_HPP__
#define __SKELETON_ENCODER_HPP__

#include "Decoder.hpp"
#include "Utils.hpp"

#define SKELETON_VERSION_MAJOR 3
#define SKELETON_VERSION_MINOR 3

class SkeletonEncoder {
public:
  SkeletonEncoder(DecoderMap& decoders,
                  ogg_int64_t fileLength,
                  ogg_int64_t oldSkeletonLength,
                  ogg_uint64_t contentOffset);
  
  ~SkeletonEncoder();
  
  // Write out the new skeleton BOS page.
  void WriteBosPage(ofstream& output);
  
  // Writes out non-bos pages.
  void WritePages(ofstream& output);
  
  ogg_uint32_t GetIndexSerial() {
    return mSerial;
  }
  
  ogg_int64_t GetTrackLength();
  
  bool Encode();

  ogg_int64_t ContentOffset() { return mContentOffset; }
private:

  vector<Decoder*> mDecoders;
  SkeletonDecoder* mSkeletonDecoder;
  ogg_int64_t mFileLength;
  ogg_int64_t mOldSkeletonLength;
  ogg_uint32_t mSerial;
  ogg_int32_t mPacketCount;
  vector<ogg_packet*> mIndexPackets;
  vector<ogg_page*> mIndexPages;
  ogg_uint64_t mContentOffset;
  
  void ConstructIndexPackets();

  void ConstructPages();

  void AppendPage(ogg_page& page);

  void ClearIndexPages();
  void CorrectOffsets();

  void AddBosPacket();
  void AddEosPacket();
  void AddFisbonePackets();
  
  bool HasFisbonePackets();
};

#endif
