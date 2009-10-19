#ifndef __SKELETON_ENCODER_HPP__
#define __SKELETON_ENCODER_HPP__

#include "OggStream.hpp"
#include "SkeletonDecoder.hpp"

class SkeletonEncoder {
public:
  SkeletonEncoder(vector<OggStream*>& streams,
                  ogg_int64_t fileLength,
                  ogg_int64_t oldSkeletonLength);
  
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

private:

  vector<OggStream*> mStreams;
  SkeletonDecoder* mSkeletonDecoder;
  ogg_int64_t mFileLength;
  ogg_int64_t mOldSkeletonLength;
  ogg_uint32_t mSerial;
  ogg_int32_t mPacketCount;
  vector<ogg_packet*> mIndexPackets;
  vector<ogg_page*> mIndexPages;

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
