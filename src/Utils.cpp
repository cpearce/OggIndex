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
 * Utils.cpp - Generic program utility functions.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */


#include <ogg/ogg.h>
#include <fstream>
#include <iostream>
#include <assert.h>
#include <string.h>
#include <limits.h>
#include "OggIndex.h"
#include "Utils.h"
#include "bytes_io.h"
#include "SkeletonDecoder.hpp"
#include "OggStream.hpp"

ogg_page*
Clone(ogg_page* p)
{
  ogg_page* q = new ogg_page();
  memcpy(q, p, sizeof(ogg_page));
  q->header = new unsigned char[p->header_len + p->body_len];
  q->body = q->header + q->header_len;
  memcpy(q->header, p->header, p->header_len);
  memcpy(q->body, p->body, p->body_len);
  assert(memcmp(p->header, q->header, q->header_len) == 0);
  assert(memcmp(p->body, q->body, q->body_len) == 0);
  assert(q->header_len == p->header_len);
  assert(q->body_len == p->body_len);
  return q;
}

void
FreeClone(ogg_page* p)
{
  delete p->header;
  delete p;
}


void
WritePage(ofstream& output, const ogg_page& page) {
  output.write((const char*)page.header, page.header_len);
  output.write((const char*)page.body, page.body_len); 
}


// Get the length in bytes of a file. 32bit only, won't work for files larger
// than 2GB.
ogg_int64_t
FileLength(const char* aFileName)
{
  ifstream is;
  is.open (aFileName, ios::binary);
  is.seekg (0, ios::end);
  streampos length = is.tellg();
  is.close();
  return (ogg_int64_t)length;
}

ogg_packet*
Clone(ogg_packet* p)
{
  if (!p)
    return 0;
  ogg_packet* q = new ogg_packet();
  memcpy(q, p, sizeof(ogg_packet));
  q->packet = new unsigned char[p->bytes];
  memcpy(q->packet, p->packet, p->bytes);
  return q;
}

bool
IsIndexPacket(ogg_packet* packet)
{
  return packet &&
         packet->bytes > HEADER_MAGIC_LEN + 8 &&
         memcmp(packet->packet, HEADER_MAGIC, HEADER_MAGIC_LEN) == 0;
}


#define FILE_BUFFER_SIZE (1024 * 1024)

// Returns nuber of bytes read.
bool ReadPage(ogg_sync_state* state,
              ogg_page* page,
              istream& stream,
              ogg_uint64_t& bytesRead)
{
  ogg_int32_t bytes = 0;
  ogg_int32_t r = 0;
  ogg_uint64_t intialBytesRead = bytesRead;
  while ((r = ogg_sync_pageout(state, page)) == 0) {
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

    ogg_int32_t ret = ogg_sync_wrote(state, bytes);
    assert(ret == 0);
  }
  if (r == -1) {
    cout << "ERROR: sync failure in ReadPage()" << endl;
    return false;
  }
  return true;
}

bool
IsPageAtOffset(const string& filename, ogg_int64_t offset, ogg_page* page)
{
  ifstream file(filename.c_str(), ios::in | ios::binary);
  assert(file);
  file.seekg((ogg_int32_t)offset, ios_base::beg);
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

void
CopyFileData(istream& input, ostream& output, ogg_int64_t bytesToCopy)
{
  assert(input.good());
  assert(output.good());
  // Copy data in chunks at most 1mb in size.
  assert(bytesToCopy >= 0);
  assert((ogg_int64_t)FILE_BUFFER_SIZE < (ogg_int64_t)INT_MAX);
  ogg_int32_t len = (ogg_int32_t)min(bytesToCopy, (ogg_int64_t)FILE_BUFFER_SIZE);
  char* buf = new char[len];
  ogg_int64_t bytesCopied = 0;
  while (bytesCopied != bytesToCopy) {
    ogg_int64_t remaining = bytesToCopy - bytesCopied;
    ogg_int32_t x = (ogg_int32_t)min(remaining, (ogg_int64_t)len);
    input.read(buf, x);
    assert(x == input.gcount());
    output.write(buf, x);
    bytesCopied += x;
  }
  delete buf;
}


ogg_uint32_t
GetChecksum(ogg_page* page)
{
  assert(page != 0);
  assert(page->header != 0);
  assert(page->header_len > 25);
  return LEUint32(page->header + 22);
}


// Reads the index out of an indexed file, and checks that the offsets
// line up with the pages they think they do, by checking the checksum.
bool VerifyIndex(const string& filename) {
  bool valid = true;
  
  ifstream input(filename.c_str(), ios::in | ios::binary);
  ogg_int64_t outputFileLength = FileLength(filename.c_str());
  ogg_sync_state state;
  ogg_int32_t ret = ogg_sync_init(&state);
  assert(ret==0);
  
  ogg_page page;
  memset(&page, 0, sizeof(ogg_page));

  ogg_uint64_t bytesRead = 0;
  ogg_uint64_t offset = 0;
  ogg_uint32_t pageNumber = 0;
  ogg_int32_t packetCount = 0;
  ogg_uint32_t skeletonSerial = 0;
  bool firstPage = true;
  
  OggStream skeleton;
 
  memset(&page, 0, sizeof(ogg_page));

  while (!skeleton.GotAllHeaders() &&
         ReadPage(&state, &page, input, bytesRead))
  {
    assert(IsPageAtOffset(filename, offset, &page));
    if (firstPage) {
      firstPage = false;
      skeletonSerial = ogg_page_serialno(&page);
      skeleton = OggStream(skeletonSerial);
    }
    if (ogg_page_serialno(&page) == skeletonSerial) {
      if (!skeleton.Decode(&page, offset)) {
        cerr << "Verification failure: Can't decode skeleton page at offset" << offset << endl;
        return false;
      }
    }
    
    ogg_uint32_t length = page.body_len + page.header_len;
    offset += length;
    memset(&page, 0, sizeof(ogg_page));
  }

  if (skeleton.mType != TYPE_SKELETON) {
    cerr << "Verification failure: First track isn't skeleton." << endl;
    return false;
  }
  SkeletonDecoder* decoder = (SkeletonDecoder*)skeleton.mDecoder;
  assert(decoder);

  // Check if the skeleton version is 3.0+, fail otherwise.
  ogg_packet* bosPacket = decoder->mPackets.front();
  if (!IsFisheadPacket(bosPacket)) {
    cerr << "Verification Failure: First packet isn't skeleton BOS packet." << endl;
    return false;
  }

  ogg_uint16_t ver_maj = LEUint16(bosPacket->packet + 8);
  ogg_uint16_t ver_min = LEUint16(bosPacket->packet + 10);
  ogg_uint32_t version = SKELETON_VERSION(ver_maj, ver_min);
  if (version < SKELETON_VERSION(3,1)) {
    cerr << "Verification Failure: skeleton version ("<< ver_maj <<"." << ver_min << ") is < 3.1." << endl;
    return false;
  }

  // Decode the 3.1 header fields, for validation later.
  // TODO: How can I validate these further?
  ogg_int64_t start_time = LEUint64(bosPacket->packet+64);
  ogg_int64_t end_time = LEUint64(bosPacket->packet+72);

  if (end_time <= start_time) {
    cerr << "Verification Failure: end_time (" << end_time << ") <= start_time (" << start_time << ")." << endl;
    return false;
  }

  ogg_int64_t length_bytes = LEUint64(bosPacket->packet+80);
  ogg_int64_t actual_file_length = FileLength(filename.c_str());
  if (length_bytes != actual_file_length) {
    cerr << "Verification Failure: index's reported file length (" << length_bytes
         << ") doesn't match actual file length (" << actual_file_length << ")." << endl;
    return false;
  }

  map<ogg_uint32_t, vector<KeyFrameInfo>*>::iterator itr = decoder->mIndex.begin();
  if (itr == decoder->mIndex.end()) {
    cerr << "WARNING: No tracks in skeleton index." << endl;
    return false;
  }

  while (valid && itr != decoder->mIndex.end()) {
    vector<KeyFrameInfo>* v = itr->second;
    ogg_uint32_t serialno = itr->first;
    itr++;

    if (v->size() == 0) {
      cerr << "WARNING: Index for track s=" << serialno << " has no keyframes" << endl;
    }
    
    cout << "Index for track s=" << serialno << endl;

    for (ogg_uint32_t i=0; valid && i<v->size(); i++) {
    
      KeyFrameInfo& keypoint = v->at(i);
    
      ogg_sync_reset(&state);
      memset(&page, 0, sizeof(ogg_page));
      char* buf = ogg_sync_buffer(&state, 8*1024);
      assert(buf);

      if (keypoint.mOffset > INT_MAX) {
        cerr << "WARNING: Can only verified up to 2^31 bytes into the file." << endl;
        break;
      }
      
      if (keypoint.mOffset > outputFileLength) {
        valid = false;
        cerr << "Verification failure: keypoint offset out of file range." << endl;
        break;
      }
      
      input.seekg((std::streamoff)keypoint.mOffset);
      ogg_int32_t bytes = (ogg_int32_t)min((ogg_int64_t)8*1024, (outputFileLength - keypoint.mOffset)); 
      assert(bytes > 0);
      input.read(buf, bytes);
      ogg_int32_t bytesToRead = input.gcount();
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
      valid = (GetChecksum(&page) == keypoint.mChecksum);
      if (!valid) {
        cerr << "Verification failure: Incorrect checksum for page at offset "
             << keypoint.mOffset << endl;
        break;
      }
      cout << "Valid page o=" << keypoint.mOffset << " c=" << keypoint.mChecksum << " t=" << keypoint.mTime << endl;
    }
    memset(&page, 0, sizeof(ogg_page));
  }

  ogg_sync_clear(&state);

  return valid;
}


bool
IsFisheadPacket(ogg_packet* packet)
{
  return packet &&
         packet->bytes > 8 &&
         memcmp(packet->packet, "fishead", 8) == 0;
}
