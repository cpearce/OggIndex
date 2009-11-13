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

bool
IsFisheadPacket(ogg_packet* packet)
{
  return packet &&
         packet->bytes > 8 &&
         memcmp(packet->packet, "fishead", 8) == 0;
}

bool
IsFisbonePacket(ogg_packet* packet)
{
  return packet &&
         packet->bytes > 8 &&
         memcmp(packet->packet, "fisbone", 8) == 0;
}

// Theora version 
int TheoraVersion(th_info* info,
                  unsigned char maj,
                  unsigned char min,
                  unsigned char sub)
{
  ogg_uint32_t ver = (maj << 16) + (min << 8) + sub;
  ogg_uint32_t th_ver = (info->version_major << 16) +
                        (info->version_minor << 8) +
                        info->version_major;
  return (th_ver >= ver) ? 1 : 0;
}