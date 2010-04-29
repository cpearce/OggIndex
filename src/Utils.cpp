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
#include "Utils.hpp"

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
  delete[] p->header;
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
         packet->bytes >= (long)(HEADER_MAGIC_LEN + 8) &&
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
    ogg_int32_t ret = ogg_sync_wrote(state, bytes);
    assert(ret == 0);
  }
  return true;
}

// Returns nuber of bytes skipped to next page, or -1 on failure.
int PageSeek(ogg_sync_state* state,
             ogg_page* page,
             istream& stream,
             ogg_uint64_t& bytesRead)
{
  int retval = 0;
  ogg_int32_t bytes = 0;
  ogg_int32_t r = 0;
  ogg_uint64_t intialBytesRead = bytesRead;
  while ((r = ogg_sync_pageseek(state, page)) <= 0) {
    if (r == 0) {
      // Need to read more data to get to next page.
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
    } else {
      assert(r<0);
      // We skipped -r bytes reading up to next ogg page capture.
      retval += (-r);
    }
  }
  if (r == -1) {
    cout << "ERROR: sync failure in ReadPage()" << endl;
    return -1;
  }
  return retval;
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
    delete[] buf;
    return false;
  }
  
  file.read(buf, page->body_len);
  assert(file.gcount() == page->body_len);
  if (memcmp(buf, page->body, page->body_len) != 0) {
    cerr << "Incorrect page offset calculation for page at offset "
         << offset << endl;
    delete[] buf;
    return false;
  }

  delete[] buf;
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
  delete[] buf;
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
                        info->version_subminor;
  return (th_ver >= ver) ? 1 : 0;
}

// Returns the number of packets that start on a page.
int
CountPacketStarts(ogg_page* page)
{
  int i;
  // If we're not continuing a packet, we're at a packet start.
  int packets_start = (ogg_page_continued(page) == 0) ? 1 : 0;
  int num_lacing_vals = page->header[26];
  unsigned char* lacing_vals = &page->header[27];
  for (i=1; i<num_lacing_vals; i++) {
    if (lacing_vals[i-1] < 0xff) {
      packets_start++;
    }
  }
  return packets_start;
}

unsigned char*
WriteLEUint64(unsigned char* p, const ogg_uint64_t num)
{
  ogg_int32_t i;
  ogg_uint64_t n = num;
  assert(p);
  for (i=0; i<8; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEUint64(p) == num);
  return p + 8;
}

unsigned char*
WriteLEInt64(unsigned char* p, const ogg_int64_t num)
{
  ogg_int64_t n = num;
  ogg_int32_t i;
  assert(p);
  for (i=0; i<8; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEInt64(p) == num);
  return p + 8;
}

unsigned char*
WriteLEUint32(unsigned char* p, const ogg_uint32_t num)
{
  ogg_uint32_t n = num;
  ogg_int32_t i;
  assert(p);
  for (i=0; i<4; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEUint32(p) == num);
  return p + 4;
}

unsigned char*
WriteLEInt32(unsigned char* p, const ogg_int32_t num)
{
  ogg_int32_t n = num;
  ogg_int32_t i;
  assert(p);
  for (i=0; i<4; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEInt32(p) == num);
  return p + 4;
}

unsigned char*
WriteLEUint16(unsigned char* p, const ogg_uint16_t num)
{
  ogg_uint16_t n = num;
  assert(p);
  p[0] = (unsigned char)(n & 0xff);
  p[1] = (unsigned char)((n >> 8) & 0xff);
  assert(LEUint16(p) == num);
  return p + 2;
}

ogg_uint64_t
LEUint64(unsigned char* p)
{
  ogg_uint64_t lo = LEUint32(p);
  ogg_uint64_t hi = LEUint32(p+4);
  return lo + (hi << 32);
}

ogg_int64_t
LEInt64(unsigned char* p)
{
  ogg_int64_t lo = LEUint32(p);
  ogg_int64_t hi = LEInt32(p+4);
  return lo + (hi << 32);
};

ogg_uint32_t
LEUint32(unsigned const char* p) {
  ogg_uint32_t i =  p[0] +
                   (p[1] << 8) + 
                   (p[2] << 16) +
                   (p[3] << 24);
  return i;  
}

ogg_int32_t
LEInt32(unsigned const char* p) {
  ogg_int32_t i =  p[0] +
             (p[1] << 8) + 
             (p[2] << 16) +
             (p[3] << 24);
  return i;  
}

ogg_uint16_t
LEUint16(unsigned const char* p) {
  ogg_uint16_t i =  p[0] +
                   (p[1] << 8);
  return i;  
}

void Tokenize(const string& str,
              vector<string>& tokens,
              const string& delimiter)
{
  string::size_type matchStart = 0;
  string::size_type matchEnd = str.find(delimiter, matchStart);
  while (matchEnd != string::npos && matchStart != matchEnd)
  {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(matchStart, matchEnd - matchStart));
    matchStart = matchEnd + delimiter.size();
    matchEnd = str.find(delimiter, matchStart);
  }
}

