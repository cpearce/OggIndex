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
#include "Utils.h"
#include "OggIndex.h"

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
