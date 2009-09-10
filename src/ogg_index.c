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
 * ogg_index.c - index track decoder.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#include "ogg_index.h"
#include <string.h>
#include <stdlib.h>

#define INDEX_HEADER_SIZE 35
#define KEY_POINT_SIZE 20
#define HEADER_MAGIC "index"
#define HEADER_MAGIC_LEN (sizeof(HEADER_MAGIC) / sizeof(HEADER_MAGIC[0]))
#define INDEX_VERSION 0x01


int ogg_index_init(ogg_index* index)
{
  if (index)
    memset(index, 0, sizeof(ogg_index));
  return 0;
}

int ogg_index_clear(ogg_index* index)
{
  if (index && index->key_points) {
    free(index->key_points);
  }  
  return 0;
}

static const unsigned char*
LEUint32(const unsigned char* p, unsigned* n) {
  *n = p[0] +
      (p[1] << 8) + 
      (p[2] << 16) +
      (p[3] << 24);
  return p + 4;
}

static const unsigned char*
LEInt64(const unsigned char* p, ogg_int64_t* n)
{
  unsigned lo = 0, hi = 0;
  p = LEUint32(p, &lo);
  p = LEUint32(p, &hi);
  *n = (ogg_int64_t)lo | ((ogg_int64_t)hi << 32);
  return p;
}

int ogg_index_decode(ogg_index* index, ogg_packet* packet)
{
  if (!index)
    return -1;
  index->num_packets++;
  if (index->num_packets > 3) {
    /* Should only have 3 packets, header, index, eos. */
    return -1;
  }
  if (index->num_packets == 1) {
    /* Header packet. */
    const unsigned char* p = packet->packet;
    if (packet->bytes < INDEX_HEADER_SIZE) {
      return -1;
    }
    if (memcmp(p, HEADER_MAGIC, HEADER_MAGIC_LEN) != 0 ||
        p[6] != INDEX_VERSION) {
      return -1;
    }
    p = packet->packet + 7;    
    p = LEInt64(p, &index->start_time);
    p = LEInt64(p, &index->end_time);
    p = LEInt64(p, &index->segment_length);
    p = LEUint32(p, &index->num_key_points);

    return 0;
  }
  if (index->num_packets == 2) {
    /* Index packet. */
    unsigned i;
    unsigned num_bytes = index->num_key_points * KEY_POINT_SIZE;
    size_t size = index->num_key_points * sizeof(ogg_index_keypoint);
    const unsigned char* p = packet->packet;
    if (packet->bytes != num_bytes) {
      return -1;
    }
    index->key_points = (ogg_index_keypoint*)malloc(size);
    if (!index->key_points) {
      return -1;
    }
    for (i=0; i<index->num_key_points; i++) {
      p = LEInt64(p, &index->key_points[i].offset);
      p = LEUint32(p, &index->key_points[i].checksum);
      p = LEInt64(p, &index->key_points[i].time);
    }
    return 0;
  }
  if (index->num_packets == 3) {
    /* eos packet. */
    return 1;
  }
  return -1;
}

int ogg_index_is_loaded(ogg_index* index)
{
  return index && index->num_packets == 3;
}
