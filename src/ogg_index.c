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
#include <limits.h>
#include <assert.h>

#define INDEX_HEADER_SIZE 31
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
  if (index && index->stream) {
    unsigned short i;
    for (i=0; i<index->num_streams; i++) {
      free(index->stream[i].key_points);
    }
    free(index->stream);
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

static int
ensure_index_capacity_increase(ogg_index* index, int size_increase)
{
  size_t size = 0;
  ogg_int64_t new_size = 0;
  unsigned target_num_streams;
  if (!index) {
    return -1;
  }
  
  target_num_streams = index->num_streams + size_increase;
  if (target_num_streams < index->sizeof_stream) {
    /* Enough memory to accomodate size increase. */
    return 0;
  }

  /* Not enough memory in index->stream to accomodate size increase.
   * Expand by 3/2 + 1. */
  new_size = index->sizeof_stream;
  while (new_size >= 0 && new_size < target_num_streams) {
    new_size = (new_size * 3) / 2 + 1;
  }
  if (new_size < 0 ||
      new_size > INT_MAX ||
      new_size * sizeof(ogg_stream_index) > INT_MAX) {
    /* Integer overflow or ridiculous index size. Fail. */
    return -1;
  }
  size = (size_t)new_size * sizeof(ogg_stream_index);
  index->stream = (ogg_stream_index*)realloc(index->stream, size);
  if (!index->stream) {
    return -1;
  }
  index->sizeof_stream = (unsigned)new_size;
  return 0;
}

int ogg_index_decode(ogg_index* index, ogg_packet* packet)
{
  if (!index || !packet) {
    return -1;
  }

  if (ogg_index_is_loaded(index)) {
    /* Don't re-decode the index... */
    return 0;
  }

  index->num_packets++;
  if (index->num_packets == 1) {
    /* Header packet. */
    const unsigned char* p = packet->packet;
    int index_size = 0;
    
    if (!p || packet->bytes < INDEX_HEADER_SIZE) {
      return -1;
    }
    if (memcmp(p, HEADER_MAGIC, HEADER_MAGIC_LEN) != 0 ||
        p[HEADER_MAGIC_LEN] != INDEX_VERSION) {
      return -1;
    }
    p = packet->packet + HEADER_MAGIC_LEN + 1;    
    p = LEInt64(p, &index->start_time);
    p = LEInt64(p, &index->end_time);
    p = LEInt64(p, &index->segment_length);

    return 0;
  
  } else if (!packet->e_o_s) {
    /* Index packet. */
    unsigned i = 0;
    ogg_stream_index *stream_index = 0;
    int expected_packet_size = 0;
    unsigned actual_num_packets = 0;
    long size = 0;
    const unsigned char* p = packet->packet;
    
    if (!p || ensure_index_capacity_increase(index, 1) == -1) {
      return -1;
    }
    
    /* i is the index into index->stream array for the new stream. */
    i = index->num_streams;
    assert(i < index->sizeof_stream && i >= 0);
    stream_index = &index->stream[i];
    assert(stream_index);

    /* Read serialno and num keypoints from packet. */
    p = LEUint32(p, &stream_index->serialno);
    p = LEUint32(p, &stream_index->num_key_points);

    /* Check that the packet's not smaller or significantly larger than
     * we expect. These cases denote a malicious or invalid num_key_points
     * field. */
    expected_packet_size = 8 + stream_index->num_key_points * KEY_POINT_SIZE;
    actual_num_packets = (packet->bytes - 8) / KEY_POINT_SIZE;
    assert(((packet->bytes - 8) % KEY_POINT_SIZE) == 0);
    if (packet->bytes < expected_packet_size ||
        stream_index->num_key_points > actual_num_packets) {
      return -1;
    }

    /* Allocate for key points. */
    size = stream_index->num_key_points * sizeof(ogg_index_keypoint);
    stream_index->key_points = (ogg_index_keypoint*)malloc((size_t)size);
    if (!stream_index->key_points) {
      return -1;
    }
    
    /* Read in key points. */
    assert(p == packet->packet + 8);
    for (i=0; i<stream_index->num_key_points; i++) {
      assert(p < packet->packet + packet->bytes);
      p = LEInt64(p, &stream_index->key_points[i].offset);

      assert(p < packet->packet + packet->bytes);
      p = LEUint32(p, &stream_index->key_points[i].checksum);

      assert(p < packet->packet + packet->bytes);
      p = LEInt64(p, &stream_index->key_points[i].time);
    }

    /* Successfully added another stream to index. */
    index->num_streams++;
    return 0;
  }

  /* Must be eos packet. */
  assert(packet->e_o_s);
  index->loaded = 1;
  return 1;
}

int ogg_index_is_loaded(ogg_index* index)
{
  return (index && index->loaded == 1) ? 1 : 0;
}

const ogg_index_keypoint*
ogg_index_get_seek_keypoint(ogg_index* index,
                            unsigned serialno,
                            ogg_int64_t target)
{
  int start = 0;
  int end = 0;
  int i = 0;
  ogg_stream_index* stream_index = 0;
  
  if (!index ||
      !ogg_index_is_loaded(index) ||
      !index->stream ||
      target > index->end_time ||
      target < index->start_time)
  {
    return 0;
  }

  /* Find the stream index we must search in. */
  for (i=0; i<index->num_streams; i++) {
    if (index->stream[i].serialno == serialno) {
      stream_index = &index->stream[i];
      break;
    }
  }
  if (!stream_index) {
    return 0;
  }

  /* Binary search to find the last key point with time less than target. */
  end = stream_index->num_key_points - 1;
  while (end > start) {
    int mid = (start + end + 1) >> 1;
    if (stream_index->key_points[mid].time == target) {
       start = mid;
       break;
    } else if (stream_index->key_points[mid].time < target) {
      start = mid;
    } else {
      end = mid - 1;
    }
  }

  return &stream_index->key_points[start];
}
