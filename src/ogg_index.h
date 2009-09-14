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
 * ogg_index.h - header file for index track decoder.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __ogg_index_h__
#define __ogg_index_h__

#ifdef __cplusplus
extern "C" {
#endif


#include <ogg/ogg.h>

/**
 * Struct representing a "keypoint". A key point has an |offset|, |time| and
 * |checksum|. This specifies that in order to render the media at |time|, the
 * last page which lies before all the key frames required to render that is
 * at |offset|, and that page has checksum |checksum|.
 */
typedef struct {

  /** Offset from start of segment. */
  ogg_int64_t offset;
  
  /** Checksum of page at offset. */
  unsigned checksum;
  
  /**
   * All key frames from pages at or after offset have an end-time greater than
   * this. Stored in milliseconds.
   */
  ogg_int64_t time;

} ogg_index_keypoint;


typedef struct {
  /** Start time of this ogg segment, in milliseconds. **/
  ogg_int64_t start_time;

  /** End time of this off segment, in milliseconds. */
  ogg_int64_t end_time;

  /** Length of this segment in bytes. */
  ogg_int64_t segment_length;

  /** Number of key points in this index. */
  unsigned num_key_points;
  
  /** Pointer to array of key points. */
  ogg_index_keypoint* key_points;
  
  /** Number of packets read in on this stream. Used during parsing. */
  unsigned num_packets;
  
} ogg_index;


/**
 * Initializes an ogg_index struct.
 * Returns 0 on success, or non-zero on failure.
 */
int ogg_index_init(ogg_index* index);

/**
 * Clears memory of an ogg index struct. Does not free the struct itself.
 * Returns 0 on success, or non-zero on failure.
 */
int ogg_index_clear(ogg_index* index);

/**
 * Decodes a packet from the index stream, building index.
 * Returns:
 * -1 - unexpected failure.
 * 0 - success, expecting more packets.
 * 1 - success, all packets read, index ready for use.
 **/
int ogg_index_decode(ogg_index* index, ogg_packet* packet);

/**
 * Gives the offset to seek to in the index segment to decode from time.
 */
ogg_int64_t ogg_index_seek_offset(ogg_index* index, ogg_int64_t time);

/**
 * Returns 1 if index is loaded and ready to use, else 0.
 */
int ogg_index_is_loaded(ogg_index* index);

/**
 * Returns a pointer to the index's key point which you should seek to in
 * order to be before all keyframes required to decode everything required
 * to render the media at 'target' milliseconds. Returns 0 if the index is
 * not loaded, or if the target lies outside of the index's range. The memory
 * returned is released in ogg_index_clear(), do not free it yourself.
 */
ogg_index_keypoint*
ogg_index_get_seek_keypoint(ogg_index* index, ogg_int64_t target);

#ifdef __cplusplus
}
#endif

#endif
