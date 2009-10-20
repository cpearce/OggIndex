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
 * OggIndex.h - generic program wide defines.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __OGG_INDEX_H__
#define __OGG_INDEX_H__

#if !defined __OGG_UINT64_T__
#if defined WIN32
#define __OGG_UINT64_T__
typedef unsigned __int64 ogg_uint64_t;
#else
#include <stdint.h>
typedef uint64_t ogg_uint64_t;
#endif
#endif

/* 
 * Size of one key point entry in the index.
 * sizeof(ogg_int64_t) + sizeof(int32) + sizeof(ogg_int64_t)
 */
#define KEY_POINT_SIZE 20

#define HEADER_MAGIC "index"
#define HEADER_MAGIC_LEN (sizeof(HEADER_MAGIC) / sizeof(HEADER_MAGIC[0]))

enum StreamType {
  TYPE_UNKNOWN,
  TYPE_VORBIS,
  TYPE_THEORA,
  TYPE_SKELETON,
  TYPE_UNSUPPORTED
};

#endif
