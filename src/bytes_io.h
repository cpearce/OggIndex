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
 * bytes_io.h - reading and writing integers to memory.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __BYTES_IO_H__
#define __BYTES_IO_H__

#include <ogg/os_types.h>

// libogg doesn't define ogg_uint64_t on Windows yet...
#if defined WIN32 && !defined __OGG_UINT64_T__
#define __OGG_UINT64_T__
typedef unsigned __int64 ogg_uint64_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

ogg_uint64_t
LEUint64(unsigned char* p);

ogg_int64_t
LEInt64(unsigned char* p);

ogg_uint32_t
LEUint32(unsigned const char* p);

static ogg_int32_t
LEInt32(unsigned const char* p);

ogg_uint16_t
LEUint16(unsigned const char* p);


unsigned char*
WriteLEUint64(unsigned char* p, const ogg_uint64_t num);

unsigned char*
WriteLEInt64(unsigned char* p, const ogg_int64_t num);

unsigned char*
WriteLEUint32(unsigned char* p, const ogg_uint32_t num);

unsigned char*
WriteLEInt32(unsigned char* p, const ogg_int32_t num);

unsigned char*
WriteLEUint16(unsigned char* p, const ogg_uint16_t num);


#ifdef __cplusplus
}
#endif

#endif
