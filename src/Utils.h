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
 * Utils.hpp - Generic program utility functions.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */


#ifndef __UTILS_H__
#define __UTILS_H__


#include <fstream>
#include <ogg/ogg.h>
#include <theora/codec.h>
#include "Options.hpp"
#include "KeyFrameInfo.hpp"

ogg_page*
Clone(ogg_page* p);

void
FreeClone(ogg_page* p);

void
WritePage(ofstream& output, const ogg_page& page);

// Get the length in bytes of a file. 32bit only, won't work for files larger
// than 2GB.
ogg_int64_t
FileLength(const char* aFileName);


static inline ogg_int64_t
InputFileLength() {
  return FileLength(gOptions.GetInputFilename().c_str());
}

ogg_packet*
Clone(ogg_packet* p);

bool
IsIndexPacket(ogg_packet* packet);

bool
IsPageAtOffset(const string& filename, ogg_int64_t offset, ogg_page* page);

void
CopyFileData(istream& input, ostream& output, ogg_int64_t bytesToCopy);

ogg_uint32_t
GetChecksum(ogg_page* page);

bool ReadPage(ogg_sync_state* state,
              ogg_page* page,
              istream& stream,
              ogg_uint64_t& bytesRead);

bool VerifyIndex(const string& filename);


bool IsFisheadPacket(ogg_packet* packet);

bool IsFisbonePacket(ogg_packet* packet);

bool DecodeIndex(KeyFrameIndex& index, ogg_packet* packet);

int TheoraVersion(th_info* info,
                  unsigned char maj,
                  unsigned char min,
                  unsigned char sub);

#endif

