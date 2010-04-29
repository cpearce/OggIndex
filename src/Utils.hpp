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
#include "Decoder.hpp"

// Define ogg_uint64_t.
#if !defined __OGG_UINT64_T__
#if defined WIN32
#define __OGG_UINT64_T__
typedef unsigned __int64 ogg_uint64_t;
#else
#include <stdint.h>
typedef uint64_t ogg_uint64_t;
#endif
#endif

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

// Returns number of bytes to next page, or -1 on failure.
// Fills |page| with next page.
// Same as ReadPage(), but uses page seek instead.
int PageSeek(ogg_sync_state* state,
             ogg_page* page,
             istream& stream,
             ogg_uint64_t& bytesRead);

bool IsFisheadPacket(ogg_packet* packet);

bool IsFisbonePacket(ogg_packet* packet);

int TheoraVersion(th_info* info,
                  unsigned char maj,
                  unsigned char min,
                  unsigned char sub);


// Returns the number of packets that start on a page.
int CountPacketStarts(ogg_page* page);

#define SKELETON_VERSION(major, minor) (((major)<<16)|(minor))

// Returns true if the file has an accurate Skeleton 3.x Index track.
bool ValidateIndexedOgg(const string& filename);

ogg_uint64_t
LEUint64(unsigned char* p);

ogg_int64_t
LEInt64(unsigned char* p);

ogg_uint32_t
LEUint32(unsigned const char* p);

ogg_int32_t
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

unsigned char*
ReadVariableLength(unsigned char* p, ogg_int64_t* num);

template<class T>
unsigned char*
WriteVariableLength(unsigned char* p, const unsigned char* limit, const T n);


/*
Usage:

 vector<string> tokens;
	 string str("Split,me up!Word2 Word3.");
	 Tokenize(str, tokens, ",<'> " );
	 vector <string>::iterator iter;
	 for(iter = tokens.begin();iter!=tokens.end();iter++){
	 		cout<< (*iter) << endl;
	}

*/
void Tokenize(const string& str,
              vector<string>& tokens,
              const string& delimiter);

#endif
