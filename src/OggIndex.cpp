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
 * OggIndex.cpp - ogg file indexer main file.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#include <iostream>
#include <fstream>

#include <assert.h>
#include <time.h>
#include <map>
#include <vector>
#include <iomanip>
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "OggIndex.h"
#include "Options.hpp"
#include "Decoder.hpp"
#include "SkeletonDecoder.hpp"
#include "SkeletonEncoder.hpp"
#include "Utils.h"

using namespace std;

bool ValidateIndexedOgg(const string& filename);


int main(int argc, char** argv) 
{
  if (!gOptions.Parse(argc, argv)) {
    return -1;
  }

  string filename = gOptions.GetInputFilename();
  ifstream input(filename.c_str(), ios::in | ios::binary);
  ogg_sync_state state;
  ogg_int32_t ret = ogg_sync_init(&state);
  assert(ret==0);

  StreamMap streams;
  ogg_page page;
  memset(&page, 0, sizeof(ogg_page));

  time_t startTime = time(0);

  ogg_uint64_t bytesRead = 0;
  ogg_uint64_t offset = 0;
  ogg_uint32_t pageNumber = 0;
  ogg_uint64_t insertionPoint = -1;
  ogg_uint64_t skeletonBosPageLen = 0;
  bool gotAllHeaders = false;
  ogg_uint64_t endOfHeaders = 0;
  ogg_uint64_t oldSkeletonLength = 0;
  
  OggStream *skeleton = 0;
  
  // We store all non-skeleton header pages in the order in which we read them,
  // so that we can rewrite them easily.
  vector<ogg_page*> headerPages;

  while (ReadPage(&state, &page, input, bytesRead)) {
    assert(IsPageAtOffset(filename, offset, &page));
    pageNumber++;
    ogg_uint32_t serial = ogg_page_serialno(&page);
    OggStream* stream = 0;
    if (ogg_page_bos(&page)) {
      stream = new OggStream(serial);
      streams[serial] = stream;
    } else {
      stream = streams[serial];
    }

    ogg_uint32_t length = page.body_len + page.header_len;
    stream->Decode(&page, offset);
    ogg_uint32_t oldSkelentonLength = 0;
    
    if (!gotAllHeaders) {
      vector<OggStream*> sv = GetStreamVector(streams);
      gotAllHeaders = true;
      for (ogg_uint32_t i=0; i<sv.size(); i++) {
        if (!sv[i]->GotAllHeaders()) {
          gotAllHeaders = false; 
          break;
        }
      }
      if (stream->mType == TYPE_SKELETON) {
        oldSkeletonLength += length;
      } else {
        // Record all header pages except skeleton. We'll just rewrite the
        // skeleton, and the skeleton decoder will remember its packets.
        headerPages.push_back(Clone(&page));
      }
      if (gotAllHeaders) {
        endOfHeaders = offset + length;
      }
    }

    if (stream->mType == TYPE_UNKNOWN) {
      cerr << "FAIL: Unhandled content type in stream serialno="
           << stream->mSerial << " aborting indexing!" << endl;
      return -1;
    }
    
    if (gOptions.GetDumpPages()) {
      ogg_int64_t granulepos = ogg_page_granulepos(&page);
      cout << "[" << stream->TypeStr() << "] page @" << offset
           << " length=" << length << " granulepos=" << granulepos 
           << " time=" << stream->GranuleposToTime(granulepos) << "ms"
           << " s=" << serial << endl;
    }

    offset += length;
    memset(&page, 0, sizeof(ogg_page));
  }

  const ogg_uint64_t fileLength = bytesRead;
  assert(input.eof());
  if (offset != fileLength) {
    cerr << "WARNING: Ogg page lengths don't sum to file length!" << endl;
  }
  
  assert(fileLength == InputFileLength());
  
  ogg_sync_clear(&state);
  
  vector<OggStream*> sv = GetStreamVector(streams);

  SkeletonEncoder encoder(sv, fileLength, oldSkeletonLength);

  // Reopen the file so we can write it out with the index.
  input.close();
  input.clear();
  input.open(filename.c_str(), ios::in | ios::binary);
  assert(input.good());
  
  // Open output file.
  ofstream output(gOptions.GetOutputFilename().c_str(), ios::out | ios::binary);
  
  // Encode the new skeleton track.
  encoder.Encode();
  
  // Write out the new skeleton BOS page.
  encoder.WriteBosPage(output);
  
  // Write out all the other tracks' header pages.
  for (ogg_uint32_t i=0; i<headerPages.size(); i++) {
    WritePage(output, *headerPages[i]);
    FreeClone(headerPages[i]);
  }

  // Write out remaining skeleton pages.
  encoder.WritePages(output);

  // Copy content pages.
  input.seekg((std::streamoff)endOfHeaders);
  CopyFileData(input, output, fileLength - endOfHeaders);
  
  output.close();
  input.close();

  if (gOptions.GetVerifyIndex()) {
    if (!ValidateIndexedOgg(gOptions.GetOutputFilename())) {
      cerr << "FAIL: Verification of the index failed!" << endl;
    } else {
      cout << "SUCCESS: index passes verification." << endl;
    }
  }
  
  ogg_int64_t trackLength = encoder.GetTrackLength();
  cout << "Index track length: " << trackLength << " bytes, "
       << ((float)trackLength / (float)(fileLength + trackLength)) * 100.0
       << "% overhead" << endl;
  return 0;
}
