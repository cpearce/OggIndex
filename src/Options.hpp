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
 * Options.hpp - header for command line options parser.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#ifndef __OPTIONS_HPP__
#define __OPTIONS_HPP__

#include <iostream>
#include <map>

using namespace std;

#define VERSION "1.0-alpha"
#include "ogg/os_types.h"


class Options {
public:
  Options();
  ~Options();
  
  // Parses options from command line, printing errors or help to stdout.
  // Returns true on success, false on failure/error.
  bool Parse(int argc, char** argv);

  string GetInputFilename();
  string GetOutputFilename();
  bool GetDumpPackets();
  bool GetDumpKeyPackets() { return mDumpKeyPackets; }
  bool GetDumpMerge() { return mDumpMerge; }
  bool GetDumpPages();
  bool GetVerifyIndex();
  ogg_int32_t GetKeyPointInterval() { return mKeyPointInterval; }
private:

  void PrintHelp();

  // 
  bool DoParse(int argc, char** argv);
  
  bool mDumpPackets;
  bool mDumpKeyPackets;
  bool mDumpPages;
  bool mDumpMerge;
  bool mVerifyIndex;
  string mInputFilename;
  string mOutputFilename;
  ogg_int32_t mKeyPointInterval;

};

// Command line parameter parser and global program options.
// TODO: Just make all methods in Options static...
extern Options gOptions;



#endif
