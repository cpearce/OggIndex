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
 * Options.cpp - command line options parser.
 *
 * Contributor(s): 
 *   Chris Pearce <chris@pearce.org.nz>
 */

#include "Options.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>

Options::Options()
  : mDumpPackets(false)
  , mDumpKeyPackets(false)
  , mDumpPages(false)
  , mDumpMerge(false)
  , mVerifyIndex(false)
  , mSerialNo(-1)
{
}

Options::~Options()
{

}

string Options::GetInputFilename() {
  return mInputFilename;
}

string Options::GetOutputFilename() {
  return mOutputFilename;
}

bool Options::GetVerifyIndex() {
#if _DEBUG
  // Always verify under debug!
  return true;
#else
  return mVerifyIndex;
#endif
}

int Options::GetSerialNo() {
  return mSerialNo;
}

bool Options::GetDumpPackets() {
  return mDumpPackets;
}

bool Options::GetDumpPages() {
  return mDumpPages;
}

void Options::PrintHelp() {
  cout
    << "OggIndex " << VERSION << endl
    << endl
    << "Indexes an Ogg file to provide allow faster seeking." << endl
    << endl
    << "Usage:" << endl
    << "  OggIndex [-v -d -p -m -s <serialno> -o <out filename>] <in filename>" << endl
    << endl
    << "Options:" << endl
    << "  -v             --  verify the index in the output file" << endl
    << "  -d             --  dump packet info to stdout" << endl
    << "  -k             --  dump only keyframe packet info to stdout" << endl
    << "  -p             --  dump page info to stdout" << endl
    << "  -m             --  dump stream keyframe merge info to stdout" << endl
    << "  -s <serial>    --  use <serial> as the index's serialno" << endl
    << "  -o <filename>  --  use <filename> as the output filename" << endl
    << endl
    << "If no output filename is specified, the indexed ogg file is written" << endl
    << "into <in filename>.indexed.ogg" << endl 
    << endl;
}

// Returns true if a string is a command line argument identifier.
static bool
IsArgument(const char* s) {
  return strcmp(s, "-v") == 0 ||
         strcmp(s, "-d") == 0 ||
         strcmp(s, "-k") == 0 ||
         strcmp(s, "-p") == 0 ||
         strcmp(s, "-s") == 0 ||
         strcmp(s, "-o") == 0 ||
         strcmp(s, "-m") == 0;
}

static bool
FileExists(const char* filename) {
  struct stat x;
  return stat(filename, &x) != -1;
}

bool Options::Parse(int argc, char** argv) {
  if (!DoParse(argc, argv)) {
    PrintHelp();
    return false;
  }
  return true;
}

static string
OutputFilename(string input) {
  size_t dotIndex = input.rfind(".");
  if (dotIndex == string::npos) {
    // No extension? Just append ".indexed".
    return input.append(".indexed");
  }
  return input.insert(dotIndex, ".indexed");
}

bool Options::DoParse(int argc, char** argv) {

  for (int argIndex=1; argIndex < argc; argIndex++) {
    const char* arg = argv[argIndex];

    if (strcmp(arg, "-v") == 0) {
      mVerifyIndex = true;
      continue;
    }

    if (strcmp(arg, "-d") == 0) {
      if (mDumpKeyPackets) {
        cerr << "ERROR: You can't use -d and -k at the same time." << endl;
        return false;
      }
      mDumpPackets = true;
      continue;
    }

    if (strcmp(arg, "-k") == 0) {
      if (mDumpPackets) {
        cerr << "ERROR: You can't use -d and -k at the same time." << endl;
        return false;
      }
      mDumpKeyPackets = true;
      continue;
    }

    if (strcmp(arg, "-p") == 0) {
      mDumpPages = true;
      continue;
    }

    if (strcmp(arg, "-m") == 0) {
      mDumpMerge = true;
      continue;
    }

    if (strcmp(arg, "-s") == 0) {
      int serial = 0;
      if (argIndex+1 == argc || (serial = atoi(argv[argIndex+1])) == 0) {
        cerr << "ERROR: You must specify an integer serialno with '-s' argument" << endl;
        return false;
      }
      mSerialNo = serial;
      argIndex++;
      continue;
    }

    if (strcmp(arg, "-o") == 0) {
      int serial = 0;
      if (argIndex+1 == argc || IsArgument(argv[argIndex+1])) {
        cerr << "ERROR: You must specify an output filename with '-o' argument" << endl;
        return false;
      }
      mOutputFilename = argv[argIndex+1];
      argIndex++;
      continue;
    }

    if (!mInputFilename.empty()) {
      cerr << "ERROR: You cannot specify more than one input file" << endl;
      return false;
    }

    // Assume argument is input filename.
    if (!FileExists(argv[argIndex])) {
      cerr << "ERROR: Input file '" << argv[argIndex] << "' does not exist" << endl;
      return false;
    }

    mInputFilename = argv[argIndex];
  }

  if (mOutputFilename.empty()) {
    // No output filename specified, use input.indexed.extension.
    mOutputFilename = OutputFilename(mInputFilename);
    cout << "Writing output to '" << mOutputFilename.c_str() << "'" << endl;
  }

  if (mInputFilename.compare(mOutputFilename) == 0) {
    cerr << "ERROR: output filename must be different from the input filename" << endl;
    return false;
  }

  if (mInputFilename.empty()) {
    cerr << "ERROR: specify input filename" << endl;
    return false;
  }

  return true;
}
