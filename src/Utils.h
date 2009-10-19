#ifndef __UTILS_H__
#define __UTILS_H__


#include <fstream>
#include "Options.hpp"

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
#endif
