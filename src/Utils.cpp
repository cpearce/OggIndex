#include <ogg/ogg.h>
#include <fstream>
#include <iostream>
#include <assert.h>
#include "Utils.h"
#include "OggIndex.h"

ogg_page*
Clone(ogg_page* p)
{
  ogg_page* q = new ogg_page();
  memcpy(q, p, sizeof(ogg_page));
  q->header = new unsigned char[p->header_len + p->body_len];
  q->body = q->header + q->header_len;
  memcpy(q->header, p->header, p->header_len);
  memcpy(q->body, p->body, p->body_len);
  assert(memcmp(p->header, q->header, q->header_len) == 0);
  assert(memcmp(p->body, q->body, q->body_len) == 0);
  assert(q->header_len == p->header_len);
  assert(q->body_len == p->body_len);
  return q;
}

void
FreeClone(ogg_page* p)
{
  delete p->header;
  delete p;
}


void
WritePage(ofstream& output, const ogg_page& page) {
  output.write((const char*)page.header, page.header_len);
  output.write((const char*)page.body, page.body_len); 
}


// Get the length in bytes of a file. 32bit only, won't work for files larger
// than 2GB.
ogg_int64_t
FileLength(const char* aFileName)
{
  ifstream is;
  is.open (aFileName, ios::binary);
  is.seekg (0, ios::end);
  streampos length = is.tellg();
  is.close();
  return (ogg_int64_t)length;
}

ogg_packet*
Clone(ogg_packet* p)
{
  if (!p)
    return 0;
  ogg_packet* q = new ogg_packet();
  memcpy(q, p, sizeof(ogg_packet));
  q->packet = new unsigned char[p->bytes];
  memcpy(q->packet, p->packet, p->bytes);
  return q;
}

bool
IsIndexPacket(ogg_packet* packet)
{
  return packet &&
         packet->bytes > HEADER_MAGIC_LEN + 8 &&
         memcmp(packet->packet, HEADER_MAGIC, HEADER_MAGIC_LEN) == 0;
}
