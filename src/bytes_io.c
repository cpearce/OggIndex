#include <assert.h>
#include <ogg/os_types.h>
#include "bytes_io.h"


unsigned char*
WriteLEUint64(unsigned char* p, const ogg_uint64_t num)
{
  ogg_int32_t i;
  ogg_uint64_t n = num;
  assert(p);
  for (i=0; i<8; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEUint64(p) == num);
  return p + 8;
}

unsigned char*
WriteLEInt64(unsigned char* p, const ogg_int64_t num)
{
  ogg_int64_t n = num;
  ogg_int32_t i;
  assert(p);
  for (i=0; i<8; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEInt64(p) == num);
  return p + 8;
}

unsigned char*
WriteLEUint32(unsigned char* p, const ogg_uint32_t num)
{
  ogg_uint32_t n = num;
  ogg_int32_t i;
  assert(p);
  for (i=0; i<4; i++) {
    p[i] = (unsigned char)(n & 0xff);
    n >>= 8;
  }
  assert(LEUint32(p) == num);
  return p + 4;
}

unsigned char*
WriteLEUint16(unsigned char* p, const ogg_uint16_t num)
{
  ogg_uint16_t n = num;
  assert(p);
  p[0] = (unsigned char)(n & 0xff);
  p[1] = (unsigned char)((n >> 8) & 0xff);
  assert(LEUint16(p) == num);
  return p + 2;
}

ogg_uint64_t
LEUint64(unsigned char* p)
{
  ogg_uint64_t lo = LEUint32(p);
  ogg_uint64_t hi = LEUint32(p+4);
  return lo + (hi << 32);
}

ogg_int64_t
LEInt64(unsigned char* p)
{
  ogg_int64_t lo = LEUint32(p);
  ogg_int64_t hi = LEInt32(p+4);
  return lo + (hi << 32);
};

ogg_uint32_t
LEUint32(unsigned const char* p) {
  ogg_uint32_t i =  p[0] +
                   (p[1] << 8) + 
                   (p[2] << 16) +
                   (p[3] << 24);
  return i;  
}

static ogg_int32_t
LEInt32(unsigned const char* p) {
  ogg_int32_t i =  p[0] +
             (p[1] << 8) + 
             (p[2] << 16) +
             (p[3] << 24);
  return i;  
}

ogg_uint16_t
LEUint16(unsigned const char* p) {
  ogg_uint16_t i =  p[0] +
                   (p[1] << 8);
  return i;  
}
