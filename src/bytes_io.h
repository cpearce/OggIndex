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
WriteLEInt32(unsigned char* p, const ogg_uint32_t num);

unsigned char*
WriteLEUint16(unsigned char* p, const ogg_uint16_t num);


#ifdef __cplusplus
}
#endif

#endif
