#ifndef __OGG_INDEX_H__
#define __OGG_INDEX_H__

// libogg doesn't define ogg_uint64_t on Windows yet...
#if defined WIN32 && !defined __OGG_UINT64_T__
#define __OGG_UINT64_T__
typedef unsigned __int64 ogg_uint64_t;
#endif

#define INT64_MAX LLONG_MAX
#define INT64_MIN LLONG_MIN
#define UINT64_MAX ULLONG_MAX
#define UINT64_MIN 0

// Need to index keyframe if we've not seen 1 in 64K.
#define MIN_KEYFRAME_OFFSET (64 * 1024)

// Size of one key point entry in the index.
// sizeof(ogg_int64_t) + sizeof(int32) + sizeof(ogg_int64_t)
#define KEY_POINT_SIZE 20

#define FILE_BUFFER_SIZE (1024 * 1024)

#define SKELETON_3_0_HEADER_LENGTH 64
#define SKELETON_3_1_HEADER_LENGTH 88

#define HEADER_MAGIC "index"
#define HEADER_MAGIC_LEN (sizeof(HEADER_MAGIC) / sizeof(HEADER_MAGIC[0]))

enum StreamType {
  TYPE_UNKNOWN,
  TYPE_VORBIS,
  TYPE_THEORA,
  TYPE_SKELETON,
  TYPE_UNSUPPORTED
};


#endif
