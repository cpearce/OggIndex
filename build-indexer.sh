SRC="src/OggIndex.cpp src/OggStream.cpp src/Options.cpp src/SkeletonDecoder.cpp src/SkeletonEncoder.cpp src/Utils.cpp  src/bytes_io.c"

g++ -O2 $SRC -l ogg -l theoradec -l vorbis -o OggIndex
