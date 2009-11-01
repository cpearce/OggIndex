SRC="src/OggIndexValid.cpp src/OggStream.cpp src/Options.cpp src/SkeletonDecoder.cpp src/SkeletonEncoder.cpp src/Utils.cpp  src/bytes_io.c"

g++ -g $SRC -l ogg -l theoradec -l vorbis -o OggIndexValid
